// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENVDB_IO_BUFFERCOMPRESSION_HAS_BEEN_INCLUDED
#define OPENVDB_IO_BUFFERCOMPRESSION_HAS_BEEN_INCLUDED

#include <openvdb/version.h>
#include <openvdb/util/Assert.h>

#include <tbb/concurrent_queue.h>

#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace io {

namespace internal {

/// @brief Allocator that default-initializes elements instead of
/// value-initializing them. For @c char this means resize()/reserve() leave
/// new bytes with indeterminate (garbage) values rather than zeroing them,
/// avoiding the wasted zero-fill. Callers must therefore not assume the
/// unwritten tail of the buffer is zero.
struct DefaultInitAllocator : std::allocator<char> {
    template <typename U>
    struct rebind { using other = DefaultInitAllocator; };

    template <typename U>
    void construct(U* ptr)
        noexcept(std::is_nothrow_default_constructible_v<U>)
    {
        ::new(static_cast<void*>(ptr)) U;  // default-init, not value-init
    }
    template <typename U, typename... Args>
    void construct(U* ptr, Args&&... args) {
        std::allocator_traits<std::allocator<char>>::construct(
            static_cast<std::allocator<char>&>(*this),
            ptr, std::forward<Args>(args)...);
    }
}; // struct DefaultInitAllocator

} // namespace internal

using DefaultInitVector = std::vector<char, internal::DefaultInitAllocator>;
using DefaultInitVectorPool = tbb::concurrent_bounded_queue<DefaultInitVector>;


struct BufferReader
{
    explicit BufferReader(const DefaultInitVector& buffer)
        : mBuffer(buffer) { }

    template <typename T>
    T readAt(size_t offset) const
    {
        OPENVDB_ASSERT(offset + sizeof(T) <= mBuffer.size());
        T value;
        std::memcpy(&value, mBuffer.data() + offset, sizeof(T));
        return value;
    }

    template <typename T>
    void readIntoVector(std::vector<T>& vector, size_t offset) const
    {
        OPENVDB_ASSERT(offset + vector.size() * sizeof(T) <= mBuffer.size());
        std::memcpy(vector.data(), mBuffer.data() + offset, vector.size() * sizeof(T));
    }

private:
    const DefaultInitVector& mBuffer;
}; // struct BufferReader


struct BufferWriter
{
    explicit BufferWriter(DefaultInitVector& buffer)
        : mBuffer(buffer) { }

    template <typename T>
    void writeAt(size_t offset, const T& value) const
    {
        OPENVDB_ASSERT(offset + sizeof(T) <= mBuffer.size());
        std::memcpy(mBuffer.data() + offset, &value, sizeof(T));
    }

    template <typename T>
    void writeFromVector(const std::vector<T>& vector, size_t offset) const
    {
        OPENVDB_ASSERT(offset + vector.size() * sizeof(T) <= mBuffer.size());
        std::memcpy(mBuffer.data() + offset, vector.data(), vector.size() * sizeof(T));
    }

private:
    DefaultInitVector& mBuffer;
}; // struct BufferWriter


/// @brief Optional overrides for the advanced zstd compression parameters.
/// Each unset field (std::nullopt, the default) falls back to the value that
/// zstd derives for compression preset 3. Field names mirror the members of
/// @c ZSTD_compressionParameters; the zstd CLI exposes them under the shorter
/// aliases shown in the comments (e.g. @c --zstd=wlog=20,strat=6).
/// No range checking is performed - values are applied to the compressor
/// directly, so out-of-range values surface as zstd errors at compress() time.
struct ZstdCompressionParameters
{
    std::optional<int> windowLog;     ///< wlog
    std::optional<int> hashLog;       ///< hlog
    std::optional<int> chainLog;      ///< clog
    std::optional<int> searchLog;     ///< slog
    std::optional<int> minMatch;      ///< mml
    std::optional<int> targetLength;  ///< tlen
    std::optional<int> strategy;      ///< strat
}; // struct ZstdCompressionParameters


struct OPENVDB_API IOBuffers
{
    IOBuffers() = default;

    BufferReader uncompressedReader();

    /// @brief Set the advanced zstd parameters applied by subsequent
    /// compress() calls. These persist on this IOBuffers until changed again,
    /// so a single set of parameters can drive any number of compress() calls.
    void setCompressionParameters(const ZstdCompressionParameters& parameters);

    void read(std::istream& is, bool compressed);
    /// @param size Block byte count, or a negative value to read the size
    /// prefix from @a is (same as read(is, compressed)).
    void read(std::istream& is, int64_t size, bool compressed);

    /// @brief Advance @a is past one serialized buffer block (the @c int64 size
    /// prefix plus its payload) without reading the payload into memory. This
    /// mirrors the framing read() consumes, so it can stand in for a read() call
    /// when the block's contents are not needed (e.g. a truncated byte plane).
    /// @param size Payload byte count, or a negative value to read the size
    /// prefix from @a is to discover it (same convention as read()).
    /// @param seekable Whether @a is supports seeking. When true the payload is
    /// skipped with seekg() so a bandwidth-limited stream is not charged for the
    /// dropped bytes; when false it is consumed with ignore().
    static void skip(std::istream& is, int64_t size, bool seekable);

    /// @brief Install a pre-faulted pooled compressed buffer so a subsequent
    /// read() reuses its allocation instead of faulting fresh pages.
    void setCompressed(DefaultInitVector&& buffer);

    /// @brief Hand the compressed buffer back for recycling. After this call
    /// the IOBuffers no longer owns it.
    DefaultInitVector releaseCompressed();

    /// @brief Inflate compressedBuffer into uncompressedBuffer. When @a keepCompressed
    /// is true the compressed allocation is kept so it can be recycled via
    /// releaseCompressed(); otherwise it is freed.
    void decompress(bool keepCompressed = false);

    /// @brief Compress uncompressedBuffer into compressedBuffer (the raw zstd
    /// frame, no size prefix - matching the layout read() expects). When
    /// @a keepUncompressed is true the uncompressed allocation is kept;
    /// otherwise it is freed. This is the compression half of write() split
    /// out so it can be run ahead of (and concurrently with) the I/O.
    void compress(bool keepUncompressed = false);

    BufferWriter allocate(int64_t size);

    /// @brief Write this buffer to @a os. Returns the total number of bytes
    /// written. When @a compressed is true and compressedBuffer is already
    /// populated (e.g. by a prior compress() call) the existing compressed
    /// bytes are written as-is; otherwise compress() is called first.
    int64_t write(std::ostream& os, bool compressed);

    void clear();

    DefaultInitVector compressedBuffer;
    DefaultInitVector uncompressedBuffer;

private:
    ZstdCompressionParameters mCompressionParameters;
}; // struct IOBuffers


struct OPENVDB_API IOOffsetTable
{
    /// @brief Iterator over block sizes. When the table is empty (e.g. it was
    /// never read on a non-seekable stream), evaluates to false and next()
    /// returns -1 without advancing.
    struct Iterator
    {
        using DequeIter = std::deque<int64_t>::iterator;

        Iterator() = default;
        Iterator(DequeIter cur, DequeIter end) : mCur(cur), mEnd(end) {}

        explicit operator bool() const { return mCur != mEnd; }

        int64_t next()
        {
            if (mCur == mEnd) return -1;
            return *mCur++;
        }

    private:
        DequeIter mCur = {};
        DequeIter mEnd = {};
    }; // struct Iterator

    IOOffsetTable() = default;

    void append(int64_t offset) { offsets.push_back(offset); }
    void clear() { offsets.clear(); }
    int64_t size() const { return offsets.size(); }

    void write(std::ostream& os, bool compressed) const;
    void read(std::istream& is, bool compressed);

    Iterator iterator() { return {offsets.begin(), offsets.end()}; }

private:
    std::deque<int64_t> offsets;
}; // struct IOOffsetTable


OPENVDB_API bool isStreamSeekable(std::istream& is);


} // namespace io
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_IO_BUFFERCOMPRESSION_HAS_BEEN_INCLUDED
