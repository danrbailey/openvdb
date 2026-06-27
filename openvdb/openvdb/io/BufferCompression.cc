// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#include "BufferCompression.h"

#include <openvdb/Exceptions.h>

#include <zstd.h>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace io {


BufferReader
IOBuffers::uncompressedReader()
{
    OPENVDB_ASSERT(!uncompressedBuffer.empty());
    return BufferReader(uncompressedBuffer);
}

void
IOBuffers::read(std::istream& is, bool compressed)
{
    read(is, -1, compressed);
}

void
IOBuffers::read(std::istream& is, int64_t size, bool compressed)
{
    if (size < 0) {
        is.read(reinterpret_cast<char*>(&size), sizeof(int64_t));
    } else {
        is.ignore(sizeof(int64_t));
    }

    if (compressed) {
        // Grow the buffer only when the actual compressed block is larger than
        // the current capacity, then read exactly the block's bytes.
        if (size > static_cast<int64_t>(compressedBuffer.capacity())) {
            compressedBuffer.reserve(size);
        }
        compressedBuffer.resize(size);
        is.read(compressedBuffer.data(), size);
    } else {
        uncompressedBuffer.resize(size);
        is.read(uncompressedBuffer.data(), size);
    }
}

void
IOBuffers::skip(std::istream& is, int64_t size, bool seekable)
{
    if (size < 0) {
        is.read(reinterpret_cast<char*>(&size), sizeof(int64_t));
    } else {
        is.ignore(sizeof(int64_t));
    }

    // Skip the payload without faulting it into memory. Seeking keeps a
    // throttled (bandwidth-limited) stream from being charged for the dropped
    // bytes; ignore() is the forward-only fallback for non-seekable streams.
    if (seekable) {
        is.seekg(size, std::ios_base::cur);
    } else {
        is.ignore(size);
    }
}

void
IOBuffers::setCompressed(DefaultInitVector&& buffer)
{
    compressedBuffer = std::move(buffer);
    compressedBuffer.clear();
}

DefaultInitVector
IOBuffers::releaseCompressed()
{
    return DefaultInitVector{std::move(compressedBuffer)};
}

void
IOBuffers::decompress(bool keepCompressed)
{
    if (!compressedBuffer.empty()) {
        const unsigned long long uncompressedSize = ZSTD_getFrameContentSize(
            compressedBuffer.data(), compressedBuffer.size());
        if (uncompressedSize == ZSTD_CONTENTSIZE_ERROR) {
            OPENVDB_THROW(IoError, "zstd decompression failed: malformed frame header");
        }
        if (uncompressedSize == ZSTD_CONTENTSIZE_UNKNOWN) {
            OPENVDB_THROW(IoError, "zstd decompression failed: content size not stored in frame");
        }
        uncompressedBuffer.resize(static_cast<size_t>(uncompressedSize));
        size_t decompSize = ZSTD_decompress(
            uncompressedBuffer.data(), uncompressedSize,
            compressedBuffer.data(), compressedBuffer.size());
        if (ZSTD_isError(decompSize)) {
            OPENVDB_THROW(IoError, "zstd decompression failed: "
                << ZSTD_getErrorName(decompSize));
        }
        if (!keepCompressed) {
            compressedBuffer.clear();
        }
    }
}

void
IOBuffers::setCompressionParameters(const ZstdCompressionParameters& parameters)
{
    mCompressionParameters = parameters;
}

void
IOBuffers::compress(bool keepUncompressed)
{
    const size_t uncompressedSize = size_t(uncompressedBuffer.size());
    const size_t capacity = ZSTD_compressBound(uncompressedSize);
    // Size to the compress bound first, then shrink to the actual frame size:
    // compressedBuffer holds the raw zstd frame (no size prefix), matching the
    // layout decompress() and read() expect.
    compressedBuffer.resize(capacity);

    // Use the advanced API so the individual ZSTD_compressionParameters can be
    // overridden. Setting the compression level to 3 first makes zstd derive
    // every parameter from preset 3; the explicit overrides below then replace
    // only the requested fields, leaving the rest at their preset-3 defaults.
    ZSTD_CCtx* const context = ZSTD_createCCtx();
    if (context == nullptr) {
        OPENVDB_THROW(IoError, "zstd compression failed: could not create context");
    }
    ZSTD_CCtx_setParameter(context, ZSTD_c_compressionLevel, 3);
    const ZstdCompressionParameters& parameters = mCompressionParameters;
    if (parameters.windowLog)    ZSTD_CCtx_setParameter(context, ZSTD_c_windowLog,    *parameters.windowLog);
    if (parameters.hashLog)      ZSTD_CCtx_setParameter(context, ZSTD_c_hashLog,      *parameters.hashLog);
    if (parameters.chainLog)     ZSTD_CCtx_setParameter(context, ZSTD_c_chainLog,     *parameters.chainLog);
    if (parameters.searchLog)    ZSTD_CCtx_setParameter(context, ZSTD_c_searchLog,    *parameters.searchLog);
    if (parameters.minMatch)     ZSTD_CCtx_setParameter(context, ZSTD_c_minMatch,     *parameters.minMatch);
    if (parameters.targetLength) ZSTD_CCtx_setParameter(context, ZSTD_c_targetLength, *parameters.targetLength);
    if (parameters.strategy)     ZSTD_CCtx_setParameter(context, ZSTD_c_strategy,     *parameters.strategy);

    const size_t frameSize = ZSTD_compress2(
        context,
        compressedBuffer.data(), capacity,
        uncompressedBuffer.data(), uncompressedSize);
    ZSTD_freeCCtx(context);
    if (ZSTD_isError(frameSize)) {
        OPENVDB_THROW(IoError, "zstd compression failed: "
            << ZSTD_getErrorName(frameSize));
    }
    compressedBuffer.resize(frameSize);
    if (!keepUncompressed) {
        uncompressedBuffer.clear();
    }
}

BufferWriter
IOBuffers::allocate(int64_t size)
{
    uncompressedBuffer.resize(size_t(size));
    return BufferWriter(uncompressedBuffer);
}

int64_t
IOBuffers::write(std::ostream& os, bool compressed)
{
    if (compressed) {
        // Reuse an already-populated compressed frame (e.g. from a prior
        // compress() call run ahead of the I/O); otherwise compress now.
        if (compressedBuffer.empty()) {
            compress(/*keepUncompressed=*/true);
        }
        const int64_t compressedSize = static_cast<int64_t>(compressedBuffer.size());
        os.write(reinterpret_cast<const char*>(&compressedSize), sizeof(int64_t));
        os.write(compressedBuffer.data(), compressedSize);
        return compressedSize;
    } else {
        // Emit the same [int64 size][payload] framing as the compressed branch
        // so IOBuffers::read can recover the block size from the inline prefix
        // on a forward-only (non-seekable) stream. The returned value is the
        // payload size only, excluding this prefix (matching the compressed
        // branch, which returns the compressed payload size).
        const int64_t uncompressedSize = static_cast<int64_t>(uncompressedBuffer.size());
        os.write(reinterpret_cast<const char*>(&uncompressedSize), sizeof(int64_t));
        os.write(uncompressedBuffer.data(), uncompressedSize);
        return uncompressedSize;
    }
}

void
IOBuffers::clear()
{
    compressedBuffer.clear();
    uncompressedBuffer.clear();
}

void
IOOffsetTable::write(std::ostream& os, bool compressed) const
{
    io::IOBuffers buffer;
    io::BufferWriter bufferWriter = buffer.allocate(int64_t((offsets.size() + 1) * sizeof(int64_t)));
    bufferWriter.writeAt<int64_t>(0, int64_t(offsets.size()));
    for (size_t i = 0; i < offsets.size(); ++i) {
        bufferWriter.writeAt<int64_t>((i + 1) * sizeof(int64_t), offsets[i]);
    }
    // The trailer holds the full size of the buffer block just written so the
    // read side can seek back over it. IOBuffers::write returns the payload size
    // and always emits an int64 size prefix ahead of it, so the framed total is
    // payload + prefix. Computed from the return value rather than tellp() so
    // this works on streams whose put position is not queryable (e.g. pipes).
    const int64_t payloadSize = buffer.write(os, compressed);
    const int64_t offset = payloadSize + int64_t(sizeof(int64_t));
    os.write(reinterpret_cast<const char*>(&offset), sizeof(int64_t));
}

void
IOOffsetTable::read(std::istream& is, bool compressed)
{
    int64_t offset = 0;
    is.read(reinterpret_cast<char*>(&offset), sizeof(int64_t));
    // The trailer holds the byte size of the preceding buffer block. Seek back
    // over both the buffer block and the trailer just read to reach the start
    // of the buffer block.
    is.seekg(-(offset + int64_t(sizeof(int64_t))), std::ios_base::cur);

    io::IOBuffers buffer;
    // offset covers [compressedSize prefix][payload]; read(is, compressed)
    // consumes both without pulling in the trailer integer that follows.
    buffer.read(is, compressed);
    buffer.decompress();
    io::BufferReader offsetTableReader = buffer.uncompressedReader();
    int64_t size = offsetTableReader.readAt<int64_t>(0);
    offsets.resize(size);
    for (int i = 0; i < size; ++i) {
        offsets[i] = offsetTableReader.readAt<int64_t>((i + 1) * sizeof(int64_t));
    }
}

bool isStreamSeekable(std::istream& is)
{
    OPENVDB_ASSERT(!is.fail() && !is.eof());
    is.seekg(0, std::ios::cur);
    bool ok = !is.fail();
    if (!ok) is.clear();
    return ok;
}

} // namespace io
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
