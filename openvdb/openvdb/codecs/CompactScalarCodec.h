// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENVDB_IO_CODECS_COMPACT_SCALAR_CODEC_HAS_BEEN_INCLUDED
#define OPENVDB_IO_CODECS_COMPACT_SCALAR_CODEC_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/io/Codec.h>
#include <openvdb/tools/Count.h>
#include <openvdb/util/CpuTimer.h>
#include <openvdb/util/logging.h>

#include "CompactTopologyCodec.h"

#include <optional>
#include <string>
#include <thread>  // for std::thread::hardware_concurrency()
#include <type_traits>  // for std::is_floating_point

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace codecs {

namespace internal {

inline bool isLittleEndian()
{
    const uint16_t probe = 0x0001;
    return *reinterpret_cast<const uint8_t*>(&probe) == 0x01;
}

// Map an on-disk "plane" index to the corresponding in-memory byte index of a
// value. Plane 0 is the most-significant byte (sign/exponent for IEEE floats);
// the final plane is the least-significant mantissa byte, which is the safe one
// to truncate for lossy storage. This is the only endian-aware code in the
// plane path: callers always think in MSB-first plane order.
inline size_t planeToByteIndex(size_t plane, size_t numBytes)
{
    // little-endian: byte[numBytes-1] is most significant -> reverse
    // big-endian:    byte[0]          is most significant -> identity
    return isLittleEndian() ? (numBytes - 1 - plane) : plane;
}

// Access a single byte of a value's in-memory object representation, given an
// already-resolved (endian-aware) byte index from planeToByteIndex. Returning a
// reference lets callers both read a byte (gather, on the write path) and write
// one in place (scatter, on the read path). Single-byte in-place access is what
// makes the per-plane ops safe to run concurrently: each plane touches a
// distinct byte of every value, so a read-modify-write of the whole value (which
// would race the other planes) is never needed. This is the one spot the
// reinterpret_cast byte-poke lives; every plane path goes through it.
template <typename ValueT>
inline uint8_t& valueByte(ValueT& value, size_t byteIndex)
{
    return reinterpret_cast<uint8_t*>(&value)[byteIndex];
}

template <typename ValueT>
inline const uint8_t& valueByte(const ValueT& value, size_t byteIndex)
{
    return reinterpret_cast<const uint8_t*>(&value)[byteIndex];
}

template <typename ValueT>
inline void writeValuePlanes(const std::vector<io::BufferWriter>& writers, size_t valueIndex,
                             const ValueT& value)
{
    for (size_t plane = 0; plane < sizeof(ValueT); ++plane)
        writers[plane].writeAt<uint8_t>(valueIndex, valueByte(value, planeToByteIndex(plane, sizeof(ValueT))));
}

template <typename ValueT>
inline ValueT readValuePlanes(const std::vector<io::BufferReader>& readers, size_t valueIndex,
                              size_t keptPlanes = sizeof(ValueT))
{
    // each plane writes a distinct byte and the loop covers every plane, so all
    // sizeof(ValueT) bytes of the (initially indeterminate) value are written
    // exactly once before it is returned. when keptPlanes < sizeof(ValueT) the
    // leading keptPlanes byte planes (MSB first) are read from their buffers and
    // the remaining (least-significant mantissa) planes are zeroed, so @a readers
    // only needs the keptPlanes leading entries.
    ValueT value;
    for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
        valueByte(value, planeToByteIndex(plane, sizeof(ValueT))) = plane < keptPlanes
            ? readers[plane].readAt<uint8_t>(valueIndex)
            : uint8_t(0);
    }
    return value;
}

// copied from TopologyCodec.h for now

template <typename TreeT>
struct SetVoxelsToBackground
{
    using RootT = typename TreeT::RootNodeType;
    using LeafT = typename TreeT::LeafNodeType;
    using ValueT = typename TreeT::ValueType;

    explicit SetVoxelsToBackground(const ValueT& background)
        : mBackground(background) { }

    bool operator()(RootT& root, size_t) const
    {
        for (auto it = root.beginValueAll(); it; ++it) {
            it.setValue(mBackground);
        }
        return true;
    }

    template<typename NodeT>
    bool operator()(NodeT& node, size_t) const
    {
        for (Index i = 0; i < NodeT::NUM_VALUES; ++i) {
            if (node.isChildMaskOff(i)) {
                node.setValueOnlyUnsafe(i, mBackground);
            }
        }
        return true;
    }

    bool operator()(LeafT& leaf, size_t) const
    {
        for (auto iter = leaf.beginValueAll(); iter; ++iter) {
            iter.setValue(mBackground);
        }
        return false;
    }

private:
    const ValueT mBackground;
}; // struct SetVoxelsToBackground

template<typename TreeT>
void setVoxelsToBackground(TreeT& tree)
{
    const typename TreeT::ValueType& background = tree.root().background();
    SetVoxelsToBackground<TreeT> op(background);
    tree::DynamicNodeManager<TreeT> nodeManager(tree);
    nodeManager.foreachTopDown(op);
}

// Read a single advanced zstd parameter override from an Int32 grid metadata
// value named @a key, returning std::nullopt when the key is absent (so the
// compressor keeps its preset-3 default for that parameter).
inline std::optional<int> readZstdParameter(const GridBase& grid, const std::string& key)
{
    if (auto meta = grid.getMetadata<Int32Metadata>(key)) {
        return meta->value();
    }
    return std::nullopt;
}

// Build a ZstdCompressionParameters from the seven Int32 grid metadata values
// named "<prefix>wlog", "<prefix>hlog", ... (the same short aliases the zstd
// CLI uses). The plain set lives under prefix "zstd_"; the per-plane float sets
// live under "zstd_float0_", "zstd_float1_", etc.
inline io::ZstdCompressionParameters readZstdCompressionParameters(
    const GridBase& grid, const std::string& prefix)
{
    io::ZstdCompressionParameters parameters;
    parameters.windowLog    = readZstdParameter(grid, prefix + "wlog");
    parameters.hashLog      = readZstdParameter(grid, prefix + "hlog");
    parameters.chainLog     = readZstdParameter(grid, prefix + "clog");
    parameters.searchLog    = readZstdParameter(grid, prefix + "slog");
    parameters.minMatch     = readZstdParameter(grid, prefix + "mml");
    parameters.targetLength = readZstdParameter(grid, prefix + "tlen");
    parameters.strategy     = readZstdParameter(grid, prefix + "strat");
    return parameters;
}

} // namespace internal

/// @brief Per-grid-type codec-specific read options for CompactScalarCodec.
///
/// Store an instance in @c io::ReadOptions::typeData keyed by the codec name
/// (@c CompactScalarCodec::name(), e.g. @c "compact_floatGrid"); the codec
/// retrieves it in readBuffers() via @c io::ReadTypedOptions::cast.
struct OPENVDB_API CompactScalarCodecReadOptions : public io::ReadTypedOptions
{
    /// @brief Number of leading (most-significant, MSB-first) byte planes to
    /// read back for a 4-byte float value type; the remaining mantissa planes
    /// are skipped on the stream and zeroed in memory, yielding a lossy but
    /// cheaper read. Plane 0 is the sign/exponent byte. Valid values are 1..4;
    /// the default (4) reads every byte plane (lossless). Ignored for value
    /// types that are not 4-byte floats.
    int readFloatBytes = 4;
}; // struct CompactScalarCodecReadOptions

/// @brief Per-leaf inactive-value state, packed one byte per leaf. A leaf with
/// no inactive values has a state of 0; otherwise the set flags mark which
/// populations of inactive values the leaf holds. The negative-background and
/// non-background flags each gate a packed per-leaf mask (and, for
/// non-background, the values themselves).
enum InactiveLeafState
{
    Background         = 0x1, ///< inactive background values present
    NegativeBackground = 0x2, ///< inactive -background values present
    NonBackground      = 0x4  ///< inactive non-background values present
};

/// @brief A DynamicNodeManager operator that records, per node, the number of
/// tiles contributing to a tile value buffer. The @c inactiveTiles flag selects
/// which tile population is counted: @c false counts active tiles (value mask on
/// bits), @c true counts inactive tiles (non-child slots whose value mask bit is
/// off).
template <typename TreeType>
struct TileCountPerLevelOp
{
    using RootT = typename TreeType::RootNodeType;
    using LeafT = typename TreeType::LeafNodeType;

    TileCountPerLevelOp(std::vector<std::vector<uint32_t>>& tileCounts, bool inactiveTiles)
        : mTileCounts(tileCounts), mInactiveTiles(inactiveTiles) { }

    bool operator()(const RootT& root, size_t) const { return true; }

    template<typename NodeT>
    bool operator()(const NodeT& node, size_t idx) const
    {
        // local tile count only. NodeT::onTileCount() recurses into descendants,
        // which would over-count: the buffer this feeds is sized to the sum of
        // per-node local counts, and each node reads/writes only its own tiles.
        //
        // active: the value mask on bits, visited via cbeginValueOn.
        // inactive: the non-child slots whose value mask bit is off -
        // childMask.countOff() counts all tile slots, of which valueMask.countOn()
        // are active. matches TileCountOp's inactive accounting.
        mTileCounts[NodeT::LEVEL][idx] = mInactiveTiles
            ? node.getChildMask().countOff() - node.getValueMask().countOn()
            : node.getValueMask().countOn();
        return true;
    }

    bool operator()(const LeafT&, size_t) const { return false; }

    std::vector<std::vector<uint32_t>>& mTileCounts;
    bool mInactiveTiles;
}; // struct TileCountPerLevelOp

/// @brief Computes, per node, the start offset into a tile value buffer.
///
/// Both the active and inactive tile value buffers are laid out by the same
/// recipe: each node owns a contiguous region whose start is the exclusive
/// prefix sum of per-node tile counts over the foreachTopDown traversal order
/// (highest internal level first, descending, each node in idx order). This
/// structure runs the count pass and prefix sum, exposing the resulting start
/// offsets for both WriteTileValuesOp and ReadTileValuesOp to consume - because
/// they share these offsets, both the write and the read can be threaded, each
/// node touching only its own region.
///
/// The @c inactiveTiles flag is forwarded to TileCountPerLevelOp and selects
/// whether the active or inactive tile population is measured.
template <typename TreeType>
struct TileValueOffsets
{
    // NodeManagerT is templated so this works with both DynamicNodeManager<TreeT>
    // (read side) and DynamicNodeManager<const TreeT> (write side); TileCountPerLevelOp
    // visits nodes by const reference either way.
    template <typename NodeManagerT, typename CountT>
    TileValueOffsets(NodeManagerT& nodeManager,
                     const std::vector<CountT>& levelNodeCounts,
                     bool inactiveTiles)
        : mStartOffsets(levelNodeCounts.size())
    {
        std::vector<std::vector<uint32_t>> tileCounts(levelNodeCounts.size());
        for (size_t i = 0; i < levelNodeCounts.size(); ++i) {
            tileCounts[i].resize(levelNodeCounts[i], 0);
            mStartOffsets[i].resize(levelNodeCounts[i], 0);
        }

        TileCountPerLevelOp<TreeType> tileCountPerLevelOp(tileCounts, inactiveTiles);
        nodeManager.foreachTopDown(tileCountPerLevelOp, /*threaded=*/true);

        // exclusive prefix sum over the top-down traversal order: descend from
        // the highest internal level to the lowest, each node in idx order. the
        // root contributes nothing to these buffers (its tile values are
        // persisted separately via the root tile buffer), so level 0 is skipped
        // and the running offset starts at 0.
        openvdb::Index64 runningOffset = 0;
        for (int level = int(tileCounts.size()) - 1; level >= 1; --level) {
            for (size_t idx = 0; idx < tileCounts[level].size(); ++idx) {
                mStartOffsets[level][idx] = runningOffset;
                runningOffset += tileCounts[level][idx];
            }
        }
        // the final running offset is the sum of all internal-node tile counts,
        // i.e. the exact element count of each byte plane (the root's tiles are
        // excluded, as above). the write side uses this to size its buffers.
        mTotalCount = runningOffset;
    }

    const std::vector<std::vector<openvdb::Index64>>& startOffsets() const { return mStartOffsets; }

    /// @brief The total number of internal-node tiles across all levels, which
    /// is the element count of each tile-value byte plane.
    openvdb::Index64 totalCount() const { return mTotalCount; }

private:
    std::vector<std::vector<openvdb::Index64>> mStartOffsets;
    openvdb::Index64 mTotalCount = 0;
}; // struct TileValueOffsets

/// @brief A DynamicNodeManager operator to count active and inactive tiles in a tree
template<typename TreeType>
struct TileCountOp
{
    using RootT = typename TreeType::RootNodeType;
    using LeafT = typename TreeType::LeafNodeType;

    TileCountOp() = default;
    TileCountOp(const TileCountOp&, tbb::split) { }

    bool operator()(const RootT& root, size_t)
    {
        for (auto iter = root.cbeginValueAll(); iter; ++iter) {
            if (iter.isValueOn()) activeCount++;
            else inactiveCount++;
        }
        return true;
    }

    template<typename NodeT>
    bool operator()(const NodeT& node, size_t)
    {
        activeCount += node.getValueMask().countOn();
        inactiveCount += node.getChildMask().countOff() - node.getValueMask().countOn();
        return true;
    }

    bool operator()(const LeafT&, size_t)
    {
        return false;
    }

    void join(const TileCountOp& other)
    {
        activeCount += other.activeCount;
        inactiveCount += other.inactiveCount;
    }

    openvdb::Index64 activeCount{0};
    openvdb::Index64 inactiveCount{0};
}; // struct TileCountOp

/// @brief A DynamicNodeManager operator that gathers tile values out of the tree
/// and writes them into a tile value buffer. The @c inactiveTiles flag selects
/// which tile population is written and must match the flag used to build the
/// start offsets (see TileValueOffsets):
///
/// active (@c false): walk the value mask's on bits via cbeginValueOn.
///
/// inactive (@c true): walk the non-child slots (value-all) and keep the
/// value-off ones. note this is NOT cbeginValueOff(), which would also visit
/// child slots.
///
/// Each node writes into a distinct, precomputed region of every byte plane
/// (the start offset is the exclusive prefix sum of per-node tile counts), so
/// this operator is safe to run threaded. It is the exact write-side mirror of
/// ReadTileValuesOp - the same traversal order and the same offsets - so values
/// round-trip byte-for-byte.
template<typename TreeType>
struct WriteTileValuesOp
{
    using RootT = typename TreeType::RootNodeType;
    using LeafT = typename TreeType::LeafNodeType;
    using ValueT = typename TreeType::ValueType;

    WriteTileValuesOp(const std::vector<io::BufferWriter>& writers,
                      const std::vector<std::vector<openvdb::Index64>>& startOffsets,
                      bool inactiveTiles)
        : mWriters(writers), mStartOffsets(startOffsets), mInactiveTiles(inactiveTiles) { }

    bool operator()(const RootT& root, size_t) const { return true; }

    template<typename NodeT>
    bool operator()(const NodeT& node, size_t idx) const
    {
        openvdb::Index64 offset = mStartOffsets[NodeT::LEVEL][idx];
        if (mInactiveTiles) {
            for (auto iter = node.cbeginValueAll(); iter; ++iter) {
                if (iter.isValueOn()) continue;
                internal::writeValuePlanes(mWriters, offset++, iter.getValue());
            }
        } else {
            for (auto iter = node.cbeginValueOn(); iter; ++iter) {
                internal::writeValuePlanes(mWriters, offset++, iter.getValue());
            }
        }
        return true;
    }

    bool operator()(const LeafT&, size_t) const { return false; }

    const std::vector<io::BufferWriter>& mWriters;
    const std::vector<std::vector<openvdb::Index64>>& mStartOffsets;
    bool mInactiveTiles;
}; // struct WriteTileValuesOp

/// @brief A DynamicNodeManager operator that reads tile values out of a tile
/// value buffer and scatters them back into the tree. The @c inactiveTiles flag
/// selects which tile population is read and must match the flag used to build
/// the start offsets (see TileValueOffsets):
///
/// active (@c false): walk the value mask's on bits via beginValueOn, mirroring
/// WriteTileValuesOp so values line up byte-for-byte.
///
/// inactive (@c true): walk the non-child slots (value-all) and keep the
/// value-off ones, mirroring WriteTileValuesOp. note this is NOT
/// beginValueOff(), which would also visit child slots.
template<typename TreeType>
struct ReadTileValuesOp
{
    using RootT = typename TreeType::RootNodeType;
    using LeafT = typename TreeType::LeafNodeType;
    using ValueT = typename TreeType::ValueType;

    ReadTileValuesOp(const std::vector<io::BufferReader>& readers,
                     const std::vector<std::vector<openvdb::Index64>>& startOffsets,
                     bool inactiveTiles, size_t keptPlanes = sizeof(ValueT))
        : mReaders(readers), mStartOffsets(startOffsets)
        , mInactiveTiles(inactiveTiles), mKeptPlanes(keptPlanes) { }

    bool operator()(RootT& root, size_t) const { return true; }

    template<typename NodeT>
    bool operator()(NodeT& node, size_t idx) const
    {
        // each node reads from a distinct, precomputed region of every byte
        // plane, so this operator is safe to run threaded.
        openvdb::Index64 offset = mStartOffsets[NodeT::LEVEL][idx];
        if (mInactiveTiles) {
            for (auto iter = node.beginValueAll(); iter; ++iter) {
                if (iter.isValueOn()) continue;
                iter.setValue(internal::readValuePlanes<ValueT>(mReaders, offset++, mKeptPlanes));
            }
        } else {
            for (auto iter = node.beginValueOn(); iter; ++iter) {
                iter.setValue(internal::readValuePlanes<ValueT>(mReaders, offset++, mKeptPlanes));
            }
        }
        return true;
    }

    bool operator()(LeafT&, size_t) const { return false; }

    const std::vector<io::BufferReader>& mReaders;
    const std::vector<std::vector<openvdb::Index64>>& mStartOffsets;
    bool mInactiveTiles;
    size_t mKeptPlanes;
}; // struct ReadTileValuesOp

template<typename TreeType>
struct ComputeLeafVoxelOffsetsOp
{
    using LeafT = typename TreeType::LeafNodeType;

    ComputeLeafVoxelOffsetsOp(std::unique_ptr<openvdb::Index64[]>& activeOffsets,
                              std::unique_ptr<openvdb::Index64[]>& inactiveOffsets)
        : mActiveOffsets(activeOffsets), mInactiveOffsets(inactiveOffsets) { }

    template<typename NodeT>
    bool operator()(NodeT&, size_t) const { return true; }

    bool operator()(LeafT& leaf, size_t idx) const
    {
        mActiveOffsets[idx + 1]   = leaf.onVoxelCount();
        mInactiveOffsets[idx + 1] = leaf.offVoxelCount();
        return false;
    }

    std::unique_ptr<openvdb::Index64[]>& mActiveOffsets;
    std::unique_ptr<openvdb::Index64[]>& mInactiveOffsets;
}; // struct ComputeLeafVoxelOffsetsOp

// BYTES

template <typename TreeT>
struct ExtractBytesOp
{
    using ValueT = typename TreeT::ValueType;

    ExtractBytesOp(tree::LeafManager<TreeT>& leafManager)
        : mLeafManager(leafManager) { }

    void run()
    {
        const size_t dstByte = internal::planeToByteIndex(mPlane, sizeof(ValueT));

        // Truncated (zeroed) plane: this plane's on-disk block was skipped by the
        // driver rather than read, so there is no buffer to decompress. Just
        // clear the destination byte of every active voxel.
        if (mZero) {
            for (Index64 idx = mStart; idx < mEnd; ++idx) {
                auto& leaf = mLeafManager.leaf(idx);
                ValueT* data = leaf.buffer().data();
                for (auto iter = leaf.beginValueOn(); iter; ++iter) {
                    internal::valueByte(data[iter.pos()], dstByte) = uint8_t(0);
                }
            }
            return;
        }

        Index64 offset = 0;

        // decompress in-place but keep the compressed allocation so it can be
        // recycled back to the shared pool instead of being freed (and later
        // re-faulted by the next read).
        mBuffer.decompress(/*keepCompressed=*/true);
        io::BufferReader bufferReader = mBuffer.uncompressedReader();
        for (Index64 idx = mStart; idx < mEnd; ++idx) {
            auto& leaf = mLeafManager.leaf(idx);
            ValueT* data = leaf.buffer().data();
            for (auto iter = leaf.beginValueOn(); iter; ++iter) {
                internal::valueByte(data[iter.pos()], dstByte) = bufferReader.readAt<uint8_t>(offset++);
            }
        }
        if (mCompression && mBufferPool != nullptr) {
            mBufferPool->push(mBuffer.releaseCompressed());
        }
        mBuffer.clear();
    }

    Index64 mStart = 0;
    Index64 mEnd = 0;
    Index64 mPlane = 0;
    io::IOBuffers mBuffer;
    bool mCompression = false;
    bool mZero = false;
    io::DefaultInitVectorPool* mBufferPool = nullptr;
    tree::LeafManager<TreeT> mLeafManager;
}; // struct ExtractBytesOp

template <typename TreeT>
struct ApplyRootTilePlaneOp
{
    using ValueT = typename TreeT::ValueType;
    using RootT = typename TreeT::RootNodeType;

    explicit ApplyRootTilePlaneOp(RootT& root)
        : mRoot(root) { }

    // Scatter one byte plane of the root tile buffer back into memory. Index 0
    // of the plane holds the background byte; the remaining entries hold the
    // tile bytes in beginValueAll() order (matching the write side). Each plane
    // op writes a distinct byte of every value, so the ops are safe to run
    // concurrently without locking. The background byte is written straight into
    // the root's own background storage (root.background() returns a reference to
    // it), so no separate copy-back via setBackground() is needed. The
    // const_casts are needed to write a single byte in place (iterator
    // setValue() would overwrite the whole value, and a read-modify-write of the
    // whole value would race the other plane ops); they mirror ExtractBytesOp's
    // direct byte access into leaf storage.
    void run()
    {
        const size_t dstByte = internal::planeToByteIndex(mPlane, sizeof(ValueT));

        // Truncated (zeroed) plane: this plane's on-disk block was skipped by the
        // caller rather than read, so there is no buffer to decompress. Just
        // clear this byte of the background and of every tile value.
        if (mZero) {
            ValueT& background = const_cast<ValueT&>(mRoot.background());
            internal::valueByte(background, dstByte) = uint8_t(0);
            if (mApplyTiles) {
                for (auto iter = mRoot.beginValueAll(); iter; ++iter) {
                    ValueT& value = const_cast<ValueT&>(iter.getValue());
                    internal::valueByte(value, dstByte) = uint8_t(0);
                }
            }
            return;
        }

        mBuffer.decompress();
        io::BufferReader reader = mBuffer.uncompressedReader();

        ValueT& background = const_cast<ValueT&>(mRoot.background());
        internal::valueByte(background, dstByte) = reader.readAt<uint8_t>(0);

        if (mApplyTiles) {
            size_t offset = 1;
            for (auto iter = mRoot.beginValueAll(); iter; ++iter) {
                ValueT& value = const_cast<ValueT&>(iter.getValue());
                internal::valueByte(value, dstByte) = reader.readAt<uint8_t>(offset++);
            }
        }
        mBuffer.clear();
    }

    RootT& mRoot;
    size_t mPlane = 0;
    bool mApplyTiles = true;
    bool mZero = false;
    io::IOBuffers mBuffer;
    bool mCompression = false;
}; // struct ApplyRootTilePlaneOp

template <typename GridT>
struct CompactScalarCodec final: public CompactTopologyCodec<GridT>
{
    using Ptr = std::unique_ptr<CompactScalarCodec<GridT>>;
    using TreeT = typename GridT::TreeType;
    using ValueT = typename TreeT::ValueType;
    using NodeMaskType = typename TreeT::LeafNodeType::NodeMaskType;

    ~CompactScalarCodec() noexcept = default;

    static inline std::string name()
    {
        return "compact_" + GridT::gridType();
    }

    void readBuffers(std::istream& is, int64_t size, io::CodecData& data, const io::ReadOptions& options, io::ReadDiagnostics&) const final
    {
        CompactCodecData& codecData = static_cast<CompactCodecData&>(data);

        const bool compression = codecData.compression;

        const bool seekable = io::isStreamSeekable(is);

        io::IOOffsetTable offsetTable;
        if (seekable) {
            const int64_t startOffset = is.tellg();
            is.seekg(startOffset + size - sizeof(int64_t));
            offsetTable.read(is, compression);
            is.seekg(startOffset);
        }

        codecData.taskGroup.wait();

        GridT& grid = static_cast<GridT&>(*codecData.grid);

        // Optional lossy read: keep only the leading (most-significant, MSB-first)
        // byte planes of each float value and zero the rest, skipping the dropped
        // planes' on-disk blocks entirely. Only applies to 4-byte float value
        // types; CompactScalarCodecReadOptions::readFloatBytes selects how many
        // planes to keep (1..4; 4 = lossless, the default). For every other case
        // keptPlanes stays at sizeof(ValueT), so the read path is unchanged.
        size_t keptPlanes = sizeof(ValueT);
        if (std::is_floating_point<ValueT>::value && sizeof(ValueT) == 4) {
            auto it = options.typeData.find(name());
            if (it != options.typeData.end()) {
                const auto& readOptions =
                    io::ReadTypedOptions::cast<CompactScalarCodecReadOptions>(it->second);
                if (readOptions.readFloatBytes >= 1 &&
                    readOptions.readFloatBytes < int(sizeof(ValueT))) {
                    keptPlanes = size_t(readOptions.readFloatBytes);
                }
            }
        }

        tree::DynamicNodeManager<TreeT> nodeManager(grid.tree());

        // counts header

        Index64 numActiveTiles = 0, numInactiveTiles = 0, activeLeafVoxels = 0, inactiveLeafVoxels = 0;
        is.read(reinterpret_cast<char*>(&numActiveTiles),     sizeof(Index64));
        is.read(reinterpret_cast<char*>(&numInactiveTiles),   sizeof(Index64));
        is.read(reinterpret_cast<char*>(&activeLeafVoxels),   sizeof(Index64));
        is.read(reinterpret_cast<char*>(&inactiveLeafVoxels), sizeof(Index64));

        auto offsetIter = offsetTable.iterator();

        const bool topologyOnly = (options.readMode == io::ReadMode::TopologyOnly);

        // root tile values (byte planes; index 0 of each plane is background)

        {
            // Each plane op writes a single, distinct byte of the background and
            // of every tile value directly into the root's own storage, so across
            // all sizeof(ValueT) planes every byte is written exactly once - no
            // pre-initialization and no copy-back of the background are required.
            // When reading topology only, the tile values are left for
            // setVoxelsToBackground() below and only the background is restored.
            auto& root = grid.tree().root();

            std::vector<ApplyRootTilePlaneOp<TreeT>> rootPlaneOps;
            rootPlaneOps.reserve(sizeof(ValueT));
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                rootPlaneOps.emplace_back(root);
                rootPlaneOps.back().mPlane = plane;
                rootPlaneOps.back().mCompression = compression;
                rootPlaneOps.back().mApplyTiles = !topologyOnly;
                // Truncated mantissa plane: zero this byte rather than reading it.
                rootPlaneOps.back().mZero = (plane >= keptPlanes);
            }

            tbb::task_group rootPlaneTaskGroup;
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                // Dropped planes are skipped on the stream (not faulted into a
                // buffer); kept planes are read as usual.
                if (plane >= keptPlanes) {
                    io::IOBuffers::skip(is, offsetIter.next(), seekable);
                } else {
                    rootPlaneOps[plane].mBuffer.read(is, offsetIter.next(), compression);
                }
                rootPlaneTaskGroup.run([&rootPlaneOps, plane]() {
                    rootPlaneOps[plane].run();
                });
            }
            rootPlaneTaskGroup.wait();
        }

        if (topologyOnly) {
            internal::setVoxelsToBackground(grid.tree());
            return;
        }

        const ValueT& background = grid.tree().background();

        if (numActiveTiles > 0) {
            // active tile values (byte planes: one buffer per byte of ValueT)

            // Only the kept leading planes are read; dropped (truncated mantissa)
            // planes are skipped on the stream and never decompressed.
            // readValuePlanes() reads readers[plane] only for plane < keptPlanes
            // and zeroes the rest, so readers holds just the kept entries.
            std::vector<io::IOBuffers> activeTileBuffers(sizeof(ValueT));
            std::vector<io::BufferReader> readers;
            readers.reserve(keptPlanes);
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                if (plane >= keptPlanes) {
                    io::IOBuffers::skip(is, offsetIter.next(), seekable);
                    continue;
                }
                activeTileBuffers[plane].read(is, offsetIter.next(), compression);
                activeTileBuffers[plane].decompress();
                readers.push_back(activeTileBuffers[plane].uncompressedReader());
            }

            // build per-node start offsets, identical prefix sum to the write
            // side, so each node reads its own region and the apply can be
            // threaded. the topology codec has already restored child and value
            // masks by this point, so the counts match what was written.
            TileValueOffsets<TreeT> activeTileOffsets(
                nodeManager, codecData.nodeCounts.counts, /*inactiveTiles=*/false);

            nodeManager.foreachTopDown(
                ReadTileValuesOp<TreeT>(readers, activeTileOffsets.startOffsets(), /*inactiveTiles=*/false, keptPlanes),
                /*threaded=*/true);
        }

        // per-leaf base offsets into the active / inactive voxel plane buffers

        const size_t numLeaves = codecData.nodeCounts.numLeaves();

        std::unique_ptr<Index64[]> activeLeafOffsets(new Index64[numLeaves + 1]);
        std::unique_ptr<Index64[]> inactiveLeafOffsets(new Index64[numLeaves + 1]);
        activeLeafOffsets[0]   = 0;
        inactiveLeafOffsets[0] = 0;

        ComputeLeafVoxelOffsetsOp<TreeT> computeOffsetsOp(activeLeafOffsets, inactiveLeafOffsets);
        nodeManager.foreachTopDown(computeOffsetsOp, /*threaded=*/true);

        for (size_t i = 1; i <= numLeaves; ++i) {
            activeLeafOffsets[i]   += activeLeafOffsets[i - 1];
            inactiveLeafOffsets[i] += inactiveLeafOffsets[i - 1];
        }

        OPENVDB_ASSERT(activeLeafOffsets[numLeaves]   == activeLeafVoxels);
        OPENVDB_ASSERT(inactiveLeafOffsets[numLeaves] == inactiveLeafVoxels);

        io::IOBuffers leafOffsetsBuffer;
        leafOffsetsBuffer.read(is, offsetIter.next(), compression);
        leafOffsetsBuffer.decompress();
        const Index64 leafOffsetsSize =
            leafOffsetsBuffer.uncompressedBuffer.size() / sizeof(Index64);
        std::vector<Index64> leafOffsets(leafOffsetsSize);
        if (leafOffsetsSize > 0) {
            leafOffsetsBuffer.uncompressedReader().readIntoVector(leafOffsets, 0);
        }

        tree::LeafManager<TreeT> leafManager(grid.tree());

        // active leaf voxels

        // READ

        tbb::task_group byteExtractionTaskGroup;

        std::vector<ExtractBytesOp<TreeT>> extractBytesOps;
        extractBytesOps.reserve(leafOffsets.size() * sizeof(ValueT));

        for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
            for (size_t i = 0; i < leafOffsets.size(); ++i) {
                Index64 start = i == 0 ? 0 : leafOffsets[i - 1];
                Index64 end = leafOffsets[i];
                extractBytesOps.emplace_back(leafManager);
                auto& op = extractBytesOps.back();
                op.mStart = start;
                op.mEnd = end;
                op.mPlane = plane;
                op.mCompression = compression;
                // Truncated mantissa plane: zero this byte of every active voxel
                // instead of reading the (skipped) on-disk block.
                op.mZero = (plane >= keptPlanes);
            }
        }

        // Decompress+scatter runs in a dedicated, deliberately under-subscribed
        // arena rather than the global one. The read itself is disk-bound (far
        // below DRAM bandwidth), so the slowdown that used to drag this loop ~20%
        // above the dd/ddtest floor was CPU-scheduling contention: dispatching
        // one task per block to ~all cores oversubscribed the box and preempted
        // the runnable read thread (proven by ablation - capping *neutered*
        // workers, which use ~zero bandwidth, recovered the full gap; see
        // openvdb/perf/active-leaf-voxel-read-scheduling-contention.md). Capping
        // the arena at roughly half the hardware threads leaves cores for the
        // read thread while keeping enough decompress parallelism that the work
        // stays fully hidden behind the I/O (the drain tail after the last read
        // is ~0). A sweep over {4,8,12,16,20,24,32} bottomed out at half-cores.
        const unsigned hardwareThreads = std::max(1u, std::thread::hardware_concurrency());
        int decompressConcurrency = std::max(1, int(hardwareThreads / 2));
        // An explicit request (>0) overrides the half-cores default. Clamp it to
        // [1, hardwareThreads]; warn when a request above the logical core count
        // is capped, since over-subscribing the box is what slows this loop.
        if (options.decompressThreads > 0) {
            decompressConcurrency = options.decompressThreads;
            if (decompressConcurrency > int(hardwareThreads)) {
                OPENVDB_LOG_WARN("requested " << decompressConcurrency
                    << " decompress threads exceeds the " << hardwareThreads
                    << " logical cores available; clamping to " << hardwareThreads);
                decompressConcurrency = int(hardwareThreads);
            }
        }
        tbb::task_arena decompressArena(decompressConcurrency, /*reserved_for_masters=*/1);

        // Pre-faulted recycle pool. Reads land in resident pages (no per-read
        // first-touch faulting), and a small bound preserves K-way decompress
        // overlap while giving the read thread natural backpressure. Sized to
        // the worker count so every TBB thread can hold a buffer in flight.
        const size_t poolSize = compression
            ? std::min<size_t>(extractBytesOps.size(),
                  size_t(decompressConcurrency) + 2)
            : 0;

        const size_t bufferCapacity = 8*1024*1024;

        io::DefaultInitVectorPool bufferPool;
        bufferPool.set_capacity(poolSize > 0 ? poolSize : 1);
        for (size_t i = 0; i < poolSize; ++i) {
            io::DefaultInitVector buffer;
            buffer.resize(bufferCapacity);
            // touch every page once so the read below never faults
            std::memset(buffer.data(), 0, bufferCapacity);
            bufferPool.push(std::move(buffer));
        }

        // The read stays on this (master) thread; only the per-block
        // decompress+scatter task runs inside the capped arena.
        decompressArena.execute([&] {
            for (size_t i = 0; i < extractBytesOps.size(); ++i) {
                // Zeroed (truncated) plane: skip its on-disk block without pulling
                // a pooled buffer or reading the payload, then let run() clear the
                // bytes. The skip keeps the stream position in sync with the write
                // order and avoids charging a throttled stream for dropped bytes.
                if (extractBytesOps[i].mZero) {
                    io::IOBuffers::skip(is, offsetIter.next(), seekable);
                    byteExtractionTaskGroup.run([&extractBytesOps, i]() {
                        extractBytesOps[i].run();
                    });
                    continue;
                }
                if (compression) {
                    // blocks only when all pooled buffers are in flight
                    io::DefaultInitVector buffer;
                    bufferPool.pop(buffer);
                    extractBytesOps[i].mBuffer.setCompressed(std::move(buffer));
                    extractBytesOps[i].mBufferPool = &bufferPool;
                }
                extractBytesOps[i].mBuffer.read(is, offsetIter.next(), compression);
                byteExtractionTaskGroup.run([&extractBytesOps, i]() {
                    extractBytesOps[i].run();
                });
            }
        });

        // inactive tile values (byte planes: one buffer per byte of ValueT)

        std::vector<io::IOBuffers> inactiveTileBuffers(sizeof(ValueT));
        std::vector<io::BufferReader> inactiveTileReaders;
        if (numInactiveTiles > 0) {
            // As with the active tiles, only the kept leading planes are read;
            // dropped planes are skipped and zeroed by readValuePlanes().
            inactiveTileReaders.reserve(keptPlanes);
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                if (plane >= keptPlanes) {
                    io::IOBuffers::skip(is, offsetIter.next(), seekable);
                    continue;
                }
                inactiveTileBuffers[plane].read(is, offsetIter.next(), compression);
                inactiveTileBuffers[plane].decompress();
                inactiveTileReaders.push_back(inactiveTileBuffers[plane].uncompressedReader());
            }
        }

        if (numInactiveTiles > 0) {
            // build per-node start offsets, identical prefix sum to the write side
            TileValueOffsets<TreeT> inactiveTileOffsets(
                nodeManager, codecData.nodeCounts.counts, /*inactiveTiles=*/true);

            nodeManager.foreachTopDown(
                ReadTileValuesOp<TreeT>(inactiveTileReaders, inactiveTileOffsets.startOffsets(), /*inactiveTiles=*/true, keptPlanes),
                /*threaded=*/true);
        }

        // inactive value masks
        uint8_t hasNonBackgroundInactiveValues = 0;
        is.read(reinterpret_cast<char*>(&hasNonBackgroundInactiveValues), sizeof(uint8_t));
        if (hasNonBackgroundInactiveValues == 1) {

            Index64 inactiveNonBackgroundValuesSize;
            is.read(reinterpret_cast<char*>(&inactiveNonBackgroundValuesSize), sizeof(Index64));

            io::IOBuffers bufferState;
            bufferState.read(is, offsetIter.next(), compression);
            if (compression)     bufferState.decompress();
            io::BufferReader bufferStateReader = bufferState.uncompressedReader();

            // The values buffer is only written when there is at least one
            // inactive non-background value (see writeBuffers, which gates the
            // buffer.write() on inactiveNonBackgroundValues.size() > 0). Mirror
            // that gate here: reading it unconditionally would consume a block
            // that was never written, desynchronizing the stream.
            std::vector<ValueT> inactiveNonBackgroundValues(inactiveNonBackgroundValuesSize);
            if (inactiveNonBackgroundValuesSize > 0) {
                // byte planes: one buffer per byte of ValueT, each holding one
                // byte per value, read back in the same plane order written
                // Only the kept leading planes are read; dropped planes are
                // skipped and zeroed by readValuePlanes().
                std::vector<io::IOBuffers> bufferValues(sizeof(ValueT));
                std::vector<io::BufferReader> bufferValuesReaders;
                bufferValuesReaders.reserve(keptPlanes);
                for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                    if (plane >= keptPlanes) {
                        io::IOBuffers::skip(is, offsetIter.next(), seekable);
                        continue;
                    }
                    bufferValues[plane].read(is, offsetIter.next(), compression);
                    if (compression)     bufferValues[plane].decompress();
                    bufferValuesReaders.push_back(bufferValues[plane].uncompressedReader());
                }

                tbb::parallel_for(tbb::blocked_range<size_t>(0, inactiveNonBackgroundValuesSize, /*grainSize=*/size_t(1024)),
                    [&](const tbb::blocked_range<size_t>& range) {
                        for (size_t i = range.begin(); i < range.end(); ++i) {
                            inactiveNonBackgroundValues[i] = internal::readValuePlanes<ValueT>(bufferValuesReaders, i, keptPlanes);
                        }
                    }
                );
            }

            // read leaf states
            std::vector<uint8_t> inactiveLeafStates(numLeaves);
            tbb::parallel_for(tbb::blocked_range<size_t>(0, numLeaves, /*grainSize=*/size_t(1024)),
                [&](const tbb::blocked_range<size_t>& range) {
                    for (size_t i = range.begin(); i < range.end(); ++i) {
                        inactiveLeafStates[i] = bufferStateReader.readAt<uint8_t>(i);
                    }
                }
            );

            const uint64_t maskBytes = sizeof(NodeMaskType);

            // exclusive prefix sums giving, for each leaf, the number of preceding
            // leaves whose state has the corresponding bit set (negative-background
            // and non-background masks are written as two separate packed regions,
            // each containing only the set leaves' masks, in leaf-index order). The
            // arrays have an extra trailing element so that the final entry holds
            // the total count of set leaves in each region.
            // these are leaf COUNTS, not byte offsets: a count is bounded by
            // numLeaves so it fits in uint32_t (halving the array footprint versus
            // uint64_t byte offsets, which is what actually mattered here since the
            // scan cost is dominated by first-touch page faulting of these arrays,
            // not the arithmetic). the byte offset is recovered at each use site by
            // multiplying by maskBytes, which promotes to uint64_t so the product
            // cannot overflow.
            // serial prefix sum: each element depends on the previous, so this
            // loop-carried dependency cannot be parallelized with blocked_range.
            // bare new[] default-initializes (leaves trivial types uninitialized),
            // avoiding the serial zero-fill that make_unique<T[]> would perform;
            // every element is written exactly once below ([0] here, [1..numLeaves]
            // in the scan), so nothing is ever read uninitialized
            std::unique_ptr<uint32_t[]> negativeBackgroundLeafCount(new uint32_t[numLeaves + 1]);
            std::unique_ptr<uint32_t[]> nonBackgroundLeafCount(new uint32_t[numLeaves + 1]);
            negativeBackgroundLeafCount[0] = 0;
            nonBackgroundLeafCount[0] = 0;

            // the per-element sentinel is trivial memory traffic, so it is folded
            // directly into the serial accumulation rather than materialized by a
            // separate parallel pass that the scan would immediately consume
            for (size_t i = 1; i <= numLeaves; ++i) {
                negativeBackgroundLeafCount[i] = negativeBackgroundLeafCount[i-1] +
                    ((inactiveLeafStates[i-1] & NegativeBackground) ? 1 : 0);
                nonBackgroundLeafCount[i] = nonBackgroundLeafCount[i-1] +
                    ((inactiveLeafStates[i-1] & NonBackground) ? 1 : 0);
            }

            // region bases within bufferState: leaf states, then negative-
            // background masks, then non-background masks
            const uint64_t negativeBackgroundStartOffset = numLeaves * sizeof(uint8_t);
            const uint64_t nonBackgroundStartOffset =
                negativeBackgroundStartOffset + uint64_t(negativeBackgroundLeafCount[numLeaves]) * maskBytes;

            // the masks themselves already live, decompressed and packed in leaf
            // order, inside bufferState; copying them into a second numLeaves-sized
            // structure here would value-initialize every leaf's mask up front
            // (serial zero-fill + first-touch faulting over numLeaves x maskBytes),
            // which dominates when only a fraction of leaves are set. so we store
            // nothing and instead read each mask on demand in the Apply loop below,
            // straight from the warm buffer into an L1-resident stack local.
            // the one thing the serial Prefix Sum 2 scan needs ahead of time is the
            // per-leaf non-background popcount, so compute just that here in
            // parallel (reading the mask into a local, counting, discarding it).
            std::vector<uint32_t> nonBackgroundCount(numLeaves, 0);
            tbb::parallel_for(tbb::blocked_range<size_t>(0, numLeaves, /*grainSize=*/size_t(16)),
                [&](const tbb::blocked_range<size_t>& range) {
                    for (size_t i = range.begin(); i < range.end(); ++i) {
                        if (inactiveLeafStates[i] & NonBackground) {
                            const uint64_t offset = nonBackgroundStartOffset + uint64_t(nonBackgroundLeafCount[i]) * maskBytes;
                            nonBackgroundCount[i] =
                                bufferStateReader.readAt<NodeMaskType>(offset).countOn();
                        }
                    }
                }
            );

            // exclusive prefix sum giving each leaf's start offset into the
            // packed inactiveNonBackgroundValues array (each leaf consumes one
            // value per set bit in its non-background mask, in leaf-index
            // order). The array has an extra trailing element so that the final
            // entry holds the total number of non-background values.
            // serial prefix sum: each element depends on the previous, so this
            // loop-carried dependency cannot be parallelized with blocked_range
            // counts were precomputed in Prep Values, so this scan is now a plain
            // contiguous accumulation with no pointer-chase or popcount inside the
            // loop-carried dependency.
            // uninitialized bare new[] (not vector(…, 0)) to skip the serial
            // zero-fill the scan would immediately overwrite; element 0 is the
            // identity and [1..numLeaves] are each written once below
            std::unique_ptr<uint64_t[]> nonBackgroundValueOffset(new uint64_t[numLeaves + 1]);
            nonBackgroundValueOffset[0] = 0;
            for (size_t i = 1; i <= numLeaves; ++i) {
                nonBackgroundValueOffset[i] = nonBackgroundValueOffset[i - 1] + nonBackgroundCount[i - 1];
            }

            tbb::parallel_for(tbb::blocked_range<size_t>(0, numLeaves, /*grainSize=*/size_t(16)),
                [&](const tbb::blocked_range<size_t>& range) {
                    for (size_t i = range.begin(); i < range.end(); ++i) {
                        if (inactiveLeafStates[i] == 0)   continue;
                        auto& leaf = leafManager.leaf(i);
                        size_t valueOffset = nonBackgroundValueOffset[i];
                        // read each leaf's masks on demand into L1-resident stack
                        // locals, straight from the warm packed buffer; only the
                        // masks this leaf's state bits flag as present are touched
                        NodeMaskType nonBackgroundMask, negativeBackgroundMask;
                        if (inactiveLeafStates[i] & NonBackground) {
                            const uint64_t offset = nonBackgroundStartOffset + uint64_t(nonBackgroundLeafCount[i]) * maskBytes;
                            nonBackgroundMask = bufferStateReader.readAt<NodeMaskType>(offset);
                        }
                        if (inactiveLeafStates[i] & NegativeBackground) {
                            const uint64_t offset = negativeBackgroundStartOffset + uint64_t(negativeBackgroundLeafCount[i]) * maskBytes;
                            negativeBackgroundMask = bufferStateReader.readAt<NodeMaskType>(offset);
                        }
                        for (auto iter = leaf.beginValueOff(); iter; ++iter) {
                            if (inactiveLeafStates[i] & NonBackground && nonBackgroundMask.isOn(iter.pos())) {
                                iter.setValue(inactiveNonBackgroundValues[valueOffset++]);
                            } else if (inactiveLeafStates[i] & NegativeBackground && negativeBackgroundMask.isOn(iter.pos())) {
                                iter.setValue(-background);
                            } else {
                                iter.setValue(background);
                            }
                        }
                    }
                }
            );
        }

        // skip over offset table
        int64_t offsetTableSize = 0;
        is.read(reinterpret_cast<char*>(&offsetTableSize), sizeof(int64_t));
        offsetTableSize += sizeof(int64_t);
        if (seekable)   is.seekg(offsetTableSize, std::ios::cur);
        else            is.ignore(offsetTableSize);

        // Drain the deferred decompress+scatter inside the arena so the cap
        // applies to the tail as well. In practice this is ~0: the work
        // completes during the read loop and is fully hidden behind the I/O.
        decompressArena.execute([&] {
            byteExtractionTaskGroup.wait();
        });

    }

    void writeBuffers(std::ostream& os, const GridBase& gridBase, const io::WriteOptions& options) const final
    {
        const bool compression = options.compression;

        io::IOOffsetTable offsetTable;

        const GridT& grid = static_cast<const GridT&>(gridBase);
        tree::DynamicNodeManager<const TreeT> nodeManager(grid.tree());

        const ValueT background = grid.tree().background();

        // Advanced zstd parameter overrides, read from transient grid metadata
        // (e.g. attached by the vdb_io command line). The plain "zstd_" set
        // drives every auxiliary (non-byte-planed) buffer and, for non-float
        // value types, every value buffer too. For float value types each byte
        // plane instead uses its own "zstd_float<plane>_" set, so the exponent
        // plane (plane 0) and the mantissa planes can be tuned independently.
        // Any parameter left unset falls back to the compressor's preset-3
        // default.
        const io::ZstdCompressionParameters plainParameters =
            internal::readZstdCompressionParameters(grid, "zstd_");
        const bool valueIsFloat = std::is_floating_point<ValueT>::value;
        std::vector<io::ZstdCompressionParameters> planeParameters(sizeof(ValueT));
        for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
            planeParameters[plane] = valueIsFloat
                ? internal::readZstdCompressionParameters(
                      grid, "zstd_float" + std::to_string(plane) + "_")
                : plainParameters;
        }

        // Root:
        // - # active tiles
        // - # inactive tiles (non-background)

        // Internal1:
        // - # active tiles
        // - # inactive tile mask (non-background) - uint8_t(0) / uint8_t(1)
        // - # inactive tiles (non-background)

        // Internal2:
        // - # active tiles
        // - # inactive tile mask (non-background) - uint8_t(0) / uint8_t(1)
        // - # inactive tiles (non-background)

        // Leaf:
        // - # inactive voxels (non-background)
        // - # inactive voxel mask (non-background) - uint8_t(0) / uint8_t(1)

        std::vector<Index64> nodeCounts = grid.tree().nodeCount();

        // counts header
        TileCountOp<TreeT> tileCountOp;
        nodeManager.reduceTopDown(tileCountOp, /*threaded=*/true);
        const Index64 numActiveTiles = tileCountOp.activeCount;
        const Index64 numInactiveTiles = tileCountOp.inactiveCount;
        const Index64 activeLeafVoxels = tools::countActiveLeafVoxels(grid.tree());
        const Index64 inactiveLeafVoxels = tools::countInactiveLeafVoxels(grid.tree());

        os.write(reinterpret_cast<const char*>(&numActiveTiles), sizeof(Index64));
        os.write(reinterpret_cast<const char*>(&numInactiveTiles), sizeof(Index64));
        os.write(reinterpret_cast<const char*>(&activeLeafVoxels), sizeof(Index64));
        os.write(reinterpret_cast<const char*>(&inactiveLeafVoxels), sizeof(Index64));

        { // root tile values (byte planes; index 0 of each plane is background)
            const auto& root = grid.tree().root();
            // one entry per tile value, plus the background at index 0
            const Index64 bufferSize = 1 + root.tileCount();

            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                io::IOBuffers buffers;
                buffers.setCompressionParameters(planeParameters[plane]);
                io::BufferWriter writer = buffers.allocate(int64_t(bufferSize));
                const size_t srcByte = internal::planeToByteIndex(plane, sizeof(ValueT));

                writer.writeAt<uint8_t>(0, internal::valueByte(background, srcByte));

                size_t offset = 1;
                for (auto iter = root.cbeginValueAll(); iter; ++iter) {
                    writer.writeAt<uint8_t>(offset++, internal::valueByte(iter.getValue(), srcByte));
                }
                offsetTable.append(buffers.write(os, compression));
            }
        }

        // active tile values (byte planes: one buffer per byte of ValueT, each
        // holding the internal-node active tile count in foreachTopDown order).
        // TileValueOffsets builds the same per-node start offsets the read side
        // uses, so the gather can be threaded - each node writes its own region -
        // and the layout round-trips byte-for-byte. the buffer is sized to
        // totalCount() (internal-node tiles only; the root's active tiles are
        // persisted separately via the root tile buffer), not numActiveTiles: an
        // over-sized buffer would leave an uninitialized tail that buffer.write()
        // would still compress, bloating the output. the gate stays
        // numActiveTiles > 0 (not totalCount() > 0) so the block count matches
        // the read side, which gates on numActiveTiles; totalCount() can be 0
        // here (all active tiles at the root) and the planes are written empty.
        if (numActiveTiles > 0) {
            TileValueOffsets<TreeT> activeTileOffsets(
                nodeManager, nodeCounts, /*inactiveTiles=*/false);
            OPENVDB_ASSERT(activeTileOffsets.totalCount() <= numActiveTiles);

            std::vector<io::IOBuffers> buffers(sizeof(ValueT));
            std::vector<io::BufferWriter> writers;
            writers.reserve(sizeof(ValueT));
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                buffers[plane].setCompressionParameters(planeParameters[plane]);
                writers.push_back(buffers[plane].allocate(int64_t(activeTileOffsets.totalCount())));
            }
            nodeManager.foreachTopDown(
                WriteTileValuesOp<TreeT>(writers, activeTileOffsets.startOffsets(), /*inactiveTiles=*/false),
                /*threaded=*/true);
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                offsetTable.append(buffers[plane].write(os, compression));
            }
        }

        // The default chunk granularity can be overridden per-grid by attaching
        // an Int64 "max_voxels_per_chunk" metadata value to the grid (e.g. from
        // the vdb_io command line). A non-positive value falls back to the
        // default.
        Index64 maxVoxelsPerChunk = 100*1000;
        if (auto override = grid.template getMetadata<Int64Metadata>("max_voxels_per_chunk")) {
            if (override->value() > 0) {
                maxVoxelsPerChunk = static_cast<Index64>(override->value());
            }
        }
        const Index64 voxelsPerChunk = std::max(activeLeafVoxels / 1024, maxVoxelsPerChunk);

        tree::LeafManager<const TreeT> leafManager(grid.tree());
        const Index64 numLeaves = leafManager.leafCount();

        std::vector<Index64> voxelsPerLeaf(numLeaves);
        for (size_t i = 0; i < numLeaves; ++i) {
            voxelsPerLeaf[i] = leafManager.leaf(i).onVoxelCount();
        }

        std::vector<Index64> leafOffsets;

        Index64 voxels = 0;
        for (size_t i = 0; i < numLeaves; ++i) {
            voxels += voxelsPerLeaf[i];
            if (voxels >= voxelsPerChunk) {
                leafOffsets.push_back(i);
                voxels = 0;
            }
        }
        if (voxels > 0) leafOffsets.push_back(numLeaves);

        std::vector<Index64> leafSizes(leafOffsets.size());
        for (size_t i = 0; i < leafOffsets.size(); ++i) {
            Index64 start = i == 0 ? 0 : leafOffsets[i - 1];
            Index64 end = leafOffsets[i];
            // compute size
            Index64 size = 0;
            for (Index64 idx = start; idx < end; ++idx) {
                auto& leaf = leafManager.leaf(idx);
                size += leaf.onVoxelCount();
            }
            leafSizes[i] = size;
        }

        // leaf offsets, written as a (optionally ZSTD-compressed) IOBuffers
        // block. The element count is recovered on read from the decompressed
        // buffer size, so no separate count prefix is stored.
        {
            io::IOBuffers buffer;
            buffer.setCompressionParameters(plainParameters);
            io::BufferWriter bufferWriter = buffer.allocate(int64_t(leafOffsets.size()) * sizeof(Index64));
            bufferWriter.writeFromVector(leafOffsets, 0);
            offsetTable.append(buffer.write(os, compression));
        }

        // per-leaf active voxel start offsets (prefix sum over voxelsPerLeaf).
        // Each leaf's voxels occupy a disjoint region of its chunk buffer, so
        // the gather can be threaded one leaf per task: leaf idx writes at
        // (leafVoxelStart[idx] - leafVoxelStart[chunkStart]) within the chunk.
        std::vector<Index64> leafVoxelStart(numLeaves + 1);
        leafVoxelStart[0] = 0;
        for (size_t i = 0; i < numLeaves; ++i) {
            leafVoxelStart[i + 1] = leafVoxelStart[i] + voxelsPerLeaf[i];
        }

        // WRITE

        // Flatten the (plane, chunk) nest into a single ordered job list. The
        // read side consumes blocks in exactly this order (plane-major, then
        // chunk), so the on-disk write order must follow the same sequence.
        struct WriteJob { Index64 start; Index64 end; Index64 size; size_t plane; };
        std::vector<WriteJob> jobs;
        jobs.reserve(sizeof(ValueT) * leafOffsets.size());
        for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
            for (size_t i = 0; i < leafOffsets.size(); ++i) {
                const Index64 start = i == 0 ? 0 : leafOffsets[i - 1];
                jobs.push_back(WriteJob{start, leafOffsets[i], leafSizes[i], plane});
            }
        }

        // Populate one chunk buffer via the threaded per-leaf gather, then
        // compress it. Each leaf writes to its own disjoint region of the
        // buffer, so the gather is threaded one leaf (range of leaves) per task.
        // The buffer is sized to one byte per active voxel for this plane (not
        // sizeof(ValueT) per voxel): over-allocating would leave an
        // uninitialized tail that the compressor would still process, bloating
        // output. Compressing here (rather than in write()) is what lets the
        // CPU-bound compression of a batch run concurrently while the disk
        // writes below stay serial.
        auto fillAndCompress = [&](io::IOBuffers& buffer, const WriteJob& job) {
            buffer.setCompressionParameters(planeParameters[job.plane]);
            const size_t srcByte = internal::planeToByteIndex(job.plane, sizeof(ValueT));
            io::BufferWriter bufferWriter = buffer.allocate(int64_t(job.size));
            const Index64 chunkVoxelStart = leafVoxelStart[job.start];
            tbb::parallel_for(tbb::blocked_range<Index64>(job.start, job.end),
                [&](const tbb::blocked_range<Index64>& range) {
                    for (Index64 idx = range.begin(); idx < range.end(); ++idx) {
                        auto& leaf = leafManager.leaf(idx);
                        Index64 offset = leafVoxelStart[idx] - chunkVoxelStart;
                        for (auto iter = leaf.cbeginValueOn(); iter; ++iter) {
                            bufferWriter.writeAt<uint8_t>(offset++, internal::valueByte(iter.getValue(), srcByte));
                        }
                    }
                });
            if (compression)   buffer.compress(/*keepUncompressed=*/false);
        };

        // Process the jobs in batches: fill and compress up to
        // compressConcurrency buffers in parallel, then serialize them to disk
        // one after the other in job order. Performance here is not as critical
        // as the read path, so this stays deliberately simple - the parallel
        // batch hides the CPU-bound compression behind itself, and the I/O is a
        // plain serial drain of the already-compressed buffers. The same
        // half-the-hardware-threads heuristic as the read path is used so the
        // box is not oversubscribed.
        const unsigned hardwareThreads = std::max(1u, std::thread::hardware_concurrency());
        const size_t compressConcurrency = std::max(1, int(hardwareThreads / 2));

        for (size_t base = 0; base < jobs.size(); base += compressConcurrency) {
            const size_t batchEnd = std::min(jobs.size(), base + compressConcurrency);
            const size_t batchSize = batchEnd - base;

            std::vector<io::IOBuffers> buffers(batchSize);
            tbb::parallel_for(tbb::blocked_range<size_t>(0, batchSize, /*grainSize=*/1),
                [&](const tbb::blocked_range<size_t>& range) {
                    for (size_t k = range.begin(); k < range.end(); ++k) {
                        fillAndCompress(buffers[k], jobs[base + k]);
                    }
                });

            for (size_t k = 0; k < batchSize; ++k) {
                offsetTable.append(buffers[k].write(os, compression));
            }
        }

        // inactive tile values (byte planes: one buffer per byte of ValueT, each
        // holding the internal-node inactive tile count in foreachTopDown order).
        // As with the active tiles, TileValueOffsets builds the per-node start
        // offsets so the gather can be threaded and round-trips byte-for-byte
        // with the read side, and totalCount() sizes each plane to exactly the
        // internal-node tiles written (the root's inactive tiles are persisted
        // separately via the root tile buffer). The gate stays numInactiveTiles
        // > 0 to match the read side's block count; totalCount() can be 0 (all
        // inactive tiles at the root) and the planes are written empty.
        if (numInactiveTiles > 0) {
            TileValueOffsets<TreeT> inactiveTileOffsets(
                nodeManager, nodeCounts, /*inactiveTiles=*/true);
            OPENVDB_ASSERT(inactiveTileOffsets.totalCount() <= numInactiveTiles);

            std::vector<io::IOBuffers> buffers(sizeof(ValueT));
            std::vector<io::BufferWriter> writers;
            writers.reserve(sizeof(ValueT));
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                buffers[plane].setCompressionParameters(planeParameters[plane]);
                writers.push_back(buffers[plane].allocate(int64_t(inactiveTileOffsets.totalCount())));
            }
            nodeManager.foreachTopDown(
                WriteTileValuesOp<TreeT>(writers, inactiveTileOffsets.startOffsets(), /*inactiveTiles=*/true),
                /*threaded=*/true);
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                offsetTable.append(buffers[plane].write(os, compression));
            }
        }

        std::vector<std::unique_ptr<NodeMaskType>> inactiveNegativeBackground(numLeaves);
        std::vector<std::unique_ptr<NodeMaskType>> inactiveNonBackground(numLeaves);

        for (size_t i = 0; i < numLeaves; ++i) {
            auto& leaf = leafManager.leaf(i);

            // no inactive values in this leaf
            if (leaf.isDense())     continue;

            uint8_t inactiveNegativeBackgroundCount = 0;
            uint8_t inactiveNonBackgroundCount = 0;

            for (auto iter = leaf.cbeginValueOff(); iter; ++iter) {
                // NOTE: test background first. When background == 0, -background is
                // -0.0f and IEEE compares -0.0f == 0.0f as true, so checking
                // -background first would misclassify ordinary background voxels.
                if (iter.getValue() == background) {
                    // ordinary inactive background value - nothing to record
                }
                else if (iter.getValue() == -background) {
                    // create leaf mask for inactive negative background
                    if (!inactiveNegativeBackground[i]) {
                        inactiveNegativeBackground[i] = std::make_unique<NodeMaskType>();
                    }
                    inactiveNegativeBackground[i]->setOn(iter.pos());
                }
                else {
                    // create leaf mask for inactive non-background values
                    if (!inactiveNonBackground[i]) {
                        inactiveNonBackground[i] = std::make_unique<NodeMaskType>();
                    }
                    inactiveNonBackground[i]->setOn(iter.pos());
                }
            }
        }

        std::vector<uint8_t> inactiveLeafStates(numLeaves, uint8_t(0));

        for (size_t i = 0; i < numLeaves; ++i) {
            auto& leaf = leafManager.leaf(i);
            // fully dense - has no inactive values
            if (leaf.isDense())     continue;

            uint8_t state = 0;
            int totalInactive = leaf.offVoxelCount();
            if (inactiveNegativeBackground[i]) {
                totalInactive -= inactiveNegativeBackground[i]->countOn();
                state |= NegativeBackground;
            }
            if (inactiveNonBackground[i]) {
                totalInactive -= inactiveNonBackground[i]->countOn();
                state |= NonBackground;
            }
            // neither -background nor non-background values -> must be background values
            if (totalInactive > 0) {
                state |= Background;
            }
            inactiveLeafStates[i] = state;
        }

        int totalInactiveNegativeBackgroundLeafNodes = 0;
        int totalInactiveNonBackgroundLeafNodes = 0;
        int totalInactiveNegativeBackgroundVoxels = 0;
        int totalInactiveNonBackgroundVoxels = 0;
        for (size_t i = 0; i < numLeaves; ++i) {
            if (inactiveNegativeBackground[i]) {
                totalInactiveNegativeBackgroundLeafNodes++;
                totalInactiveNegativeBackgroundVoxels += inactiveNegativeBackground[i]->countOn();
            }
            if (inactiveNonBackground[i]) {
                totalInactiveNonBackgroundLeafNodes++;
                totalInactiveNonBackgroundVoxels += inactiveNonBackground[i]->countOn();
            }
        }

        uint8_t hasNonBackgroundInactiveValues = 0;
        for (size_t i = 0; i < numLeaves; ++i) {
            if (inactiveLeafStates[i] != 0) {
                hasNonBackgroundInactiveValues = 1;
                break;
            }
        }

        std::vector<ValueT> inactiveNonBackgroundValues;

        os.write(reinterpret_cast<const char*>(&hasNonBackgroundInactiveValues), sizeof(uint8_t));
        if (hasNonBackgroundInactiveValues) {
            size_t count = numLeaves * sizeof(uint8_t);

            for (size_t i = 0; i < numLeaves; ++i) {
                if (inactiveLeafStates[i] & NegativeBackground) {
                    count += sizeof(NodeMaskType);
                }
                if (inactiveLeafStates[i] & NonBackground) {
                    count += sizeof(NodeMaskType);
                }
            }

            io::IOBuffers buffer;
            buffer.setCompressionParameters(plainParameters);
            io::BufferWriter bufferWriter = buffer.allocate(int64_t(count));

            size_t offset = 0;

            // write leaf states
            for (size_t i = 0; i < numLeaves; ++i) {
                bufferWriter.writeAt<uint8_t>(offset++, inactiveLeafStates[i]);
            }
            // write inactive negative background masks
            for (size_t i = 0; i < numLeaves; ++i) {
                if (inactiveLeafStates[i] & NegativeBackground) {
                    bufferWriter.writeAt<NodeMaskType>(offset, *inactiveNegativeBackground[i]);
                    offset += sizeof(NodeMaskType);
                }
            }
            // write inactive non-background masks
            for (size_t i = 0; i < numLeaves; ++i) {
                if (inactiveLeafStates[i] & NonBackground) {
                    bufferWriter.writeAt<NodeMaskType>(offset, *inactiveNonBackground[i]);
                    offset += sizeof(NodeMaskType);
                }
            }
            // write inactive values
            for (size_t i = 0; i < numLeaves; ++i) {
                if (inactiveLeafStates[i] & NonBackground) {
                    auto leaf = leafManager.leaf(i);
                    for (auto iter = inactiveNonBackground[i]->beginOn(); iter; ++iter) {
                        inactiveNonBackgroundValues.push_back(leaf.getValue(iter.pos()));
                    }
                }
            }
            size_t inactiveNonBackgroundValuesSize = inactiveNonBackgroundValues.size();
            os.write(reinterpret_cast<const char*>(&inactiveNonBackgroundValuesSize), sizeof(size_t));
            offsetTable.append(buffer.write(os, compression));
        }

        // inactive non-background voxel values (byte planes: one buffer per
        // byte of ValueT, each holding one byte per value)
        if (inactiveNonBackgroundValues.size() > 0) {
            const size_t numValues = inactiveNonBackgroundValues.size();
            std::vector<io::IOBuffers> buffers(sizeof(ValueT));
            std::vector<io::BufferWriter> writers;
            writers.reserve(sizeof(ValueT));
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                buffers[plane].setCompressionParameters(planeParameters[plane]);
                writers.push_back(buffers[plane].allocate(int64_t(numValues)));
            }
            for (size_t i = 0; i < numValues; ++i) {
                internal::writeValuePlanes(writers, i, inactiveNonBackgroundValues[i]);
            }
            for (size_t plane = 0; plane < sizeof(ValueT); ++plane) {
                offsetTable.append(buffers[plane].write(os, compression));
            }
        }

        // write out the offset table at the end of the grid

        offsetTable.write(os, compression);
    }
}; // struct CompactScalarCodec

} // namespace codecs
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_IO_CODECS_COMPACT_SCALAR_CODEC_HAS_BEEN_INCLUDED
