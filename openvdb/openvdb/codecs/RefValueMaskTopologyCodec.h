// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENVDB_VALUEMASK_TOPOLOGY_CODEC_HAS_BEEN_INCLUDED
#define OPENVDB_VALUEMASK_TOPOLOGY_CODEC_HAS_BEEN_INCLUDED

/// @brief Define to 0 to disable ZSTD compression (raw buffer with uint64 size prefix).
#ifndef OPENVDB_VALUEMASK_TOPOLOGY_CODEC_COMPRESSED
#define OPENVDB_VALUEMASK_TOPOLOGY_CODEC_COMPRESSED 1
#endif

#include <openvdb/openvdb.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/io/Codec.h>
#include <openvdb/Grid.h>
#if OPENVDB_VALUEMASK_TOPOLOGY_CODEC_COMPRESSED
#include <zstd.h>
#endif
#include <vector>
#include <streambuf>
#include <cstring>

/// ValueMaskTopologyCodec
///
/// Stores only the tree structure (child masks + value masks) — no tile values,
/// no background, no node origins except root children.  Designed for testing
/// topology-only compression ratios with ZSTD and a breadth-first layout that
/// is amenable to overlapped I/O.
///
/// Binary layout (all written into a single buffer, then ZSTD-compressed):
///
///   [PASS 1 — child masks, sufficient for full tree allocation]
///     uint32  numTiles
///     uint32  numRootChildren
///     Int32[numTiles]         tile origin X values
///     Int32[numTiles]         tile origin Y values
///     Int32[numTiles]         tile origin Z values
///     uint8[numTiles]         tile active flags
///     Int32[numRootChildren]  child origin X values
///     Int32[numRootChildren]  child origin Y values
///     Int32[numRootChildren]  child origin Z values
///     NodeMask * numInternal(N)   child masks, breadth-first top-down
///     NodeMask * numInternal(N-1) ...
///     (leaves have no child masks)
///
///   [PASS 2 — value masks, breadth-first top-down]
///     NodeMask * numInternal(N)   value masks
///     NodeMask * numInternal(N-1) value masks
///     NodeMask * numLeaves        value masks
///
/// ZSTD header: uint64 uncompressed size, uint64 compressed size (sans 4-byte
/// magic), then compressed bytes without the magic prefix.

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace codecs {
namespace internal {

// ---------------------------------------------------------------------------
// MemStreamBuf helpers — write/read NodeMask bytes directly into a buffer
// ---------------------------------------------------------------------------

/// @brief Minimal streambuf that writes into a pre-allocated memory region.
struct MemWriteBuf : std::streambuf {
    MemWriteBuf(char* p, size_t n) { setp(p, p + n); }
};

/// @brief Minimal streambuf that reads from an existing memory region.
struct MemReadBuf : std::streambuf {
    MemReadBuf(const char* p, size_t n) {
        char* nc = const_cast<char*>(p);
        setg(nc, nc, nc + n);
    }
};

// ---------------------------------------------------------------------------
// ZSTD helpers
// ---------------------------------------------------------------------------

/// Compress @p data and write (uncompressedSize, compressedSize, bytes) to @p os.
/// When OPENVDB_VALUEMASK_TOPOLOGY_CODEC_COMPRESSED == 0, writes a raw uint64
/// size prefix followed by the uncompressed bytes.
inline void compressAndWrite(std::ostream& os, const char* data, size_t size)
{
#if OPENVDB_VALUEMASK_TOPOLOGY_CODEC_COMPRESSED
    const size_t dstCapacity = ZSTD_compressBound(size);
    std::vector<char> compressed(dstCapacity);
    const size_t compressedSize = ZSTD_compress(
        compressed.data(), dstCapacity, data, size, /*level=*/1);
    if (ZSTD_isError(compressedSize)) {
        OPENVDB_THROW(IoError, "ValueMaskTopologyCodec: ZSTD compression failed: "
            << ZSTD_getErrorName(compressedSize));
    }
    // The existing codebase strips the 4-byte ZSTD magic and re-prepends on read.
    static constexpr size_t   kMagicSize = 4;
    static constexpr uint32_t kMagicNum  = 0xFD2FB528u;
    uint32_t magic;
    std::memcpy(&magic, compressed.data(), kMagicSize);
    if (magic != kMagicNum) {
        OPENVDB_THROW(IoError, "ValueMaskTopologyCodec: ZSTD frame missing magic number");
    }
    const uint64_t uncompSz = static_cast<uint64_t>(size);
    const uint64_t compSz   = static_cast<uint64_t>(compressedSize - kMagicSize);
    os.write(reinterpret_cast<const char*>(&uncompSz), sizeof(uint64_t));
    os.write(reinterpret_cast<const char*>(&compSz),   sizeof(uint64_t));
    os.write(compressed.data() + kMagicSize, static_cast<std::streamsize>(compSz));
#else
    const uint64_t sz = static_cast<uint64_t>(size);
    os.write(reinterpret_cast<const char*>(&sz), sizeof(uint64_t));
    os.write(data, static_cast<std::streamsize>(size));
#endif
}

/// Read from @p is, decompress, and return the raw buffer.
inline std::vector<char> readAndDecompress(std::istream& is)
{
#if OPENVDB_VALUEMASK_TOPOLOGY_CODEC_COMPRESSED
    uint64_t uncompSz = 0, compSz = 0;
    is.read(reinterpret_cast<char*>(&uncompSz), sizeof(uint64_t));
    is.read(reinterpret_cast<char*>(&compSz),   sizeof(uint64_t));
    static constexpr size_t   kMagicSize = 4;
    static constexpr uint32_t kMagicNum  = 0xFD2FB528u;
    std::vector<char> compressed(kMagicSize + compSz);
    std::memcpy(compressed.data(), &kMagicNum, kMagicSize);
    is.read(compressed.data() + kMagicSize, static_cast<std::streamsize>(compSz));
    std::vector<char> uncompressed(uncompSz);
    const size_t decompSz = ZSTD_decompress(
        uncompressed.data(), uncompSz,
        compressed.data(), kMagicSize + compSz);
    if (ZSTD_isError(decompSz)) {
        OPENVDB_THROW(IoError, "ValueMaskTopologyCodec: ZSTD decompression failed: "
            << ZSTD_getErrorName(decompSz));
    }
    return uncompressed;
#else
    uint64_t sz = 0;
    is.read(reinterpret_cast<char*>(&sz), sizeof(uint64_t));
    std::vector<char> buffer(sz);
    is.read(buffer.data(), static_cast<std::streamsize>(sz));
    return buffer;
#endif
}

// ---------------------------------------------------------------------------
// Mask I/O helpers
// ---------------------------------------------------------------------------

/// Write @p mask into the buffer at @p cursor, then advance @p cursor.
template <typename MaskT>
inline void writeMask(char*& cursor, const MaskT& mask)
{
    const Index maskBytes = MaskT::memUsage();
    MemWriteBuf wbuf(cursor, maskBytes);
    std::ostream os(&wbuf);
    mask.save(os);
    cursor += maskBytes;
}

/// Read a NodeMask from the buffer at @p cursor into @p mask, then advance @p cursor.
template <typename MaskT>
inline void readMask(const char*& cursor, MaskT& mask)
{
    const Index maskBytes = MaskT::memUsage();
    MemReadBuf rbuf(cursor, maskBytes);
    std::istream is(&rbuf);
    mask.load(is);
    cursor += maskBytes;
}

// ---------------------------------------------------------------------------
// Buffer-size computation
// ---------------------------------------------------------------------------

/// Returns the total mask bytes required for @p nodes (one internal level) and
/// all levels below it: (child mask + value mask) per internal node, plus
/// value mask per leaf.
template <typename InternalT>
size_t computeMaskBytes(const std::vector<const InternalT*>& nodes)
{
    using ChildT = typename InternalT::ChildNodeType;
    // Each InternalT node contributes one child mask + one value mask.
    const size_t perNode = 2 * static_cast<size_t>(InternalT::NodeMaskType::memUsage());
    size_t total = nodes.size() * perNode;

    if constexpr (ChildT::LEVEL > 0) {
        // Children are also internal — recurse.
        std::vector<const ChildT*> children;
        for (const auto* n : nodes)
            for (auto it = n->cbeginChildOn(); it; ++it)
                children.push_back(&(*it));
        total += computeMaskBytes(children);
    } else {
        // Children are leaves — only value masks needed.
        size_t numLeaves = 0;
        for (const auto* n : nodes)
            for (auto it = n->cbeginChildOn(); it; ++it)
                ++numLeaves;
        total += numLeaves * static_cast<size_t>(ChildT::NodeMaskType::memUsage());
    }
    return total;
}

// ---------------------------------------------------------------------------
// Write pass 1: child masks, breadth-first top-down
// ---------------------------------------------------------------------------

template <typename InternalT>
void writeChildMasks(char*& cursor, const std::vector<const InternalT*>& nodes)
{
    using ChildT = typename InternalT::ChildNodeType;
    for (const auto* n : nodes)
        writeMask(cursor, n->getChildMask());

    if constexpr (ChildT::LEVEL > 0) {
        std::vector<const ChildT*> children;
        for (const auto* n : nodes)
            for (auto it = n->cbeginChildOn(); it; ++it)
                children.push_back(&(*it));
        writeChildMasks(cursor, children);
    }
    // Leaf nodes have no child masks — base case, nothing to recurse.
}

// ---------------------------------------------------------------------------
// Write pass 2: value masks, breadth-first top-down
// ---------------------------------------------------------------------------

template <typename InternalT>
void writeValueMasks(char*& cursor, const std::vector<const InternalT*>& nodes)
{
    using ChildT = typename InternalT::ChildNodeType;
    for (const auto* n : nodes)
        writeMask(cursor, n->getValueMask());

    if constexpr (ChildT::LEVEL > 0) {
        std::vector<const ChildT*> children;
        for (const auto* n : nodes)
            for (auto it = n->cbeginChildOn(); it; ++it)
                children.push_back(&(*it));
        writeValueMasks(cursor, children);
    } else {
        // Children are leaves — write their value masks.
        std::vector<const ChildT*> leaves;
        for (const auto* n : nodes)
            for (auto it = n->cbeginChildOn(); it; ++it)
                leaves.push_back(&(*it));
        for (const auto* leaf : leaves)
            writeMask(cursor, leaf->getValueMask());
    }
}

// ---------------------------------------------------------------------------
// Main write function
// ---------------------------------------------------------------------------

template <typename GridT>
void valueMaskTopologyWrite(std::ostream& os, const GridBase& gridBase)
{
    const GridT& grid = static_cast<const GridT&>(gridBase);
    using TreeT     = typename GridT::TreeType;
    using RootT     = typename TreeT::RootNodeType;
    using Internal1 = typename RootT::ChildNodeType;

    const RootT& root = grid.tree().root();

    // --- Collect root tiles ---
    const uint32_t numTiles        = static_cast<uint32_t>(root.tileCount());
    const uint32_t numRootChildren = static_cast<uint32_t>(root.childCount());

    std::vector<Int32>  tileX, tileY, tileZ;
    std::vector<uint8_t> tileActive;
    tileX.reserve(numTiles); tileY.reserve(numTiles);
    tileZ.reserve(numTiles); tileActive.reserve(numTiles);
    for (auto it = root.cbeginValueAll(); it; ++it) {
        const Coord& ijk = it.getCoord();
        tileX.push_back(ijk.x());
        tileY.push_back(ijk.y());
        tileZ.push_back(ijk.z());
        tileActive.push_back(static_cast<uint8_t>(it.isValueOn() ? 1 : 0));
    }

    // --- Collect root children ---
    std::vector<const Internal1*> int1nodes;
    int1nodes.reserve(numRootChildren);
    for (auto it = root.cbeginChildOn(); it; ++it)
        int1nodes.push_back(&(*it));

    // --- Pre-compute exact buffer size (single allocation, no realloc) ---
    size_t bufSize =
          sizeof(uint32_t) * 2                                          // numTiles, numRootChildren
        + static_cast<size_t>(numTiles) * (3 * sizeof(Int32) + sizeof(uint8_t))  // tile origins + active
        + static_cast<size_t>(numRootChildren) * 3 * sizeof(Int32);   // child origins

    if (!int1nodes.empty())
        bufSize += computeMaskBytes(int1nodes);

    // --- Allocate and fill buffer ---
    std::vector<char> buffer(bufSize);
    char* cursor = buffer.data();

    // Header
    std::memcpy(cursor, &numTiles,        sizeof(uint32_t)); cursor += sizeof(uint32_t);
    std::memcpy(cursor, &numRootChildren, sizeof(uint32_t)); cursor += sizeof(uint32_t);

    // Tile origins, component-separated for better ZSTD compression
    for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(cursor, &tileX[i], sizeof(Int32)); cursor += sizeof(Int32); }
    for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(cursor, &tileY[i], sizeof(Int32)); cursor += sizeof(Int32); }
    for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(cursor, &tileZ[i], sizeof(Int32)); cursor += sizeof(Int32); }
    for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(cursor, &tileActive[i], sizeof(uint8_t)); cursor += sizeof(uint8_t); }

    // Root child origins, component-separated
    for (const auto* n : int1nodes) { Int32 v = n->origin().x(); std::memcpy(cursor, &v, sizeof(Int32)); cursor += sizeof(Int32); }
    for (const auto* n : int1nodes) { Int32 v = n->origin().y(); std::memcpy(cursor, &v, sizeof(Int32)); cursor += sizeof(Int32); }
    for (const auto* n : int1nodes) { Int32 v = n->origin().z(); std::memcpy(cursor, &v, sizeof(Int32)); cursor += sizeof(Int32); }

    // Pass 1: child masks breadth-first
    if (!int1nodes.empty())
        writeChildMasks(cursor, int1nodes);

    // Pass 2: value masks breadth-first
    if (!int1nodes.empty())
        writeValueMasks(cursor, int1nodes);

    OPENVDB_ASSERT(cursor == buffer.data() + bufSize);

    compressAndWrite(os, buffer.data(), bufSize);
}

// ---------------------------------------------------------------------------
// Read pass 1: allocate tree structure from child masks
// ---------------------------------------------------------------------------

/// Reads child masks for @p nodes, allocates child nodes, inserts them, and
/// recurses until the leaf level.
template <typename InternalT>
void allocateChildren(const char*& cursor,
    std::vector<InternalT*>& nodes,
    const typename InternalT::ValueType& background)
{
    using ChildT    = typename InternalT::ChildNodeType;
    using NodeMaskT = typename InternalT::NodeMaskType;

    std::vector<ChildT*> children;
    for (auto* n : nodes) {
        NodeMaskT childMask;
        readMask(cursor, childMask);
        for (auto it = childMask.beginOn(); it; ++it) {
            Coord   origin = n->offsetToGlobalCoord(it.pos());
            ChildT* child  = new ChildT(PartialCreate(), origin, background);
            n->setChildUnsafe(it.pos(), child);
            children.push_back(child);
        }
    }

    if constexpr (ChildT::LEVEL > 0) {
        allocateChildren(cursor, children, background);
    }
    // ChildT::LEVEL == 0 → children are leaves, no further recursion needed.
}

// ---------------------------------------------------------------------------
// Read pass 2: apply value masks
// ---------------------------------------------------------------------------

/// Reads and applies value masks for @p nodes and all levels below.
/// Uses InternalNode::setValueMask(Index, bool) (respects child mask) for
/// internal nodes to avoid the assert in setValueMaskUnsafe when children
/// are already present.
template <typename InternalT>
void applyValueMasks(const char*& cursor, std::vector<InternalT*>& nodes)
{
    using ChildT    = typename InternalT::ChildNodeType;
    using NodeMaskT = typename InternalT::NodeMaskType;

    for (auto* n : nodes) {
        NodeMaskT valueMask;
        readMask(cursor, valueMask);
        // setValueMask(Index, bool) guards against setting a child position
        // active, which makes it safe to call after pass 1 has placed children.
        for (auto it = valueMask.beginOn(); it; ++it)
            n->setValueMask(it.pos(), true);
    }

    if constexpr (ChildT::LEVEL > 0) {
        std::vector<ChildT*> children;
        for (auto* n : nodes)
            for (auto it = n->beginChildOn(); it; ++it)
                children.push_back(&(*it));
        applyValueMasks(cursor, children);
    } else {
        // Children are leaves.
        std::vector<ChildT*> leaves;
        for (auto* n : nodes)
            for (auto it = n->beginChildOn(); it; ++it)
                leaves.push_back(&(*it));
        using LeafMaskT = typename ChildT::NodeMaskType;
        for (auto* leaf : leaves) {
            LeafMaskT valueMask;
            readMask(cursor, valueMask);
            leaf->setValueMask(valueMask);
        }
    }
}

// ---------------------------------------------------------------------------
// Main read function
// ---------------------------------------------------------------------------

template <typename GridT>
void valueMaskTopologyRead(GridBase& gridBase, std::istream& is)
{
    GridT& grid = static_cast<GridT&>(gridBase);
    using TreeT     = typename GridT::TreeType;
    using RootT     = typename TreeT::RootNodeType;
    using Internal1 = typename RootT::ChildNodeType;
    using ValueT    = typename GridT::ValueType;

    TreeT& tree = grid.tree();
    RootT& root = tree.root();
    root.clear();

    std::vector<char> buffer = readAndDecompress(is);
    const char* cursor = buffer.data();

    // Header
    uint32_t numTiles = 0, numRootChildren = 0;
    std::memcpy(&numTiles,        cursor, sizeof(uint32_t)); cursor += sizeof(uint32_t);
    std::memcpy(&numRootChildren, cursor, sizeof(uint32_t)); cursor += sizeof(uint32_t);

    const ValueT& background = root.background();
    const ValueT  zero       = zeroVal<ValueT>();

    // Read root tiles
    if (numTiles > 0) {
        std::vector<Int32>   tileX(numTiles), tileY(numTiles), tileZ(numTiles);
        std::vector<uint8_t> tileActive(numTiles);
        for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(&tileX[i],      cursor, sizeof(Int32));   cursor += sizeof(Int32); }
        for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(&tileY[i],      cursor, sizeof(Int32));   cursor += sizeof(Int32); }
        for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(&tileZ[i],      cursor, sizeof(Int32));   cursor += sizeof(Int32); }
        for (uint32_t i = 0; i < numTiles; ++i) { std::memcpy(&tileActive[i], cursor, sizeof(uint8_t)); cursor += sizeof(uint8_t); }
        for (uint32_t i = 0; i < numTiles; ++i)
            root.addTile(Coord(tileX[i], tileY[i], tileZ[i]), zero, tileActive[i] != 0);
    }

    if (numRootChildren > 0) {
        // Read root child origins
        std::vector<Int32> childX(numRootChildren), childY(numRootChildren), childZ(numRootChildren);
        for (uint32_t i = 0; i < numRootChildren; ++i) { std::memcpy(&childX[i], cursor, sizeof(Int32)); cursor += sizeof(Int32); }
        for (uint32_t i = 0; i < numRootChildren; ++i) { std::memcpy(&childY[i], cursor, sizeof(Int32)); cursor += sizeof(Int32); }
        for (uint32_t i = 0; i < numRootChildren; ++i) { std::memcpy(&childZ[i], cursor, sizeof(Int32)); cursor += sizeof(Int32); }

        // Allocate root children and register with root
        std::vector<Internal1*> int1nodes;
        int1nodes.reserve(numRootChildren);
        for (uint32_t i = 0; i < numRootChildren; ++i) {
            Coord     origin(childX[i], childY[i], childZ[i]);
            Internal1* child = new Internal1(PartialCreate(), origin, background);
            root.addChild(child);
            int1nodes.push_back(child);
        }

        // Pass 1: read child masks and allocate all descendant nodes
        allocateChildren(cursor, int1nodes, background);

        // Pass 2: read and apply value masks for all internal and leaf nodes
        applyValueMasks(cursor, int1nodes);
    }

    tree.clearAllAccessors();
}

} // namespace internal

// ===========================================================================
// Public codec struct
// ===========================================================================

/// @brief Topology-only codec that stores child masks and value masks using a
///   breadth-first binary layout, optionally ZSTD-compressed.
///
/// @tparam GridT  The concrete grid type (e.g. @c FloatGrid).
/// @tparam Mode   @c ReadWrite (default) or @c ReadOnly.
template <typename GridT, io::CodecMode Mode = io::CodecMode::ReadWrite>
struct ValueMaskTopologyCodec final : public io::Codec
{
    using Ptr = std::unique_ptr<ValueMaskTopologyCodec<GridT, Mode>>;

    ~ValueMaskTopologyCodec() noexcept = default;

    static std::string name() { return "valuemask_topology"; }

    io::CodecData::Ptr createData() override
    {
        auto data = std::make_unique<io::CodecData>();
        data->grid = GridT::create();
        return data;
    }

    void readTopology(std::istream& is, io::CodecData& data,
        const io::ReadOptions& /*options*/,
        io::ReadDiagnostics& /*diagnostics*/) const final
    {
        internal::valueMaskTopologyRead<GridT>(*data.grid, is);
    }

    void writeTopology(std::ostream& os, const GridBase& grid,
        const io::WriteOptions& /*options*/) const final
    {
        if constexpr (Mode == io::CodecMode::ReadOnly) return;
        internal::valueMaskTopologyWrite<GridT>(grid, os);
    }
}; // struct ValueMaskTopologyCodec

} // namespace codecs
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_VALUEMASK_TOPOLOGY_CODEC_HAS_BEEN_INCLUDED
