// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#ifndef OPENVDB_COMPACT_TOPOLOGY_CODEC_HAS_BEEN_INCLUDED
#define OPENVDB_COMPACT_TOPOLOGY_CODEC_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/io/Codec.h>
#include <openvdb/io/BufferCompression.h>
#include <openvdb/Grid.h>
#include <openvdb/util/CpuTimer.h>

#include <tbb/task_group.h>
#include <tbb/task_arena.h>

#include <array>
#include <sstream>
#include <vector>
#include <cstring>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace codecs {
namespace internal {

// Returns the occupancy of a mask: 1 for full, 0 for empty, 2 for partial.
template<typename MaskT>
static uint8_t getMaskOccupancy(const MaskT& mask)
{
    if (mask.isOn())         return uint8_t(1); // full
    else if (mask.isOff())   return uint8_t(0); // empty
    return uint8_t(2); // partial
}

// Per-level node counts plus the node-indexing arithmetic that depends only on
// them. counts is indexed by node level (counts[0] == leaf level, highest
// internal level last), matching the top-down occupancy ordering produced by the
// writer. This is the only piece of topology metadata the scalar codec needs, so
// it is the unit that crosses between the two codecs (stored in CompactCodecData).
struct NodeCounts
{
    Index32 depth = 0;
    std::vector<Index32> counts;

    // Total number of (internal + leaf) nodes.
    Index32 total() const
    {
        Index32 n = 0;
        for (const Index32 c : counts)  n += c;
        return n;
    }

    // Number of leaf nodes (leaves are the lowest level, counts[0]).
    size_t numLeaves() const { return counts.empty() ? 0 : size_t(counts[0]); }

    // Returns the global top-down node index (same ordering as occupancy buffer)
    // for a node of type NodeT at position idx within its level.
    // Writer emits highest internal level first, so all levels above NodeT::LEVEL
    // contribute their counts first.
    template<typename RootT, typename NodeT>
    size_t globalNodeIndex(size_t idx) const
    {
        using NodeChainT = typename RootT::NodeChainType;
        size_t nodeIndex = 0;
        openvdb::foreachIndex<NodeT::LEVEL + 1, NodeChainT::Size - 1>([&](auto i)
        {
            constexpr Index L = static_cast<Index>(decltype(i)::value);
            nodeIndex += counts[L];
        });
        nodeIndex += idx;
        return nodeIndex;
    }
}; // struct NodeCounts

// Describes the byte layout of the (serialized) topology buffer: the header
// fields and the offsets of each region within it. This is purely a topology-codec
// concern - it is built locally while reading or writing the topology buffer and
// never outlives that work (unlike NodeCounts, which is handed to CompactCodecData).
struct TopologyBufferLayout
{
    TopologyBufferLayout() = default;

    TopologyBufferLayout(Index32 _numRootChildren,
                         Index32 _numRootTiles,
                         NodeCounts _nodeCounts)
        : numRootChildren(_numRootChildren)
        , numRootTiles(_numRootTiles)
        , nodeCounts(std::move(_nodeCounts))
    {
        computeOffsets();
    }

    void read(const io::BufferReader& reader)
    {
        numRootChildren = reader.readAt<Index32>(0);
        numRootTiles = reader.readAt<Index32>(sizeof(Index32));
        nodeCounts.depth = reader.readAt<Index32>(2 * sizeof(Index32));
        nodeCounts.counts.resize(nodeCounts.depth - 1);
        for (Index32 i = 0; i < nodeCounts.depth - 1; ++i) {
            nodeCounts.counts[i] = reader.readAt<Index32>(3 * sizeof(Index32) + i * sizeof(Index32));
        }
        computeOffsets();
    }

    void write(const io::BufferWriter& writer) const
    {
        writer.writeAt<Index32>(0, numRootChildren);
        writer.writeAt<Index32>(sizeof(Index32), numRootTiles);
        writer.writeAt<Index32>(2 * sizeof(Index32), nodeCounts.depth);
        for (Index32 i = 0; i < nodeCounts.depth - 1; ++i) {
            writer.writeAt<Index32>(3 * sizeof(Index32) + i * sizeof(Index32), nodeCounts.counts[i]);
        }
    }

    Index32 numRootChildren = 0;
    Index32 numRootTiles = 0;
    NodeCounts nodeCounts;
    size_t rootOriginOffset = 0;
    size_t rootTileActiveOffset = 0;
    size_t nodeMaskOffset = 0;

private:
    void computeOffsets()
    {
        size_t offset = 0;
        offset += 3 * sizeof(Index32);                           // numRootChildren, numRootTiles, depth
        offset += (nodeCounts.depth - 1) * sizeof(Index32);      // nodeCounts
        rootOriginOffset = offset;
        offset += 3 * (numRootChildren + numRootTiles) * sizeof(Index32);
        rootTileActiveOffset = offset;
        offset += numRootTiles * sizeof(uint8_t);
        nodeMaskOffset = offset;
    }
}; // struct TopologyBufferLayout

template<typename TreeT>
struct ReadChildMasksOp
{
    using RootT = typename TreeT::RootNodeType;
    using LeafT = typename TreeT::LeafNodeType;

    explicit ReadChildMasksOp(const io::BufferReader& _reader, const TopologyBufferLayout& _layout):
        reader(_reader), layout(_layout) { }

    bool operator()(RootT& root, size_t) const { return true; }

    template <typename NodeT>
    bool operator()(NodeT& node, size_t idx) const
    {
        using ChildT = typename NodeT::ChildNodeType;
        using NodeMaskT = typename NodeT::NodeMaskType;
        using NodeChainT = typename RootT::NodeChainType;

        // Writer emits child masks highest-level-first, so this level's block is
        // preceded by every higher internal level's masks. Use each level's real
        // mask type to get the byte stride (mirrors the writer's allocation).
        size_t offset = layout.nodeMaskOffset;
        openvdb::foreachIndex<NodeT::LEVEL + 1, NodeChainT::Size - 1>([&](auto i) {
            constexpr Index L = static_cast<Index>(decltype(i)::value);
            using LevelMaskT = typename NodeChainT::template Get<L>::NodeMaskType;
            offset += layout.nodeCounts.counts[L] * sizeof(LevelMaskT);
        });
        offset += idx * sizeof(NodeMaskT);

        const NodeMaskT mask = reader.readAt<NodeMaskT>(offset);

        for (auto it = mask.beginOn(); it; ++it) {
            std::unique_ptr<ChildT> child(new ChildT(PartialCreate(), node.offsetToGlobalCoord(it.pos()), zeroVal<typename ChildT::ValueType>()));
            node.setChildUnsafe(it.pos(), child.release());
        }

        return true;
    }

    bool operator()(LeafT& leaf, size_t) const { return false; }

    const io::BufferReader& reader;
    const TopologyBufferLayout& layout;
}; // struct ReadChildMasksOp

template<typename TreeT>
struct ReadOccupancyValueMasksOp
{
    using RootT = typename TreeT::RootNodeType;

    ReadOccupancyValueMasksOp(const io::BufferReader& _occupancyReader,
                              const internal::NodeCounts& _nodeCounts)
        : occupancyReader(_occupancyReader), nodeCounts(_nodeCounts) { }

    bool operator()(RootT& root, size_t) const { return true; }

    template <typename NodeT>
    bool operator()(NodeT& node, size_t idx) const
    {
        const size_t nodeIndex = nodeCounts.globalNodeIndex<RootT, NodeT>(idx);
        uint8_t occ = occupancyReader.readAt<uint8_t>(nodeIndex);

        if (occ == uint8_t(1)) {
            using NodeMaskT = typename NodeT::NodeMaskType;
            NodeMaskT mask;
            mask.setOn();
            if constexpr (NodeT::LEVEL == 0) {
                node.setValueMask(mask);
            } else {
                node.setValueMaskUnsafe(mask);
            }
        }
        // partial (2) - defer to ReadValueMasksOp
        // empty (0) - value mask already off

        return true;
    }

    const io::BufferReader& occupancyReader;
    const internal::NodeCounts& nodeCounts;
}; // struct ReadOccupancyValueMasksOp

template<typename TreeT>
struct ComputeValueMaskOffsetsOp
{
    using RootT = typename TreeT::RootNodeType;

    ComputeValueMaskOffsetsOp(const io::BufferReader& _occupancyReader,
                              const internal::NodeCounts& _nodeCounts,
                              std::unique_ptr<size_t[]>& _valueMaskOffsets)
        : occupancyReader(_occupancyReader)
        , nodeCounts(_nodeCounts)
        , valueMaskOffsets(_valueMaskOffsets) {}

    bool operator()(RootT&, size_t) const { return true; }

    template <typename NodeT>
    bool operator()(NodeT&, size_t idx) const
    {
        const size_t nodeIndex = nodeCounts.globalNodeIndex<RootT, NodeT>(idx);
        if (occupancyReader.readAt<uint8_t>(nodeIndex) == uint8_t(2)) {
            valueMaskOffsets[nodeIndex + 1] = sizeof(typename NodeT::NodeMaskType);
        } else {
            valueMaskOffsets[nodeIndex + 1] = 0;
        }
        return true;
    }

    const io::BufferReader& occupancyReader;
    const internal::NodeCounts& nodeCounts;
    std::unique_ptr<size_t[]>& valueMaskOffsets;
}; // struct ComputeValueMaskOffsetsOp

template<typename TreeT>
struct ReadValueMasksOp
{
    using RootT = typename TreeT::RootNodeType;

    ReadValueMasksOp(const io::BufferReader& _valueMasksReader,
                     const std::unique_ptr<size_t[]>& _valueMaskOffsets,
                     const internal::NodeCounts& _nodeCounts)
        : valueMasksReader(_valueMasksReader)
        , valueMaskOffsets(_valueMaskOffsets)
        , nodeCounts(_nodeCounts) {}

    bool operator()(RootT& root, size_t) const { return true; }

    template <typename NodeT>
    bool operator()(NodeT& node, size_t idx) const
    {
        const size_t nodeIndex = nodeCounts.globalNodeIndex<RootT, NodeT>(idx);
        const size_t end = valueMaskOffsets[nodeIndex + 1];
        const size_t prev = valueMaskOffsets[nodeIndex];
        if (end > prev) {
            using NodeMaskT = typename NodeT::NodeMaskType;
            if constexpr (NodeT::LEVEL == 0) {
                node.setValueMask(valueMasksReader.readAt<NodeMaskT>(prev));
            } else {
                node.setValueMaskUnsafe(valueMasksReader.readAt<NodeMaskT>(prev));
            }
        }
        return true;
    }

    const io::BufferReader& valueMasksReader;
    const std::unique_ptr<size_t[]>& valueMaskOffsets;
    const internal::NodeCounts& nodeCounts;
}; // struct ReadValueMasksOp

template<typename TreeT>
struct WriteChildMasksOp
{
    using RootT = typename TreeT::RootNodeType;
    using LeafT = typename TreeT::LeafNodeType;

    WriteChildMasksOp(const io::BufferWriter& _writer, const internal::TopologyBufferLayout& _layout)
        : writer(_writer), layout(_layout) { }

    bool operator()(const RootT&, size_t) const { return true; }

    template <typename NodeT>
    bool operator()(const NodeT& node, size_t idx) const
    {
        using NodeMaskT = typename NodeT::NodeMaskType;
        using NodeChainT = typename RootT::NodeChainType;
        size_t offset = layout.nodeMaskOffset;
        openvdb::foreachIndex<NodeT::LEVEL + 1, NodeChainT::Size - 1>([&](auto i) {
            constexpr Index L = static_cast<Index>(decltype(i)::value);
            using LevelMaskT = typename NodeChainT::template Get<L>::NodeMaskType;
            offset += layout.nodeCounts.counts[L] * sizeof(LevelMaskT);
        });
        offset += idx * sizeof(NodeMaskT);
        writer.writeAt<NodeMaskT>(offset, node.getChildMask());
        return true;
    }

    bool operator()(const LeafT&, size_t) const { return false; }

    const io::BufferWriter& writer;
    const internal::TopologyBufferLayout& layout;
}; // struct WriteChildMasksOp

template<typename TreeT>
struct WriteOccupancyValueMasksOp
{
    using RootT = typename TreeT::RootNodeType;

    WriteOccupancyValueMasksOp(const io::BufferWriter& _writer, const internal::NodeCounts& _nodeCounts)
        : writer(_writer), nodeCounts(_nodeCounts) { }

    bool operator()(const RootT&, size_t) const { return true; }

    template <typename NodeT>
    bool operator()(const NodeT& node, size_t idx) const
    {
        const size_t nodeIndex = nodeCounts.globalNodeIndex<RootT, NodeT>(idx);
        writer.writeAt<uint8_t>(nodeIndex, getMaskOccupancy(node.getValueMask()));
        return true;
    }

    const io::BufferWriter& writer;
    const internal::NodeCounts& nodeCounts;
}; // struct WriteOccupancyValueMasksOp

template<typename TreeT>
struct WriteComputeValueMaskOffsetsOp
{
    using RootT = typename TreeT::RootNodeType;

    WriteComputeValueMaskOffsetsOp(const internal::NodeCounts& _nodeCounts,
                                   std::vector<size_t>& _valueMaskOffsets)
        : nodeCounts(_nodeCounts)
        , valueMaskOffsets(_valueMaskOffsets) {}

    bool operator()(const RootT&, size_t) const { return true; }

    // Records each partial node's value-mask byte size at [nodeIndex + 1]; a
    // subsequent serial prefix sum turns these into absolute byte offsets. This
    // mirrors the read-side ComputeValueMaskOffsetsOp, so the per-node work has
    // no cross-node dependency and may run threaded. Leaf nodes fall through to
    // this template (the returned bool is irrelevant for them).
    template <typename NodeT>
    bool operator()(const NodeT& node, size_t idx) const
    {
        const size_t nodeIndex = nodeCounts.globalNodeIndex<RootT, NodeT>(idx);
        if (getMaskOccupancy(node.getValueMask()) == uint8_t(2)) {
            valueMaskOffsets[nodeIndex + 1] = sizeof(typename NodeT::NodeMaskType);
        } else {
            valueMaskOffsets[nodeIndex + 1] = 0;
        }
        return true;
    }

    const internal::NodeCounts& nodeCounts;
    std::vector<size_t>& valueMaskOffsets;
}; // struct WriteComputeValueMaskOffsetsOp

template<typename TreeT>
struct WriteValueMasksOp
{
    using RootT = typename TreeT::RootNodeType;

    WriteValueMasksOp(const io::BufferWriter& _writer,
                      const std::vector<size_t>& _valueMaskOffsets,
                      const internal::NodeCounts& _nodeCounts)
        : writer(_writer)
        , valueMaskOffsets(_valueMaskOffsets)
        , nodeCounts(_nodeCounts) {}

    bool operator()(const RootT&, size_t) const { return true; }

    // valueMaskOffsets is the prefix-summed array; a node has a stored value
    // mask when its byte range is non-empty (end > prev), with prev being its
    // start offset. Mirrors the read-side ReadValueMasksOp. Leaf nodes fall
    // through to this template (the returned bool is irrelevant for them).
    template <typename NodeT>
    bool operator()(const NodeT& node, size_t idx) const
    {
        const size_t nodeIndex = nodeCounts.globalNodeIndex<RootT, NodeT>(idx);
        const size_t end = valueMaskOffsets[nodeIndex + 1];
        const size_t prev = valueMaskOffsets[nodeIndex];
        if (end > prev) {
            using NodeMaskT = typename NodeT::NodeMaskType;
            writer.writeAt<NodeMaskT>(prev, node.getValueMask());
        }
        return true;
    }

    const io::BufferWriter& writer;
    const std::vector<size_t>& valueMaskOffsets;
    const internal::NodeCounts& nodeCounts;
}; // struct WriteValueMasksOp

} // namespace internal

struct CompactCodecData : public io::CodecData {
    // Safety net: a tbb::task_group with pending tasks calls std::terminate if
    // destroyed without being waited on. readBuffers() normally joins it, but it
    // may be skipped (e.g. TopologyOnly reads), so guarantee the join here. The
    // destructor body runs before the members below are torn down, so the task's
    // referenced state is still alive at this point.
    ~CompactCodecData() noexcept override { taskGroup.wait(); }

    // State produced during readTopology() that the deferred (F) task continues
    // to reference after readTopology() returns. These must outlive the task,
    // so they are owned here (the wait happens later in readBuffers()).
    // Declared before taskGroup so the task group is destroyed first. Only the
    // node counts cross this boundary (the topology buffer's byte layout is a
    // readTopology() local); the scalar codec also reads nodeCounts from here.
    internal::NodeCounts nodeCounts;
    io::IOBuffers valueMasksIOBuffers;
    std::unique_ptr<size_t[]> valueMaskOffsets;
    bool compression = true;

    tbb::task_group taskGroup;
};

template <typename GridT>
struct CompactTopologyCodec : public io::Codec
{
    using Ptr = std::unique_ptr<CompactTopologyCodec<GridT>>;

    ~CompactTopologyCodec() noexcept = default;

    io::CodecData::Ptr createData() override
    {
        auto data = std::make_unique<CompactCodecData>();
        data->grid = GridT::create();
        return data;
    }

    void readTopology(std::istream& is, io::CodecData& data, const io::ReadOptions& options,
        io::ReadDiagnostics& diagnostics) const final
    {
        using TreeT = typename GridT::TreeType;
        using RootT = typename TreeT::RootNodeType;
        using RootChildT = typename RootT::ChildNodeType;
        using ValueType = typename TreeT::ValueType;

        using namespace internal;

        CompactCodecData& codecData = static_cast<CompactCodecData&>(data);

        uint8_t compressionFlag = 0;
        is.read(reinterpret_cast<char*>(&compressionFlag), sizeof(uint8_t));
        const bool compression = compressionFlag == 1;
        codecData.compression = compression;

        GridT& grid = static_cast<GridT&>(*codecData.grid);
        grid.tree().clearAllAccessors();

        RootT& root = grid.tree().root();

        ///////////////////////////////////////////////////////////////////////////
        // (A)
        ///////////////////////////////////////////////////////////////////////////

        // Each block is self-describing: IOBuffers::write prefixes the payload
        // with its byte size, and IOBuffers::read (size < 0) reads that prefix
        // inline and then exactly that many bytes. This keeps the read strictly
        // forward-only (no seek), matching the scalar codec's non-seekable path.

        auto readBlock = [&is, compression](io::IOBuffers& buffers) {
            buffers.read(is, compression);
        };

        io::IOBuffers topologyIOBuffers;

        readBlock(topologyIOBuffers);

        ///////////////////////////////////////////////////////////////////////////
        // (D)
        ///////////////////////////////////////////////////////////////////////////

        // The topology buffer's byte layout is only needed while parsing that
        // buffer (this task group, joined below), so it is a local. Only its node
        // counts outlive readTopology(); they are handed to codecData after the
        // join for the deferred (F) task and the scalar codec.
        internal::TopologyBufferLayout layout;
        Index32 numNodes = 0;
        tree::DynamicNodeManager<TreeT> nodeManager(grid.tree());

        tbb::task_group bdTaskGroup;
        bdTaskGroup.run([&]() {
            topologyIOBuffers.decompress();

            io::BufferReader topologyReader = topologyIOBuffers.uncompressedReader();
            layout.read(topologyReader);

            // read number of root node children and their origins

            const Index32 numRootChildren = layout.numRootChildren;
            const Index32 numRootTiles = layout.numRootTiles;

            numNodes = layout.nodeCounts.total();

            // read root origins

            const size_t stride = (numRootChildren + numRootTiles) * sizeof(Index32);
            for (Index32 i = 0; i < numRootChildren; ++i) {
                // read origin
                const size_t offset = layout.rootOriginOffset;
                const size_t offsetX = offset + i * sizeof(Index32);
                const size_t offsetY = offset + stride + i * sizeof(Index32);
                const size_t offsetZ = offset + 2 * stride + i * sizeof(Index32);
                Coord origin(topologyReader.readAt<Index32>(offsetX), topologyReader.readAt<Index32>(offsetY), topologyReader.readAt<Index32>(offsetZ));
                // create and add child
                std::unique_ptr<RootChildT> child(new RootChildT());
                child->setOrigin(origin);
                root.addChild(child.release());
            }

            // read all root tile active states

            for (Index32 i = 0; i < numRootTiles; ++i) {
                // read origin
                const size_t offset = layout.rootOriginOffset + numRootChildren * sizeof(Index32);
                const size_t offsetX = offset + i * sizeof(Index32);
                const size_t offsetY = offset + stride + i * sizeof(Index32);
                const size_t offsetZ = offset + 2 * stride + i * sizeof(Index32);
                Coord origin(topologyReader.readAt<Index32>(offsetX), topologyReader.readAt<Index32>(offsetY), topologyReader.readAt<Index32>(offsetZ));
                // read active state
                const size_t activeOffset = layout.rootTileActiveOffset + i * sizeof(uint8_t);
                const bool active = topologyReader.readAt<uint8_t>(activeOffset) == uint8_t(1);
                // add tile
                root.addTile(origin, zeroVal<ValueType>(), active);
            }

            // allocate internal nodes

            ReadChildMasksOp<TreeT> readChildMasksOp(topologyReader, layout);
            nodeManager.foreachTopDown(readChildMasksOp, /*threaded=*/true);

        });

        ///////////////////////////////////////////////////////////////////////////
        // (B)
        ///////////////////////////////////////////////////////////////////////////

        io::IOBuffers occupancyIOBuffers;

        bdTaskGroup.run([&]() {

            readBlock(occupancyIOBuffers);
        });

        bdTaskGroup.wait();

        // The byte-layout offsets are done with; only the node counts cross into
        // the deferred tasks below and the scalar codec, so hand them to codecData.
        codecData.nodeCounts = std::move(layout.nodeCounts);

        ///////////////////////////////////////////////////////////////////////////
        // (E)
        ///////////////////////////////////////////////////////////////////////////

        // Owned by codecData so the deferred (F) task can keep using it after
        // readTopology() returns.
        std::unique_ptr<size_t[]>& valueMaskOffsets = codecData.valueMaskOffsets;

        tbb::task_group ceTaskGroup;
        ceTaskGroup.run([&]() {
            occupancyIOBuffers.decompress();

            // read all value masks

            OPENVDB_ASSERT(occupancyIOBuffers.uncompressedBuffer.size() == size_t(numNodes));

            io::BufferReader occupancyReader = occupancyIOBuffers.uncompressedReader();

            ReadOccupancyValueMasksOp<TreeT> readOccupancyValueMasksOp(occupancyReader, codecData.nodeCounts);
            nodeManager.foreachTopDown(readOccupancyValueMasksOp, /*threaded=*/true);

            valueMaskOffsets = std::make_unique<size_t[]>(numNodes+1);
            valueMaskOffsets[0] = 0;

            ComputeValueMaskOffsetsOp<TreeT> computeValueMaskOffsetsOp(occupancyReader, codecData.nodeCounts, valueMaskOffsets);
            nodeManager.foreachTopDown(computeValueMaskOffsetsOp, /*threaded=*/true);

            // prefix sum

            for (size_t i = 1; i <= numNodes; ++i) {
                valueMaskOffsets[i] += valueMaskOffsets[i - 1];
            }

        });
        // ceTaskGroup.wait();

        ///////////////////////////////////////////////////////////////////////////
        // (C)
        ///////////////////////////////////////////////////////////////////////////

        // Owned by codecData so the deferred (F) task can keep using it after
        // readTopology() returns.
        io::IOBuffers& valueMasksIOBuffers = codecData.valueMasksIOBuffers;

        ceTaskGroup.run([&]() {
            readBlock(valueMasksIOBuffers);
        });
        ceTaskGroup.wait();

        ///////////////////////////////////////////////////////////////////////////
        // (F)
        ///////////////////////////////////////////////////////////////////////////

        // This task is intentionally NOT waited on here. It only references state
        // owned by codecData (nodeCounts, valueMaskOffsets,
        // valueMasksIOBuffers and the grid), all of which outlive readTopology().
        // The wait is deferred to CompactScalarCodec::readBuffers(). The node
        // manager is local to readTopology(), so it is recreated here from the
        // grid (which lives in codecData) rather than captured by reference.
        codecData.taskGroup.run([&codecData]() {
            codecData.valueMasksIOBuffers.decompress();

            GridT& grid = static_cast<GridT&>(*codecData.grid);
            tree::DynamicNodeManager<TreeT> nodeManager(grid.tree());

            io::BufferReader valueMasksReader = codecData.valueMasksIOBuffers.uncompressedReader();
            ReadValueMasksOp<TreeT> readValueMasksOp(
                valueMasksReader, codecData.valueMaskOffsets, codecData.nodeCounts);
            nodeManager.foreachTopDown(readValueMasksOp, /*threaded=*/true);

        });

        // allocate the leaf nodes asynchronously

        codecData.taskGroup.run([&]() {
            using LeafT = typename TreeT::LeafNodeType;
            tree::LeafManager<TreeT> leafManager(grid.tree());

            leafManager.foreach([](LeafT& leaf, size_t) { leaf.allocate(); }, /*threaded=*/true);
        });

        // NOTE: codecData.taskGroup is intentionally NOT waited on here.
        // The deferred task for allocation must be joined in readBuffers() before writing to the grid.
    }

    void writeTopology(std::ostream& os, const GridBase& gridBase, const io::WriteOptions& options) const final
    {
        using TreeT = typename GridT::TreeType;
        using RootT = typename TreeT::RootNodeType;

        using namespace internal;

        // compression is on by default, store whether it is enabled to the stream

        const bool compressed = options.compression;

        uint8_t compressedFlag = compressed ? 1 : 0;
        os.write(reinterpret_cast<const char*>(&compressedFlag), sizeof(uint8_t));

        const GridT& grid = static_cast<const GridT&>(gridBase);
        const auto& root = grid.tree().root();

        // Step 1: build root origin / tile data

        const Index32 numRootChildren = root.childCount();
        const Index32 numRootTiles = root.tileCount();
        std::vector<Int32> rootOrigins;
        rootOrigins.reserve(3 * (numRootChildren + numRootTiles));
        for (auto it = root.cbeginChildOn(); it; ++it)  rootOrigins.push_back(it->origin().x());
        for (auto it = root.cbeginValueAll(); it; ++it)  rootOrigins.push_back(it.getCoord().x());
        for (auto it = root.cbeginChildOn(); it; ++it)  rootOrigins.push_back(it->origin().y());
        for (auto it = root.cbeginValueAll(); it; ++it)  rootOrigins.push_back(it.getCoord().y());
        for (auto it = root.cbeginChildOn(); it; ++it)  rootOrigins.push_back(it->origin().z());
        for (auto it = root.cbeginValueAll(); it; ++it)  rootOrigins.push_back(it.getCoord().z());

        std::vector<uint8_t> rootTileActive;
        rootTileActive.reserve(numRootTiles);
        for (auto it = root.cbeginValueAll(); it; ++it)
            rootTileActive.push_back(it.isValueOn() ? uint8_t(1) : uint8_t(0));

        // Step 2: gather per-level node counts

        using NodeChainT = typename RootT::NodeChainType;
        constexpr Index Depth = TreeT::DEPTH;

        tree::NodeManager<const TreeT> staticNodeManager(grid.tree());
        const Index32 depth = static_cast<Index32>(grid.tree().treeDepth());

        internal::NodeCounts nodeCounts;
        nodeCounts.depth = depth;
        nodeCounts.counts.resize(depth - 1);
        for (Index32 i = 0; i < depth - 1; ++i) {
            nodeCounts.counts[i] = staticNodeManager.nodeCount(i);
        }
        const Index32 numNodes = nodeCounts.total();

        // Step 3: lay out the topology buffer, then size and allocate it

        internal::TopologyBufferLayout layout(numRootChildren, numRootTiles, nodeCounts);

        size_t childMaskRegionSize = 0;
        openvdb::foreachIndex<0, NodeChainT::Size>([&](auto idx) {
            constexpr Index L = static_cast<Index>(decltype(idx)::value);
            if constexpr (L >= 1 && L + 1 < Depth) {
                using NodeT = typename NodeChainT::template Get<L>;
                using MaskT = typename NodeT::NodeMaskType;
                childMaskRegionSize += nodeCounts.counts[L] * sizeof(MaskT);
            }
        });

        const size_t topologySize = layout.nodeMaskOffset + childMaskRegionSize;
        io::IOBuffers topologyBuffers;
        io::BufferWriter topologyWriter = topologyBuffers.allocate(int64_t(topologySize));

        // Step 4: write header into topology buffer

        layout.write(topologyWriter);

        // Step 5: write root origins and tile active states

        topologyWriter.writeFromVector(rootOrigins, layout.rootOriginOffset);

        topologyWriter.writeFromVector(rootTileActive, layout.rootTileActiveOffset);

        // Step 6: allocate occupancy buffer and build dynamic node manager

        const size_t occupancySize = numNodes;
        io::IOBuffers occupancyBuffers;
        io::BufferWriter occupancyWriter = occupancyBuffers.allocate(int64_t(occupancySize));

        tree::DynamicNodeManager<const TreeT> nodeManager(grid.tree());

        // Step 7: write child masks (threaded)

        WriteChildMasksOp<TreeT> writeChildMasksOp(topologyWriter, layout);
        nodeManager.foreachTopDown(writeChildMasksOp, /*threaded=*/true);

        // Step 8: write occupancy (threaded)

        WriteOccupancyValueMasksOp<TreeT> writeOccupancyValueMasksOp(occupancyWriter, nodeCounts);
        nodeManager.foreachTopDown(writeOccupancyValueMasksOp, /*threaded=*/true);

        // Step 9: compute value mask byte sizes (threaded), prefix sum (serial),
        // then write value masks (threaded). This mirrors the read side: each
        // node records its mask size at [nodeIndex + 1], and the prefix sum turns
        // those into absolute byte offsets, with valueMaskOffsets[numNodes] being
        // the total value-mask region size.

        std::vector<size_t> valueMaskOffsets(numNodes + 1, 0);
        WriteComputeValueMaskOffsetsOp<TreeT> writeComputeValueMaskOffsetsOp(nodeCounts, valueMaskOffsets);
        nodeManager.foreachTopDown(writeComputeValueMaskOffsetsOp, /*threaded=*/true);

        for (size_t i = 1; i <= numNodes; ++i) {
            valueMaskOffsets[i] += valueMaskOffsets[i - 1];
        }

        const size_t valueMasksSize = valueMaskOffsets[numNodes];
        io::IOBuffers valueMasksBuffers;
        io::BufferWriter valueMasksWriter = valueMasksBuffers.allocate(int64_t(valueMasksSize));

        WriteValueMasksOp<TreeT> writeValueMasksOp(valueMasksWriter, valueMaskOffsets, nodeCounts);
        nodeManager.foreachTopDown(writeValueMasksOp, /*threaded=*/true);

        // IOBuffers::write prefixes each block with its (post-compression) byte
        // size, so the read side sizes its buffer from that inline prefix with
        // no offset table and no backward seek - the write stays a single
        // forward pass. This mirrors the scalar codec's framing.
        topologyBuffers.write(os, compressed);
        occupancyBuffers.write(os, compressed);
        valueMasksBuffers.write(os, compressed);
    }
}; // struct CompactTopologyCodec

} // namespace codecs
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_COMPACT_TOPOLOGY_CODEC_HAS_BEEN_INCLUDED
