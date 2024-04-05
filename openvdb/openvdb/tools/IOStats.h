// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

#ifndef OPENVDB_TOOLS_IO_STATS_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_IO_STATS_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>
#include <openvdb/tree/NodeManager.h>

#include <appstats/AppStats.h>
#define APPSTATS_NO_TBB

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {

namespace io_stats_internal {

template <typename TreeT>
struct ComputeStatsOp
{
    using RootT = typename TreeT::RootNodeType;
    using LeafT = typename TreeT::LeafNodeType;
    using ValueT = typename TreeT::ValueType;

    ComputeStatsOp(const TreeT& tree) : mBackground(tree.background()) { }
    ComputeStatsOp(const ComputeStatsOp& other, tbb::split)
        : mBackground(other.mBackground) { }

    // do nothing for root nodes
    void operator()(const RootT& root) { }

    // internal nodes
    template <typename NodeT>
    void operator()(const NodeT& node)
    {
        if constexpr(!std::is_same_v<ValueT, bool>) {
            using MaskT = typename NodeT::NodeMaskType;

            // Copy all of this node's values into an array.
            std::unique_ptr<ValueT[]> valuePtr(new ValueT[NodeT::NUM_VALUES]);
            ValueT* values = valuePtr.get();
            const ValueT zero = zeroVal<ValueT>();
            for (Index i = 0; i < NodeT::NUM_VALUES; ++i) {
                values[i] = (node.isChildMaskOff(i) ? node.getTable()[i].getValue() : zero);
            }
            io::MaskCompress<ValueT, MaskT> maskCompressData(node.getValueMask(), node.getChildMask(), values, mBackground);
            int8_t maskCompress = maskCompressData.metadata;
            if (maskCompress == int8_t(0))          nodeCompress0++;
            else if (maskCompress == int8_t(1))     nodeCompress1++;
            else if (maskCompress == int8_t(2))     nodeCompress2++;
            else if (maskCompress == int8_t(3))     nodeCompress3++;
            else if (maskCompress == int8_t(4))     nodeCompress4++;
            else if (maskCompress == int8_t(5))     nodeCompress5++;
        }
    }

    // leaf nodes
    void operator()(const LeafT& leaf)
    {
        if constexpr(!std::is_same_v<ValueT, bool>) {
            using MaskT = typename LeafT::NodeMaskType;

            io::MaskCompress<ValueT, MaskT> maskCompressData(leaf.valueMask(), MaskT(), leaf.buffer().data(), mBackground);
            int8_t maskCompress = maskCompressData.metadata;
            if (maskCompress == int8_t(0))          leafCompress0++;
            else if (maskCompress == int8_t(1))     leafCompress1++;
            else if (maskCompress == int8_t(2))     leafCompress2++;
            else if (maskCompress == int8_t(3))     leafCompress3++;
            else if (maskCompress == int8_t(4))     leafCompress4++;
            else if (maskCompress == int8_t(5))     leafCompress5++;
        }
    }

    void join(const ComputeStatsOp& other)
    {
        nodeCompress0 += other.nodeCompress0;
        nodeCompress1 += other.nodeCompress1;
        nodeCompress2 += other.nodeCompress2;
        nodeCompress3 += other.nodeCompress3;
        nodeCompress4 += other.nodeCompress4;
        nodeCompress5 += other.nodeCompress5;

        leafCompress0 += other.leafCompress0;
        leafCompress1 += other.leafCompress1;
        leafCompress2 += other.leafCompress2;
        leafCompress3 += other.leafCompress3;
        leafCompress4 += other.leafCompress4;
        leafCompress5 += other.leafCompress5;
    }

    const ValueT mBackground;

    int nodeCompress0 = 0;
    int nodeCompress1 = 0;
    int nodeCompress2 = 0;
    int nodeCompress3 = 0;
    int nodeCompress4 = 0;
    int nodeCompress5 = 0;

    int leafCompress0 = 0;
    int leafCompress1 = 0;
    int leafCompress2 = 0;
    int leafCompress3 = 0;
    int leafCompress4 = 0;
    int leafCompress5 = 0;
}; // struct ComputeStatsOp

template <typename TreeT>
void computeIOStats(const TreeT& tree, std::string gridClass)
{
    if constexpr(!std::is_same_v<typename TreeT::ValueType, bool>) {
        tree::NodeManager<const TreeT> nodeManager(tree);
        ComputeStatsOp<TreeT> op(tree);
        nodeManager.reduceTopDown(op, /*threaded=*/true);

        // store the grid class and value type as the "platform"

        std::stringstream ss;
        ss << gridClass << "_" << tree.valueType();
        std::string platform = ss.str();

        AppStats::Sample sample("vdbio-v1", "count", platform);

        if (op.nodeCompress0 > 0)   sample.send("node0", op.nodeCompress0);
        if (op.nodeCompress1 > 0)   sample.send("node1", op.nodeCompress1);
        if (op.nodeCompress2 > 0)   sample.send("node2", op.nodeCompress2);
        if (op.nodeCompress3 > 0)   sample.send("node3", op.nodeCompress3);
        if (op.nodeCompress4 > 0)   sample.send("node4", op.nodeCompress4);
        if (op.nodeCompress5 > 0)   sample.send("node5", op.nodeCompress5);

        if (op.leafCompress0 > 0)   sample.send("leaf0", op.leafCompress0);
        if (op.leafCompress1 > 0)   sample.send("leaf1", op.leafCompress1);
        if (op.leafCompress2 > 0)   sample.send("leaf2", op.leafCompress2);
        if (op.leafCompress3 > 0)   sample.send("leaf3", op.leafCompress3);
        if (op.leafCompress4 > 0)   sample.send("leaf4", op.leafCompress4);
        if (op.leafCompress5 > 0)   sample.send("leaf5", op.leafCompress5);
    }
}

void computeIOStats(const GridBase& gridBase)
{
    std::stringstream ss;
    auto gridClass = gridBase.getGridClass();
    if (gridClass == GRID_LEVEL_SET)        ss << "levelset";
    else if (gridClass == GRID_STAGGERED)   ss << "staggered";
    else if (gridClass == GRID_FOG_VOLUME)  ss << "fog";
    else                                    ss << "other";
    std::string gridClassStr = ss.str();

    // dynamic dispatch to typed computeIOStats method
    gridBase.apply<GridTypes>(
        [&](auto& grid) { computeIOStats(grid.tree(), gridClassStr); }
    );
}

} // namespace io_stats_internal

/// @brief Write out IO stats to AppStats
void ioStats(const GridBase& grid)
{
    io_stats_internal::computeIOStats(grid);
}


} // namespace tools
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_TOOLS_IO_STATS_HAS_BEEN_INCLUDED
