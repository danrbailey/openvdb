// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

/// @file SOP_OpenVDB_Merge.cc
///
/// @authors Dan Bailey
///
/// @brief Merge OpenVDB grids.

#include <houdini_utils/ParmFactory.h>
#include <openvdb_houdini/Utils.h>
#include <openvdb_houdini/PointUtils.h>
#include <openvdb_houdini/SOP_NodeVDB.h>
#include <openvdb_houdini/GEO_PrimVDB.h>

#include <openvdb/points/PointDataGrid.h> // points::PointDataGrid
#include <openvdb/tools/GridTransformer.h> // tools::replaceToMatch()
#include <openvdb/tools/LevelSetRebuild.h> // tools::doLevelSetRebuild()
#include <openvdb/tools/Merge.h>

#include <UT/UT_Interrupt.h>
#include <UT/UT_Version.h>
#include <stdexcept>
#include <string>
#include <vector>


using namespace openvdb;

namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;


class SOP_OpenVDB_Merge: public hvdb::SOP_NodeVDB
{
public:
    SOP_OpenVDB_Merge(OP_Network*, const char* name, OP_Operator*);
    ~SOP_OpenVDB_Merge() final {}

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);

    class Cache: public SOP_VDBCacheOptions
    {
    public:
        fpreal getTime() const { return mTime; }
    protected:
        OP_ERROR cookVDBSop(OP_Context&) final;
    private:
        fpreal mTime = 0.0;
    };

protected:
    bool updateParmsFlags() final;
};


void
newSopOperator(OP_OperatorTable* table)
{
    if (table == nullptr) return;

    hutil::ParmList parms;

    parms.add(hutil::ParmFactory(PRM_STRING, "group", "Group")
        .setTooltip("Specify a subset of the input VDBs to be modified.")
        .setChoiceList(&hutil::PrimGroupMenuInput1)
        .setDocumentation(
            "A subset of the input VDBs to be modified"
            " (see [specifying volumes|/model/volumes#group])"));

    parms.add(hutil::ParmFactory(PRM_STRING, "collation", "Collation")
        .setDefault("nameclassandtype")
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "nameclassandtype", "Name, VDB Class and Value Type",
            "classandtype",     "VDB Class and Value Type"
        })
        .setTooltip(
            "Criteria under which groups of grids are merged.")
        .setDocumentation(
            "The criteria under which groups of grids are merged.\n\n"
            "Grids with different names will be merged if name is not part"
            " of the collation criteria"));

    // Menu of resampling options
    parms.add(hutil::ParmFactory(PRM_STRING, "resample", "Resample VDBs")
        .setDefault("first")
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "first",        "To Match First VDB"
        })
        .setTypeExtended(PRM_TYPE_JOIN_PAIR)
        .setDocumentation("Specify which grid to use as reference when resampling."));

    // Menu of resampling interpolation order options
    parms.add(hutil::ParmFactory(PRM_ORD, "resampleinterp", "Interpolation")
        .setDefault(PRMoneDefaults)
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "point",     "Nearest",
            "linear",    "Linear",
            "quadratic", "Quadratic"
        })
        .setTooltip(
            "Specify the type of interpolation to be used when\n"
            "resampling one VDB to match the other's transform.")
        .setDocumentation(
            "The type of interpolation to be used when resampling one VDB"
            " to match the other's transform\n\n"
            "Nearest neighbor interpolation is fast but can introduce noticeable"
            " sampling artifacts.  Quadratic interpolation is slow but high-quality."
            " Linear interpolation is intermediate in speed and quality."));

    parms.beginSwitcher("Group1");

    parms.addFolder("Merge Operation");

    parms.add(hutil::ParmFactory(PRM_STRING, "op_sdf", "SDF VDBs")
        .setDefault("sdfunion")
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "none",             "None            ",
            "sdfunion",         "SDF Union",
            "sdfintersect",     "SDF Intersect"
        })
        .setTooltip("Merge operation for SDF VDBs.")
        .setDocumentation(
            "Merge operation for SDF VDBs.\n\n"
            "None:\n"
            "   Add grids to the output of the SOP without merging them.\n\n"
            "SDF Union:\n"
            "    Generate the union of all signed distance fields with same collation.\n\n"
            "SDF Intersection:\n"
            "    Generate the intersection of all signed distance fields with same collation.\n\n"));

    parms.endSwitcher();

    // Register this operator.
    hvdb::OpenVDBOpFactory("VDB Merge", SOP_OpenVDB_Merge::factory, parms, *table)
#ifndef SESI_OPENVDB
        .setInternalName("DW_OpenVDBMerge")
#endif
        .addInput("VDBs To Merge")
        .setMaxInputs()
        .setVerb(SOP_NodeVerb::COOK_INPLACE, []() { return new SOP_OpenVDB_Merge::Cache; })
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Merge VDB SDF grids.\"\"\"\n\
\n\
@overview\n\
\n\
Merges VDB SDF grids.\n\
\n\
Unlike the VDB Combine SOP, it provides a multi-input so can merge more than two grids without needing an \
additional merge SOP. \n\
Float and double VDB SDF grids can be merged, all other VDB grids are added to the output without merging. \n\
\n\
Grids with different transforms will be resampled to match the first VDB in the collation group. \n\
\n\
@related\n\
\n\
- [OpenVDB Combine|Node:sop/DW_OpenVDBCombine]\n\
- [Node:sop/merge]\n\
\n\
@examples\n\
\n\
See [openvdb.org|http://www.openvdb.org/download/] for source code\n\
and usage examples.\n");
}


OP_Node*
SOP_OpenVDB_Merge::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Merge(net, name, op);
}


SOP_OpenVDB_Merge::SOP_OpenVDB_Merge(OP_Network* net,
    const char* name, OP_Operator* op):
    hvdb::SOP_NodeVDB(net, name, op)
{
}


// Enable or disable parameters in the UI.
bool
SOP_OpenVDB_Merge::updateParmsFlags()
{
    bool changed = false;

    return changed;
}


namespace {


struct MergeKey
{
    enum Collation
    {
        NameClassAndType = 0,
        ClassAndType
    };

    explicit MergeKey(   const GU_PrimVDB& vdbPrim,
                    const Collation collation = NameClassAndType)
        : name(collation == NameClassAndType ? vdbPrim.getGridName() : "")
        , valueType(vdbPrim.getStorageType())
        , gridClass(vdbPrim.getGrid().getGridClass()) { }

    inline bool operator==(const MergeKey& rhs) const
    {
        return (name == rhs.name &&
                valueType == rhs.valueType &&
                gridClass == rhs.gridClass);
    }

    inline bool operator!=(const MergeKey& rhs) const { return !this->operator==(rhs); }

    inline bool operator<(const MergeKey& rhs) const
    {
        if (name < rhs.name)                                                return true;
        if (name == rhs.name) {
            if (valueType < rhs.valueType)                                  return true;
            if (valueType == rhs.valueType && gridClass < rhs.gridClass)    return true;
        }
        return false;
    }

    std::string name = "";
    UT_VDBType valueType = UT_VDB_INVALID;
    openvdb::GridClass gridClass = GRID_UNKNOWN;
}; // struct MergeKey


struct MergeOp
{
    enum Resample
    {
        Nearest = 0,
        Linear,
        Quadratic
    };

    struct OutputGrid
    {
        explicit OutputGrid(GridBase::Ptr _grid, GEO_PrimVDB* _primitive = nullptr)
            : grid(_grid), primitive(_primitive) { }

        GridBase::Ptr grid;
        GEO_PrimVDB* primitive = nullptr;
    };

    using OutputGrids = std::deque<OutputGrid>;

    using StringRemapType = std::unordered_map<std::string, std::string>;

    SOP_OpenVDB_Merge::Cache* self;
    hvdb::Interrupter& interrupt;
    StringRemapType opRemap;
    std::deque<GU_PrimVDB*> vdbPrims;
    std::deque<const GU_PrimVDB*> constVdbPrims;
    std::deque<GU_PrimVDB*> vdbPrimsToRemove;

    explicit MergeOp(hvdb::Interrupter& _interrupt): self(nullptr), interrupt(_interrupt) { }

    inline std::string getOp(const MergeKey& key) const
    {
        std::string op;

        if (key.gridClass == openvdb::GRID_LEVEL_SET) {
            if (key.valueType == UT_VDB_FLOAT || key.valueType == UT_VDB_DOUBLE) {
                op = opRemap.at("op_sdf");
            }
        }

        if (op == "none")   op = "";

        return op;
    }

    template<typename GridT>
    typename GridT::Ptr resampleToMatch(const GridT& src, const GridT& ref, int order)
    {
        using ValueT = typename GridT::ValueType;
        const ValueT ZERO = openvdb::zeroVal<ValueT>();

        const openvdb::math::Transform& refXform = ref.constTransform();

        typename GridT::Ptr dest;
        if (src.getGridClass() == openvdb::GRID_LEVEL_SET) {
            // For level set grids, use the level set rebuild tool to both resample the
            // source grid to match the reference grid and to rebuild the resulting level set.
            const bool refIsLevelSet = ref.getGridClass() == openvdb::GRID_LEVEL_SET;
            OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
            const ValueT halfWidth = refIsLevelSet
                ? ValueT(ZERO + ref.background() * (1.0 / ref.voxelSize()[0]))
                : ValueT(src.background() * (1.0 / src.voxelSize()[0]));
            OPENVDB_NO_TYPE_CONVERSION_WARNING_END

            if (!openvdb::math::isFinite(halfWidth)) {
                std::stringstream msg;
                msg << "Resample to match: Illegal narrow band width = " << halfWidth
                    << ", caused by grid '" << src.getName() << "' with background "
                    << ref.background();
                throw std::invalid_argument(msg.str());
            }

            try {
                dest = openvdb::tools::doLevelSetRebuild(src, /*iso=*/ZERO,
                    /*exWidth=*/halfWidth, /*inWidth=*/halfWidth, &refXform, &interrupt);
            } catch (openvdb::TypeError&) {
                self->addWarning(SOP_MESSAGE, ("skipped rebuild of level set grid "
                    + src.getName() + " of type " + src.type()).c_str());
                dest.reset();
            }
        }
        if (!dest && src.constTransform() != refXform) {
            // For non-level set grids or if level set rebuild failed due to an unsupported
            // grid type, use the grid transformer tool to resample the source grid to match
            // the reference grid.
            dest = src.copyWithNewTree();
            dest->setTransform(refXform.copy());
            switch (order) {
                case 0: tools::resampleToMatch<tools::PointSampler>(src, *dest, interrupt); break;
                case 1: tools::resampleToMatch<tools::BoxSampler>(src, *dest, interrupt); break;
                case 2: tools::resampleToMatch<tools::QuadraticSampler>(src, *dest, interrupt); break;
            }
        }
        return dest;
    }

    template <typename GridT>
    OutputGrids mergeTypedNoop( const MergeKey& mergeKey,
                                const MergeKey::Collation& collationKey)
    {
        using TreeT = typename GridT::TreeType;

        OutputGrids result;

        std::deque<tools::TreeToMerge<TreeT>> trees;

        tbb::task_group tasks;

        auto hasUniqueTree = [&](GU_PrimVDB* vdbPrim)
        {
            // TODO: replace with Grid::isTreeUnique() when this method is made virtual
            const TreeBase::ConstPtr treeBaseConstPtr = vdbPrim->getConstGridPtr()->constBaseTreePtr();
            return treeBaseConstPtr.use_count () <= 2;
        };

        auto stealTree = [&](auto& gridBase, GU_PrimVDB* vdbPrim = nullptr)
        {
            result.emplace_back(gridBase, vdbPrim);
        };

        auto copyTree = [&](auto& gridBase, GU_PrimVDB* vdbPrim = nullptr)
        {
            auto grid = GridBase::grid<GridT>(gridBase);
            // insert an empty shared pointer and asynchronously replace with a deep copy
            result.emplace_back(GridBase::Ptr(), vdbPrim);
            OutputGrid& output = result.back();
            tasks.run(
                [&, grid] {
                    output.grid = grid->deepCopy();
                }
            );
        };

        for (GU_PrimVDB* vdbPrim : vdbPrims) {
            MergeKey key(*vdbPrim, collationKey);
            if (key != mergeKey)    continue;

            if (hasUniqueTree(vdbPrim)) {
                GridBase::Ptr gridBase = vdbPrim->getGridPtr();
                stealTree(gridBase, vdbPrim);
            } else {
                GridBase::ConstPtr gridBase = vdbPrim->getConstGridPtr();
                copyTree(gridBase, vdbPrim);
            }
        }
        for (const GU_PrimVDB* constVdbPrim : constVdbPrims) {
            MergeKey key(*constVdbPrim, collationKey);
            if (key != mergeKey)    continue;

            GridBase::ConstPtr gridBase = constVdbPrim->getConstGridPtr();
            copyTree(gridBase);
        }

        if (interrupt.wasInterrupted())
        {
            tasks.cancel();
            return OutputGrids();
        }

        // resampling and deep copying of trees is done in parallel asynchronously,
        // now synchronize to ensure all these tasks have completed
        tasks.wait();

        if (interrupt.wasInterrupted())     return OutputGrids();

        return result;
    }

    template <typename GridT>
    OutputGrids mergeTyped( const MergeKey& mergeKey,
                            const MergeKey::Collation& collationKey)
    {
        using TreeT = typename GridT::TreeType;

        OutputGrids result;

        const std::string op = this->getOp(mergeKey);

        const int samplingOrder = static_cast<int>(
            self->evalInt("resampleinterp", 0, self->getTime()));

        GridBase::Ptr reference;

        std::deque<tools::TreeToMerge<TreeT>> trees;

        tbb::task_group tasks;

        auto hasUniqueTree = [&](GU_PrimVDB* vdbPrim)
        {
            // TODO: replace with Grid::isTreeUnique() when this method is made virtual
            const TreeBase::ConstPtr treeBaseConstPtr = vdbPrim->getConstGridPtr()->constBaseTreePtr();
            return treeBaseConstPtr.use_count () <= 2;
        };

        auto stealTree = [&](auto& gridBase, GU_PrimVDB* vdbPrim = nullptr)
        {
            result.emplace_back(gridBase, vdbPrim);
            if (!reference)  reference = gridBase;
        };

        auto copyTree = [&](auto& gridBase, GU_PrimVDB* vdbPrim = nullptr)
        {
            auto grid = GridBase::grid<GridT>(gridBase);
            if (!reference)  reference = grid->copyWithNewTree();
            // insert a reference and asynchronously replace with a deep copy
            result.emplace_back(reference, vdbPrim);
            OutputGrid& output = result.back();
            tasks.run(
                [&, grid] {
                    output.grid = grid->deepCopy();
                }
            );
        };

        auto addConstTree = [&](auto& gridBase, GU_PrimVDB* vdbPrim = nullptr)
        {
            auto grid = GridBase::grid<GridT>(gridBase);
            if (grid->constTransform() == reference->constTransform()) {
                trees.emplace_back(grid->tree(), openvdb::DeepCopy());
            } else {
                // insert an empty tree and asynchronously replace with a resampled tree
                typename TreeT::Ptr emptyTree = typename TreeT::Ptr(new TreeT);
                trees.emplace_back(emptyTree, openvdb::Steal());
                tools::TreeToMerge<TreeT>& treeToMerge = trees.back();
                tasks.run(
                    [&, grid] {
                        auto refGrid = GridBase::grid<GridT>(reference);
                        auto newGrid = this->resampleToMatch(*grid, *refGrid, samplingOrder);
                        treeToMerge.reset(newGrid->treePtr(), openvdb::Steal());
                    }
                );
            }
            if (vdbPrim)    vdbPrimsToRemove.push_back(vdbPrim);
        };

        for (GU_PrimVDB* vdbPrim : vdbPrims) {
            MergeKey key(*vdbPrim, collationKey);
            if (key != mergeKey)    continue;

            if (hasUniqueTree(vdbPrim)) {
                GridBase::Ptr gridBase = vdbPrim->getGridPtr();
                if ((!reference) || op.empty()) stealTree(gridBase, vdbPrim);
                else                            addConstTree(gridBase, vdbPrim);
            } else {
                GridBase::ConstPtr gridBase = vdbPrim->getConstGridPtr();
                if ((!reference) || op.empty()) copyTree(gridBase, vdbPrim);
                else                            addConstTree(gridBase, vdbPrim);
            }
        }
        for (const GU_PrimVDB* constVdbPrim : constVdbPrims) {
            MergeKey key(*constVdbPrim, collationKey);
            if (key != mergeKey)    continue;

            GridBase::ConstPtr gridBase = constVdbPrim->getConstGridPtr();
            if ((!reference) || op.empty())     copyTree(gridBase);
            else                                addConstTree(gridBase);
        }

        if (interrupt.wasInterrupted())
        {
            tasks.cancel();
            return OutputGrids();
        }

        // resampling and deep copying of trees is done in parallel asynchronously,
        // now synchronize to ensure all these tasks have completed
        tasks.wait();

        if (interrupt.wasInterrupted())     return OutputGrids();

        // perform merge

        if (result.size() == 1 && !trees.empty()) {
            auto grid = GridBase::grid<GridT>(result.front().grid);
            tree::DynamicNodeManager<TreeT> nodeManager(grid->tree());
            if (op == "sdfunion") {
                nodeManager.foreachTopDown(tools::CsgUnionOp<TreeT>(trees));
            } else if (op == "sdfintersect") {
                nodeManager.foreachTopDown(tools::CsgIntersectionOp<TreeT>(trees));
            }
        }

        return result;
    }

    OutputGrids merge(  const MergeKey& key,
                        const MergeKey::Collation& collationKey)
    {
        if (key.valueType == UT_VDB_FLOAT) {
            return this->mergeTyped<FloatGrid>(key, collationKey);
        } else if (key.valueType == UT_VDB_DOUBLE) {
            return this->mergeTyped<DoubleGrid>(key, collationKey);
        } else if (key.valueType == UT_VDB_INT32) {
            return this->mergeTypedNoop<Int32Grid>(key, collationKey);
        } else if (key.valueType == UT_VDB_INT64) {
            return this->mergeTypedNoop<Int64Grid>(key, collationKey);
        } else if (key.valueType == UT_VDB_BOOL) {
            return this->mergeTypedNoop<BoolGrid>(key, collationKey);
        } else if (key.valueType == UT_VDB_VEC3F) {
            return this->mergeTypedNoop<Vec3SGrid>(key, collationKey);
        } else if (key.valueType == UT_VDB_VEC3D) {
            return this->mergeTypedNoop<Vec3DGrid>(key, collationKey);
        } else if (key.valueType == UT_VDB_VEC3I) {
            return this->mergeTypedNoop<Vec3IGrid>(key, collationKey);
        } else if (key.valueType == UT_VDB_POINTDATA) {
            return this->mergeTypedNoop<points::PointDataGrid>(key, collationKey);
        }

        return OutputGrids();
    }

}; // struct MergeOp

} // namespace


OP_ERROR
SOP_OpenVDB_Merge::Cache::cookVDBSop(OP_Context& context)
{
    try {
        mTime = context.getTime();

        const std::string groupName = evalStdString("group", mTime);

        MergeKey::Collation collationKey = MergeKey::NameClassAndType;
        const std::string collation = evalStdString("collation", mTime);
        if (collation == "classandtype")    collationKey = MergeKey::ClassAndType;

        hvdb::Interrupter boss("Merging VDBs");

        MergeOp mergeOp(boss);
        mergeOp.self = this;
        mergeOp.opRemap["op_sdf"] = evalStdString("op_sdf", mTime);

        // extract non-const VDB primitives from first input

        hvdb::VdbPrimIterator vdbIt(gdp, matchGroup(*gdp, groupName));
        for (; vdbIt; ++vdbIt) {
            GU_PrimVDB* vdbPrim = *vdbIt;
            mergeOp.vdbPrims.push_back(vdbPrim);
        }

        // extract const VDB primitives from second input +

        for (int i = 1; i < nInputs(); i++) {
            const GU_Detail* pointsGeo = inputGeo(i);
            hvdb::VdbPrimCIterator vdbIt(pointsGeo, matchGroup(*pointsGeo, groupName));
            for (; vdbIt; ++vdbIt) {
                const GU_PrimVDB* constVdbPrim = *vdbIt;
                mergeOp.constVdbPrims.push_back(constVdbPrim);
            }
        }

        // extract all merge keys

        std::set<MergeKey> uniqueKeys;

        for (GU_PrimVDB* vdbPrim : mergeOp.vdbPrims) {
            MergeKey key(*vdbPrim, collationKey);
            uniqueKeys.insert(key);
        }
        for (const GU_PrimVDB* constVdbPrim : mergeOp.constVdbPrims) {
            MergeKey key(*constVdbPrim, collationKey);
            uniqueKeys.insert(key);
        }

        std::vector<MergeKey> keys(uniqueKeys.begin(), uniqueKeys.end());

        // iterate over each merge key in parallel and perform merge operations

        std::vector<MergeOp::OutputGrids> outputGridsArray;
        outputGridsArray.resize(keys.size());

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, keys.size()),
            [&](tbb::blocked_range<size_t>& range)
            {
                for (size_t i = range.begin(); i < range.end(); i++) {
                    outputGridsArray[i] = mergeOp.merge(keys[i], collationKey);
                }
            }
        );

        // replace primitives from first input, create primitives from second input +

        for (MergeOp::OutputGrids& outputGrids : outputGridsArray) {
            for (MergeOp::OutputGrid& outputGrid : outputGrids) {
                GridBase::Ptr grid = outputGrid.grid;
                if (!grid)  continue;
                GEO_PrimVDB* primitive = outputGrid.primitive;
                if (primitive)  hvdb::replaceVdbPrimitive(*gdp, grid, *primitive);
                else            hvdb::createVdbPrimitive(*gdp, grid);
            }
        }

        // remove old primitives that have now been merged into another

        for (GU_PrimVDB* primitive : mergeOp.vdbPrimsToRemove) {
            if (primitive)  gdp->destroyPrimitive(*primitive, /*andPoints=*/true);
        }

        if (boss.wasInterrupted()) addWarning(SOP_MESSAGE, "processing was interrupted");
        boss.end();

    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }

    return error();
}
