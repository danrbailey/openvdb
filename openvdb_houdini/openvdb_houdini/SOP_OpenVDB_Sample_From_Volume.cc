// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0
//
/// @file SOP_OpenVDB_Sample_From_Volume.cc
///
/// @author Richard Jones, Mary Ferrante
///
/// @brief SOP to sample a volume in positions defined in volume values of another, with anti-aliasing if required.

#include <houdini_utils/ParmFactory.h>
#include <openvdb/math/Math.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/Morphology.h>
#include <openvdb/tools/MultiResGrid.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/tree/NodeManager.h>
#include <openvdb_houdini/Utils.h>
#include <openvdb_houdini/UT_VDBUtils.h>
#include <openvdb_houdini/SOP_NodeVDB.h>
#include <UT/UT_Interrupt.h>
#include <stdexcept>


namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;

enum class SamplerTypes {
    S_POINT,
    S_BOX,
    S_QUADRATIC,
    S_CUBIC
};

enum class MipCalcType {
    MAX = 0,
    AVE
};

class SOP_OpenVDB_Sample_From_Volume: public hvdb::SOP_NodeVDB
{
public:
    SOP_OpenVDB_Sample_From_Volume(OP_Network*, const char* name, OP_Operator*);

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);

    class Cache: public SOP_VDBCacheOptions { OP_ERROR cookVDBSop(OP_Context&) override; };

protected:
    bool updateParmsFlags() override;
};


////////////////////////////////////////


// Build UI and register this operator.
void
newSopOperator(OP_OperatorTable* table)
{
    if (table == nullptr) return;

    hutil::ParmList parms;

    parms.add(hutil::ParmFactory(PRM_STRING, "values", "Values Group")
        .setChoiceList(&hutil::PrimGroupMenuInput1)
        .setTooltip("Specify the group that contains the values to be sampled.")
        .setDocumentation("The group that contains the values to be sampled."));

    parms.add(hutil::ParmFactory(PRM_STRING, "positions", "Positions Group")
        .setChoiceList(&hutil::PrimGroupMenuInput2)
        .setTooltip("Specify the group that contains the positions to sample.")
        .setDocumentation("The group that contains the positions to sample."));

    parms.add(hutil::ParmFactory(PRM_STRING, "outputname", "Output Name")
        .setDefault(PRMzeroDefaults)
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
        "keep",     "Keep Incoming VDB Names",
        "append",   "Append Operation Name",
        "custom",   "Custom Name"
        })
        .setTooltip("Rename output grid(s)")
        .setDocumentation(
            "How to name the generated VDB volumes\n\n"
            "If you choose __Keep Incoming VDB Names__, the generated fields"
            " will replace the input fields."));

    parms.add(hutil::ParmFactory(PRM_STRING, "customname", "Custom Name")
    .setTooltip("Rename all output grids with this custom name")
    .setDocumentation("If this is not blank, the output VDB will use this name."));

    parms.add(hutil::ParmFactory(PRM_ORD, "filter", "Filter")
        .setDefault(PRMoneDefaults)
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "point",     "Nearest",
            "linear",    "Linear",
            "quadratic", "Quadratic",
            "cubic", "Cubic"
        })
        .setDocumentation("\
How to interpolate values at fractional voxel positions\n\
\n\
Nearest:\n\
    Use the value from the nearest voxel.\n\n\
    This is fast but can cause aliasing artifacts.\n\
Linear:\n\
    Interpolate trilinearly between the values of immediate neighbors.\n\n\
    This matches what [Node:sop/volumemix] and [Vex:volumesample] do.\n\
Quadratic:\n\
    Interpolate triquadratically between the values of neighbors.\n\n\
    This produces smoother results than trilinear interpolation but is slower.\n\
Cubic:\n\
    Interpolate tricubically between the values of neighbors.\n\n\
    This produces smoother results than triquadratic interpolation but is slower still.\n"));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "mipmap", "Use Anti-Aliasing")
        .setDefault(PRMoneDefaults)
        .setTooltip("Use mip-mapping to reduce aliasing."));

    parms.add(hutil::ParmFactory(PRM_ORD, "mipfilter", "Mip Map Filter")
        .setDefault(PRMoneDefaults)
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "point",     "Nearest",
            "linear",    "Linear",
            "quadratic", "Quadratic",
            "cubic", "Cubic"
        })
        .setDocumentation("\
How to interpolate values at fractional voxel positions in the mip map stack.\n\
\n\
Nearest:\n\
    Use the value from the nearest voxel.\n\n\
    This is fast but can cause aliasing artifacts.\n\
Linear:\n\
    Interpolate trilinearly between the values of immediate neighbors.\n\n\
    This matches what [Node:sop/volumemix] and [Vex:volumesample] do.\n\
Quadratic:\n\
    Interpolate triquadratically between the values of neighbors.\n\n\
    This produces smoother results than trilinear interpolation but is slower.\n\
Cubic:\n\
    Interpolate tricubically between the values of neighbors.\n\n\
    This produces smoother results than triquadratic interpolation but is slower still.\n"));

    parms.add(hutil::ParmFactory(PRM_INT_J, "miplevels", "Mip Map Levels")
        .setDefault(PRMthreeDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 2, PRM_RANGE_UI, 10)
        .setTooltip("The number of mip-maps of decreasing resolution to generate. Mip level calculation will be clamped to this value."));

    parms.add(hutil::ParmFactory(PRM_ORD, "mipcalc", "Mip Level Calculation")
        .setDefault(PRMzeroDefaults)
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "max",     "Max",
            "average", "Average",
        })
        .setDocumentation("\
How to calculate the mip level from the gradient of the positions to sample.\n\
\n\
Max:\n\
    Use the componentwise maximum.\n\n\
Average:\n\
    Use the average of all components.\n"));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "mipbias", "Bias")
        .setDefault(PRMzeroDefaults)
        .setRange(PRM_RANGE_UI, -4, PRM_RANGE_UI, 4)
        .setTooltip("Amount to bias the mip level calculation in favour of lower levels. Note this does not get baked into the mip level output."));

    parms.add(hutil::ParmFactory(PRM_STRING, "miplevel", "Mip Level Group")
        .setChoiceList(&hutil::PrimGroupMenuInput3)
        .setTooltip("Optional grid to use as the mip level calculation, if no grid is found this will be re-calculated."
                    " This is useful for optimising setups that use the same sample position grids.")
        .setDocumentation("Optional grid to use as the mip level calculation, if no grid is found this will be re-calculated."
                    " This is useful for optimising setups that use the same sample position grids."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "outputmiplevel", "Output Mip Level")
        .setDefault(PRMzeroDefaults)
        .setTooltip("Output the mip level calculation as a float VDB with the same topology as the input position VDB."));

    // Deactivate background value toggle
    parms.add(hutil::ParmFactory(PRM_TOGGLE, "deactivate", "Deactivate Background Voxels")
        .setDefault(PRMoneDefaults)
        .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
        .setDocumentation(
            "Deactivate active output voxels whose values equal"
            " the output VDB's background value."));

    // Deactivation tolerance slider
    parms.add(hutil::ParmFactory(PRM_FLT_J, "deactivatetol", "Deactivate Tolerance")
        .setDefault(PRMzeroDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1)
        .setTooltip(
            "Deactivate active output voxels whose values\n"
            "equal the output VDB's background value.\n"
            "Voxel values are considered equal if they differ\n"
            "by less than the specified tolerance.")
        .setDocumentation(
            "When deactivation of background voxels is enabled,"
            " voxel values are considered equal to the background"
            " if they differ by less than this tolerance."));

    // Prune toggle
    parms.add(hutil::ParmFactory(PRM_TOGGLE, "prune", "Prune")
        .setDefault(PRMoneDefaults)
        .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
        .setDocumentation(
            "Reduce the memory footprint of output VDBs that have"
            " (sufficiently large) regions of voxels with the same value.\n\n"
            "NOTE:\n"
            "    Pruning affects only the memory usage of a VDB.\n"
            "    It does not remove voxels, apart from inactive voxels\n"
            "    whose value is equal to the background."));

    // Pruning tolerance slider
    parms.add(hutil::ParmFactory(PRM_FLT_J, "prunetol", "Prune Tolerance")
        .setDefault(PRMzeroDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1)
        .setTooltip(
            "Collapse regions of constant value in output VDBs.\n"
            "Voxel values are considered equal if they differ\n"
            "by less than the specified tolerance.")
        .setDocumentation(
            "When pruning is enabled, voxel values are considered equal"
            " if they differ by less than the specified tolerance."));

    hvdb::OpenVDBOpFactory("VDB Sample From Volume", SOP_OpenVDB_Sample_From_Volume::factory, parms, *table)
        .setNativeName("")
        .addInput("Values to Sample")
        .addInput("Positions to Sample")
        .addOptionalInput("Optional Mip Level VDB")
        .setVerb(SOP_NodeVerb::COOK_INPLACE, []() { return new SOP_OpenVDB_Sample_From_Volume::Cache; })
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Sample values from VDB volumes with anti-aliasing.\"\"\"\n\
\n\
@overview\n\
\n\
Suppling a VDB grid of positions to sample, values are interpolated from the input VDB volumes \n\
 when upsampling (i.e. positions gradient is less than zero) and use anti-aliased interpolation \n\
 when downsampling (i.e. positions gradient greater than zero).\n\
This maintains a smooth result for most sampling operations.\n\
\n\
@related\n\
\n\
\n\
@examples\n\
\n\
See [openvdb.org|http://www.openvdb.org/download/] for source code\n\
and usage examples.\n");
}


////////////////////////////////////////


OP_Node*
SOP_OpenVDB_Sample_From_Volume::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Sample_From_Volume(net, name, op);
}


SOP_OpenVDB_Sample_From_Volume::SOP_OpenVDB_Sample_From_Volume(OP_Network* net,
    const char* name, OP_Operator* op):
    hvdb::SOP_NodeVDB(net, name, op)
{
}

bool
SOP_OpenVDB_Sample_From_Volume::updateParmsFlags()
{
    bool changed = false;
    const fpreal time = CHgetEvalTime();

    const bool useCustomName = (evalStdString("outputname", 0) == "custom");
    const bool useMipMaps = static_cast<bool>(evalInt("mipmap", 0, time));
    const bool outputMipLevel = static_cast<bool>(evalInt("outputmiplevel", 0, time));
    changed |= enableParm("customname", useCustomName);
    changed |= setVisibleState("customname", useCustomName);

    changed |= enableParm("mipfilter", useMipMaps);
    changed |= enableParm("miplevels", useMipMaps);
    changed |= enableParm("mipcalc", useMipMaps);
    changed |= enableParm("mipbias", useMipMaps);
    changed |= enableParm("miplevel", useMipMaps && !outputMipLevel);
    changed |= enableParm("outputmiplevel", useMipMaps);

    changed |= enableParm("deactivatetol", evalInt("deactivate", 0, time));
    changed |= enableParm("prunetol", evalInt("prune", 0, time));

    return changed;
}

////////////////////////////////////////
namespace {

// @todo: allow construction from individual grids (that are from a multires)
template<typename MultiResGridT>
class MultiResGridSampler
{
public:
    using ValueType = typename MultiResGridT::ValueType;
    using TreeType = typename MultiResGridT::TreeType;
    using AccessorType = typename openvdb::tree::ValueAccessor<const TreeType>;

    MultiResGridSampler(const MultiResGridT& grid) : mMultiResGrid(grid)
    {
        const int numGrids = mMultiResGrid.numLevels();
        mAccessors.reserve(numGrids);
        mGridScales.reserve(numGrids);
        for (int i = 0; i < numGrids; ++i) {
            mAccessors.emplace_back(AccessorType(mMultiResGrid.constTree(i)));
            mGridScales.emplace_back(1.0 / static_cast<double>(1 << i));
        }
    }

    // note: this is in the index space of the top level grid
    template<openvdb::Index Order>
    typename TreeType::ValueType sampleTop(const openvdb::Vec3R& xyz) const
    {
        return openvdb::tools::Sampler<Order>::sample(mAccessors.front(), xyz);
    }

    // note: this is in the index space of the top level grid
    template<openvdb::Index Order>
    typename TreeType::ValueType sampleValue(const openvdb::Vec3R& xyz, double level) const
    {
        const size_t level0 = size_t(floor(level)), level1 = size_t(ceil(level));
        const ValueType v0 = openvdb::tools::Sampler<Order>::sample(mAccessors[level0], xyz * mGridScales[level0]);
        if (level0 == level1) return v0;
        assert(level1 - level0 == 1);
        const ValueType v1 = openvdb::tools::Sampler<Order>::sample(mAccessors[level1], xyz * mGridScales[level1]);
        OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
        const ValueType a = static_cast<ValueType>(level1 - level);
        OPENVDB_NO_TYPE_CONVERSION_WARNING_END
        return a * v0 + (static_cast<ValueType>(1) - a) * v1;
    }

private:

    const MultiResGridT&         mMultiResGrid;
    std::vector<AccessorType>    mAccessors;
    std::vector<double>          mGridScales;
};


template<typename ValuesTreeT, typename PositionTreeT, int SamplerOrder>
struct SampleValueAtPositionOp
{
    SampleValueAtPositionOp(const openvdb::Grid<ValuesTreeT>& values, const PositionTreeT& positionTree)
        : mValuesAccessor(values.tree())
        , mPositionAccessor(positionTree)
        , mValueTransform(values.transform()) {}

    SampleValueAtPositionOp(const SampleValueAtPositionOp& other)
        : mValuesAccessor(other.mValuesAccessor.tree())
        , mPositionAccessor(other.mPositionAccessor.tree())
        , mValueTransform(other.mValueTransform) {}

    template <typename NodeT>
    void operator()(NodeT& node) const
    {
        typename openvdb::tools::GridSampler<openvdb::tree::ValueAccessor<const ValuesTreeT>,
            typename openvdb::tools::Sampler<SamplerOrder>> valueSampler(mValuesAccessor, mValueTransform);
        for (auto iter = node.beginValueOn(); iter; ++iter) {
            const openvdb::Vec3d pos = mPositionAccessor.getValue(iter.getCoord());
            iter.setValue(valueSampler.wsSample(pos));
        }
    }

    const openvdb::tree::ValueAccessor<const ValuesTreeT> mValuesAccessor;
    const openvdb::tree::ValueAccessor<const PositionTreeT> mPositionAccessor;
    const openvdb::math::Transform& mValueTransform;
};


template<typename ValuesTreeT, typename PositionGridT, int SamplerOrder, int MipSamplerOrder>
struct SampleValueMipMapOp
{
    using MultiResGridT = typename openvdb::tools::MultiResGrid<ValuesTreeT>;

    SampleValueMipMapOp(const MultiResGridT& valueGrid, const PositionGridT& positionGrid, const openvdb::FloatGrid& mipLevelGrid, const float bias)
        : mValueGrid(valueGrid)
        , mPositionAccessor(positionGrid.tree())
        , mMipAccessor(mipLevelGrid.tree())
        , mValueTransform(valueGrid.transform())
        , mPositionTransform(positionGrid.transform())
        , mMipTransform(mipLevelGrid.transform())
        , mBias(bias)
        , mMaxMipLevel(valueGrid.numLevels() - 1) {}

    SampleValueMipMapOp(const SampleValueMipMapOp& other)
        : mValueGrid(other.mValueGrid)
        , mPositionAccessor(other.mPositionAccessor.tree())
        , mMipAccessor(other.mMipAccessor.tree())
        , mValueTransform(other.mValueTransform)
        , mPositionTransform(other.mPositionTransform)
        , mMipTransform(other.mMipTransform)
        , mBias(other.mBias)
        , mMaxMipLevel(other.mMaxMipLevel) {}

    template<typename NodeT>
    void operator()(NodeT& node) const
    {
        MultiResGridSampler<MultiResGridT> multiResGridSampler(mValueGrid);
        openvdb::tools::DualGridSampler<openvdb::tree::ValueAccessor<const openvdb::FloatTree>,
            openvdb::tools::BoxSampler> mipSampler(mMipAccessor, mMipTransform, mPositionTransform); //to allow mip and position differing transforms
        for (auto iter = node.beginValueOn(); iter; ++iter) {
            const openvdb::Coord ijk = iter.getCoord();
            const openvdb::Vec3R pos = mPositionAccessor.getValue(ijk);
            const openvdb::Vec3d& valueISPos = mValueTransform.worldToIndex(pos);
            // calculate the mip scale based on the ratio of resolutions between the source transform and the target grid
            // log_2 (source / target) for linear transforms (this will be added to the mip level, because log(ab) = log(a) + log(b))
            const float offset = (1.0 / 3.0) * std::log2(mValueTransform.voxelSize(valueISPos).product() / mPositionTransform.voxelSize(ijk.asVec3d()).product());
            const float mipLevel = mipSampler(ijk) - offset - mBias; // this used to account for transform differences (and biasing)
            if (mipLevel > 0.0f) {
                iter.setValue(multiResGridSampler.template sampleValue<MipSamplerOrder>(valueISPos, openvdb::math::Min(mipLevel, mMaxMipLevel)));
            }
            else iter.setValue(multiResGridSampler.template sampleTop<SamplerOrder>(valueISPos));
        }
    }
    const MultiResGridT& mValueGrid;
    const openvdb::tree::ValueAccessor<const typename PositionGridT::TreeType> mPositionAccessor;
    const openvdb::tree::ValueAccessor<const openvdb::FloatTree> mMipAccessor;
    const openvdb::math::Transform& mValueTransform;
    const openvdb::math::Transform& mPositionTransform;
    const openvdb::math::Transform& mMipTransform;
    const float mBias;
    const float mMaxMipLevel;
};

template<typename PositionGridT, int SamplerOrder, int MipSamplerOrder>
struct SampleVolumeOp
{
    SampleVolumeOp(openvdb::GridBase::Ptr outGrid,
        const PositionGridT& positions,
        openvdb::FloatGrid::ConstPtr mipLevel = nullptr,
        const int mipLevels = 0,
        const float bias = 0.0f)
        : mOutGrid(outGrid)
        , mPositions(positions)
        , mMipLevel(mipLevel)
        , mMipLevels(mipLevels)
        , mBias(bias) {}

    template<typename GridT>
    void operator()(const GridT& values) {
        using TreeT = typename GridT::TreeType;
        typename GridT::Ptr outGrid = openvdb::gridPtrCast<GridT>(mOutGrid);
        outGrid->setTransform(mPositions.transform().copy());
        outGrid->topologyUnion(mPositions);
        openvdb::tree::NodeManager<TreeT> manager(outGrid->tree());

        if (mMipLevel) {
            openvdb::tools::MultiResGrid<TreeT> valuesMultiRes(mMipLevels, values);
            SampleValueMipMapOp<TreeT, PositionGridT, SamplerOrder, MipSamplerOrder>
                sampleOp(valuesMultiRes, mPositions, *mMipLevel, mBias);
            manager.foreachBottomUp(sampleOp);
        }
        else {
            SampleValueAtPositionOp<TreeT, typename PositionGridT::TreeType, SamplerOrder> sampleOp(values, mPositions.tree());
            manager.foreachBottomUp(sampleOp);
        }
    }

private:
    openvdb::GridBase::Ptr mOutGrid;
    const PositionGridT& mPositions;
    openvdb::FloatGrid::ConstPtr mMipLevel;
    const int mMipLevels;
    const float mBias;
};

template<typename PositionGridT, int SamplerOrder, int MipSamplerOrder>
openvdb::GridBase::Ptr doSample(const openvdb::GridBase& values, const PositionGridT& positions,
openvdb::FloatGrid::ConstPtr mipLevel = nullptr, const int mipLevels = 0, const float bias = 0.0f)
{
    openvdb::GridBase::Ptr outGrid = values.copyGridWithNewTree();
    SampleVolumeOp<PositionGridT, SamplerOrder, MipSamplerOrder> op(outGrid, positions, mipLevel, mipLevels, bias);

    using SampleableGridTypes = openvdb_houdini::NumericGridTypes::Append<openvdb_houdini::Vec3GridTypes>;
    values.apply<SampleableGridTypes>(op);
    return outGrid;
}

template<typename PositionGridT, int SamplerOrder>
openvdb::GridBase::Ptr doSample(const openvdb::GridBase& values, const PositionGridT& positions,
openvdb::FloatGrid::ConstPtr mipLevel = nullptr, const SamplerTypes mipSampleType = SamplerTypes::S_BOX, const int mipLevels = 0, const float bias = 0.0f)
{
    if (mipSampleType == SamplerTypes::S_POINT) return doSample<PositionGridT, SamplerOrder, 0>(values, positions, mipLevel, mipLevels, bias);
    else if (mipSampleType == SamplerTypes::S_BOX) return doSample<PositionGridT, SamplerOrder, 1>(values, positions, mipLevel, mipLevels, bias);
    else if (mipSampleType == SamplerTypes::S_QUADRATIC) return doSample<PositionGridT, SamplerOrder, 2>(values, positions, mipLevel, mipLevels, bias);
    else if (mipSampleType == SamplerTypes::S_CUBIC) return doSample<PositionGridT, SamplerOrder, 3>(values, positions, mipLevel, mipLevels, bias);
    else throw std::runtime_error("Unsupported mip sampler type.");
}

template<typename PositionGridT>
openvdb::GridBase::Ptr doSample(const openvdb::GridBase& values, const openvdb::GridBase& positions, const SamplerTypes samplerType,
openvdb::FloatGrid::ConstPtr mipLevel = nullptr, const SamplerTypes mipSampleType = SamplerTypes::S_BOX, const int mipLevels = 0, const float bias = 0.0f)
{
    const PositionGridT& posGrid = static_cast<const PositionGridT&>(positions);
    if (samplerType == SamplerTypes::S_POINT) return doSample<PositionGridT, 0>(values, posGrid, mipLevel, mipSampleType, mipLevels, bias);
    else if (samplerType == SamplerTypes::S_BOX) return doSample<PositionGridT, 1>(values, posGrid, mipLevel, mipSampleType, mipLevels, bias);
    else if (samplerType == SamplerTypes::S_QUADRATIC) return doSample<PositionGridT, 2>(values, posGrid, mipLevel, mipSampleType, mipLevels, bias);
    else if (samplerType == SamplerTypes::S_CUBIC) return doSample<PositionGridT, 3>(values, posGrid, mipLevel, mipSampleType, mipLevels, bias);
    else throw std::runtime_error("Unsupported sampler type.");
}

inline openvdb::GridBase::Ptr doSample(const openvdb::GridBase& values, const openvdb::GridBase& positions, const SamplerTypes sampleType,
openvdb::FloatGrid::ConstPtr mipLevel = nullptr, const SamplerTypes mipSampleType = SamplerTypes::S_BOX, const int mipLevels = 0, const float bias = 0.0f)
{
    if (positions.valueType() == openvdb::typeNameAsString<openvdb::Vec3f>()) {
        return doSample<openvdb::Vec3fGrid>(values, positions, sampleType, mipLevel, mipSampleType, mipLevels, bias);
    }
    else if (positions.valueType() == openvdb::typeNameAsString<openvdb::Vec3d>()) {
        return doSample<openvdb::Vec3dGrid>(values, positions, sampleType, mipLevel, mipSampleType, mipLevels, bias);
    }
    else throw std::runtime_error("Unsupported positions grid type for sample from volume.");
}

// Different Mip Level Calculations
// Standard practice e.g. OpenGL, use a max componentwise calculation but for non-standard grids e.g. frustum grids it may 
// be preferable to use an average of all the components or some other method. This is configurable with the MipT template.

struct MaxMip {
    template <typename ValueT>
    static inline ValueT get(
        const openvdb::math::Vec3<ValueT>& dPdx, 
        const openvdb::math::Vec3<ValueT>& dPdy, 
        const openvdb::math::Vec3<ValueT>& dPdz)
    {
        // equivalent to log2(max(length(dx,dy,dz))) but avoids sqrt
        return  0.5f * std::log2(openvdb::math::Max(dPdx.lengthSqr(), dPdy.lengthSqr(), dPdz.lengthSqr()));
    }
};

struct AverageMip {
    template <typename ValueT>
    static inline ValueT get(
        const openvdb::math::Vec3<ValueT>& dPdx,
        const openvdb::math::Vec3<ValueT>& dPdy,
        const openvdb::math::Vec3<ValueT>& dPdz)
    {
        return  std::log2((dPdx.length() + dPdy.length() + dPdz.length()) / 3.0);
    }
};

//This stores the mip map level calculation in a grid for reuse
//The edge voxels may have incorrect values so will be eroded/reset to background
//@todo: consider reverting to FD_1ST and BD_1ST on voxels bordering inactive regions
//@todo: get typed map to avoid virtual function call
template<typename PositionGridT, typename MipCalcT>
struct MipLevelOp
{
    using DiffType = openvdb::math::D1Vec<openvdb::math::CD_2ND>;
    using PositionT = typename PositionGridT::ValueType;
    using ElementType = typename openvdb::VecTraits<PositionT>::ElementType;

    MipLevelOp(const PositionGridT& posGrid)
        : mStencil(posGrid)
        , mMap(posGrid.transform().baseMap().get()) {}

    template <typename LeafT>
    void operator()(LeafT& leaf, size_t) const
    {
        for (auto iter = leaf.beginValueOn(); iter; ++iter) {
            const openvdb::Coord ijk = iter.getCoord();
            // move the stencil to the relevant position, calculate the partial derivatives
            mStencil.moveTo(ijk);
            PositionT dPdx(DiffType::inX(mStencil, 0), DiffType::inX(mStencil, 1), DiffType::inX(mStencil, 2));
            PositionT dPdy(DiffType::inY(mStencil, 0), DiffType::inY(mStencil, 1), DiffType::inY(mStencil, 2));
            PositionT dPdz(DiffType::inZ(mStencil, 0), DiffType::inZ(mStencil, 1), DiffType::inZ(mStencil, 2));
            // convert to world space
            assert(mMap);
            const openvdb::Vec3d& pos = ijk.asVec3d();
            dPdx = mMap->applyIJT(dPdx, pos);
            dPdy = mMap->applyIJT(dPdy, pos);
            dPdz = mMap->applyIJT(dPdz, pos);

            const float mipLevel = MipCalcT::get(dPdx, dPdy, dPdz);
            // calculate mip level to sample
            iter.setValue(mipLevel);
        }
    }

    mutable openvdb::math::SevenPointStencil<PositionGridT> mStencil;
    const openvdb::math::MapBase* mMap;
};

template<typename PositionGridT, typename MipCalcT>
openvdb::FloatGrid::Ptr calculateMipLevelTyped(const PositionGridT& positions) 
{
    openvdb::FloatGrid::Ptr newGrid = openvdb::FloatGrid::create();
    newGrid->setTransform(positions.transform().copy());
    newGrid->topologyUnion(positions);
    newGrid->tree().voxelizeActiveTiles();

    MipLevelOp<PositionGridT, MipCalcT> mipLevelOp(positions);
    openvdb::FloatTree& tree = newGrid->tree();
    openvdb::tree::LeafManager<openvdb::FloatTree> manager(tree);
    manager.foreach(mipLevelOp);
    // erode edges by 1 voxel avoid incorrect gradient calcs
    // @todo: revert to FD and BD in gradient alculation when neighbour inactive
    openvdb::tools::erodeActiveValues(tree, 1, openvdb::tools::NearestNeighbors::NN_FACE);
    openvdb::tools::pruneInactive(tree);
    manager.rebuild();
    // need to set all these newly inactive values to 0.0 in case they are sampled later
    manager.foreach(
        [](openvdb::FloatTree::LeafNodeType& leaf, size_t) {
            for (auto iter = leaf.beginValueOff(); iter; ++iter) iter.setValue(0.0f);
        }
    );
    return newGrid;
}

template <typename PositionGridT>
inline openvdb::FloatGrid::Ptr calculateMipLevelTyped(const openvdb::GridBase& positions, const MipCalcType mipLevelType)
{
    const PositionGridT& positionGrid = UTvdbGridCast<const PositionGridT>(positions);
    if (mipLevelType == MipCalcType::MAX) return calculateMipLevelTyped<PositionGridT, MaxMip>(positionGrid);
    else if (mipLevelType == MipCalcType::AVE) return calculateMipLevelTyped<PositionGridT, AverageMip>(positionGrid);
    else throw std::runtime_error("Unsupported mip level calculation type.");
}

inline openvdb::FloatGrid::Ptr calculateMipLevel(const openvdb::GridBase& positions, const MipCalcType mipCalcType)
{
    if (positions.valueType() == openvdb::typeNameAsString<openvdb::Vec3f>()) return calculateMipLevelTyped<openvdb::Vec3fGrid>(positions, mipCalcType);
    else if (positions.valueType() == openvdb::typeNameAsString<openvdb::Vec3d>()) return calculateMipLevelTyped<openvdb::Vec3dGrid>(positions, mipCalcType);
    else throw std::runtime_error("Unsupported positions grid type for calculating mip level.");
}


template <typename GridType>
inline void doPrune(GridType& grid, double tolerance)
{
    typedef typename GridType::ValueType ValueT;

    openvdb::tools::pruneInactive(grid.tree());
    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
    const auto value = openvdb::zeroVal<ValueT>() + tolerance;
    OPENVDB_NO_TYPE_CONVERSION_WARNING_END
    openvdb::tools::prune(grid.tree(), static_cast<ValueT>(value));
}


template <typename GridType>
inline void doDeactivate(GridType& grid, double tolerance)
{
    using ValueT = typename GridType::ValueType;

    OPENVDB_NO_TYPE_CONVERSION_WARNING_BEGIN
    const auto value = openvdb::zeroVal<ValueT>() + tolerance;
    OPENVDB_NO_TYPE_CONVERSION_WARNING_END
    openvdb::tools::deactivate(grid.tree(), grid.background(), static_cast<ValueT>(value));
}

}

OP_ERROR
SOP_OpenVDB_Sample_From_Volume::Cache::cookVDBSop(OP_Context& context)
{
    try {
        const fpreal time = context.getTime();

        // Construct a functor to process grids of arbitrary type.
        UT_AutoInterrupt progress("Cooking SOP");

        // Get the group of grids to process.
        const GA_PrimitiveGroup* valuesGroup = this->matchGroup(*gdp, evalStdString("values", time));

        // Get the first positions grid to use as positions to sample
        const GU_Detail* secondInput = inputGeo(1);
        hvdb::VdbPrimCIterator posIt{secondInput, matchGroup(*secondInput, evalStdString("positions", time))};

        if (!posIt) {
            addError(SOP_MESSAGE, "Missing positions to sample grid");
            return error();
        }
        const openvdb::GridBase& positions = posIt->getConstGrid();

        const SamplerTypes samplerType = static_cast<SamplerTypes>(evalInt("filter", 0, time));
        const bool useMipMaps = static_cast<bool>(evalInt("mipmap", 0, time));
        const int mipLevels = useMipMaps ? evalInt("miplevels", 0, time) : 0;
        const float mipBias = useMipMaps ? evalFloat("mipbias", 0, time) : 0;
        const bool outputMip  = useMipMaps ? static_cast<bool>(evalInt("outputmiplevel", 0, time)) : false;
        const SamplerTypes mipSampleType  = useMipMaps ?
            static_cast<SamplerTypes>(evalInt("mipfilter", 0, time)) : SamplerTypes::S_POINT;
        const MipCalcType mipCalcType  = useMipMaps ?
            static_cast<MipCalcType>(evalInt("mipcalc", 0, time)) : MipCalcType::MAX;
        const float deactivate = static_cast<bool>(evalInt("deactivate", 0, time));
        const float prune = static_cast<bool>(evalInt("prune", 0, time));
        const float deactivateTol = deactivate ? evalFloat("deactivatetol", 0, time) : 0.0f;
        const float pruneTol = prune ? evalFloat("prunetol", 0, time) : 0.0f;

        openvdb::FloatGrid::Ptr newMipLevel;
        openvdb::FloatGrid::ConstPtr mipLevel;
        if (useMipMaps) {

            // if mip level has been input, use it, otherwise calculate it
            const GU_Detail* thirdInput = inputGeo(2);

            if (thirdInput) {
                hvdb::VdbPrimCIterator mipLevelIt{thirdInput, matchGroup(*thirdInput, evalStdString("miplevel", time))};
                if (!mipLevelIt || mipLevelIt->getStorageType() != UT_VDB_FLOAT) {
                    addError(SOP_MESSAGE, "No mip level float valued VDB found on third input");
                    return error();
                }
                mipLevel = UTvdbGridCast<openvdb::FloatGrid>(mipLevelIt->getGridPtr());
                if (outputMip) addWarning(SOP_MESSAGE, "Mip level VDB on third input, Output Mip Level will be ignored.");
            }
            else {
                newMipLevel = calculateMipLevel(positions, mipCalcType);
                if (deactivate) UTvdbCallAllType(UT_VDB_FLOAT, doDeactivate, *newMipLevel, deactivateTol);
                if (prune) UTvdbCallAllType(UT_VDB_FLOAT, doPrune, *newMipLevel, pruneTol);
                mipLevel = newMipLevel;
            }
        }
        // Process each VDB primitive that belongs to the values group.
        for (hvdb::VdbPrimIterator it(gdp, valuesGroup); it; ++it) {
            if (progress.wasInterrupted()) throw std::runtime_error("processing was interrupted");
            const openvdb::GridBase& values = it->getConstGrid();
            openvdb::GridBase::Ptr newGrid = doSample(values, positions, samplerType,
                mipLevel, mipSampleType, mipLevels, mipBias);
            assert(newGrid);
            // Rename grid
            std::string gridName = (*it)->getGridName();
            const auto renaming = evalStdString("outputname", time);
            if (renaming == "append") {
                gridName += "_sampled";
            } else if (renaming == "custom") {
                const auto customName = evalStdString("customname", time);
                if (!customName.empty()) gridName = customName;
            }
            if (deactivate) UTvdbCallAllType(UTvdbGetGridType(*newGrid), doDeactivate, *newGrid, deactivateTol);
            if (prune) UTvdbCallAllType(UTvdbGetGridType(*newGrid), doPrune, *newGrid, pruneTol);
            hvdb::replaceVdbPrimitive(*gdp, newGrid, **it, true, gridName.c_str());

            if (newMipLevel && outputMip) {
                hvdb::createVdbPrimitive(*gdp, newMipLevel, "miplevel");
            }
        }
    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }
    return error();
}
