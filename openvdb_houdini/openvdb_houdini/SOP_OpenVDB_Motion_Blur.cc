// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0
//
/// @file SOP_OpenVDB_Motion_Blur.cc
///
/// @author Richard Jones

/// @brief This file contains the implementation of the SOP_OpenVDB_Motion_Blur and SOP_OpenVDB_Motion_Blur_Camera,
/// which provide functionality for blurring VDB grids along streamlines defined by velocity and acceleration fields.
/// The blurring can be controlled by various parameters, including the use of ramps for weighting and the inclusion of camera motion.
/// There are two versions of the same base SOP: a compilable SOP without camera motion and a non-compilable SOP with camera motion.
///
#include <houdini_utils/ParmFactory.h>
#include <openvdb_houdini/Utils.h>
#include <openvdb_houdini/SOP_NodeVDB.h>

#include <openvdb/tools/MotionBlur.h>
#include <openvdb/tools/LevelSetRebuild.h>
#include <openvdb/util/NullInterrupter.h>

#include <UT/UT_Ramp.h>
#include <UT/UT_Interrupt.h>
#include <GA/GA_PageIterator.h>
#include <GU/GU_PrimPoly.h>
#include <CH/CH_Manager.h>
#include <PRM/PRM_Parm.h>

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <variant>

namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;

namespace {

    namespace blur = openvdb::tools::blur;
    using CompType = blur::CompositionOperator;
    using SamplerType = blur::Rand01Subsampler;
    using BlurParms = blur::BlurParms<SamplerType>;

    //Taken from SOP_OpenVDB_Rasterize_Frustum
    // Returns a null pointer if geoPt is null or if no reference vdb is found.
    inline openvdb::math::Transform::Ptr
    getReferenceTransform(const GU_Detail* geoPt, const GA_PrimitiveGroup* group = nullptr,
        UT_ErrorManager* log = nullptr)
    {
        if (geoPt) {
            hvdb::VdbPrimCIterator vdbIt(geoPt, group);
            if (vdbIt) {
                return (*vdbIt)->getGrid().transform().copy();
            } else if (log) {
                log->addWarning(SOP_OPTYPE_NAME, SOP_MESSAGE, "Could not find a reference VDB grid");
            }
        }

        return {};
    }

    ///
    /// @brief WeightT for UT_Ramp valued weights to use with blurAlongStreamlines
    ///
    template <bool Average>
    struct RampWeightCache
    {
        explicit RampWeightCache(UT_Ramp* ramp)
            : mRamp(ramp)
        {
            assert(mRamp);
        }

        RampWeightCache(const RampWeightCache& other)
            : mRamp(other.mRamp)
            , mWeights(other.mWeights)
        {}

        void clear()
        {
            mWeights.clear();
        }

        void initFromSegments(const std::vector<size_t>& segments)
        {
            size_t numSegments = segments.size();
            if (numSegments == 0) return;

            float segmentWidth = 1.f / static_cast<float>(numSegments);
            size_t totalSamples = std::reduce(segments.begin(), segments.end());
            if (totalSamples == 1) {
                mWeights = {1.f};
                return;
            }
            mWeights.resize(totalSamples);

            float totalWeight = 0.f;
            size_t offset = 0;
            for (size_t segIdx = 0; segIdx < numSegments; ++segIdx) {
                size_t samples = segments[segIdx];
                float localStep = segmentWidth / static_cast<float>(samples - 1);
                for (size_t i = 0; i < samples; ++i) {
                    float pos = segIdx * segmentWidth + i * localStep;
                    float weight = rampAt(pos);
                    mWeights[offset + i] = weight;
                    totalWeight += weight;
                }
                offset += samples;
            }

            if (openvdb::math::isApproxZero(totalWeight)) {
                float fill = Average ? 1.f / totalSamples : 1.f;
                mWeights.assign(totalSamples, fill);
                return;
            }
            if (Average) {
                float invTotal = 1.f / totalWeight;
                for (float& w : mWeights) w *= invTotal;
            }
        }

        float at(size_t idx) const
{
            assert(idx < mWeights.size());
            return mWeights[idx];
        }

        float rampAt(float pos) const
        {
            float val[4];
            mRamp->rampLookup(pos, val);
            return val[0];
        }

        UT_Ramp* const mRamp;
        std::vector<float> mWeights;
    };

    ///
    /// @brief WeightT for UT_Ramp valued weights to use with blurAlongStreamlines,
    /// with precomputed samples for faster evaluation. Linearly interpolates between samples.
    ///
    template <bool Average>
    struct ApproxRampWeightCache : public RampWeightCache<Average>
    {
        ApproxRampWeightCache(UT_Ramp* ramp, int subsamples)
            : RampWeightCache<Average>(ramp)
            , mRampSamples(std::make_shared<std::vector<float>>(subsamples))
        {
            auto& samples = *mRampSamples;
            float step = 1.f / static_cast<float>(subsamples - 1);
            for (int i = 0; i < subsamples; ++i) {
                samples[i] = this->rampAt(i * step);
            }
        }

        ApproxRampWeightCache(const ApproxRampWeightCache& other)
            : RampWeightCache<Average>(other.mRamp)
            , mRampSamples(other.mRampSamples) {}

        void initFromSegments(const std::vector<size_t>& segments) {
            size_t numSegments = segments.size();
            if (numSegments == 0) return;

            float segmentWidth = 1.f / static_cast<float>(numSegments);
            size_t totalSamples = std::reduce(segments.begin(), segments.end());
            if (totalSamples == 1) {
                this->mWeights = {1.f};
                return;
            }
            this->mWeights.resize(totalSamples);

            float totalWeight = 0.f;
            size_t offset = 0;
            for (size_t segIdx = 0; segIdx < numSegments; ++segIdx) {
                size_t samples = segments[segIdx];
                float localStep = segmentWidth / static_cast<float>(samples - 1);
                for (size_t i = 0; i < samples; ++i) {
                    float pos = segIdx * segmentWidth + i * localStep;
                    float weight = interpAt(pos);
                    this->mWeights[offset + i] = weight;
                    totalWeight += weight;
                }
                offset += samples;
            }

            if (openvdb::math::isApproxZero(totalWeight)) {
                float fill = Average ? 1.f / totalSamples : 1.f;
                this->mWeights.assign(totalSamples, fill);
                return;
            }
            if (Average) {
                float invTotal = 1.f / totalWeight;
                for (float& w : this->mWeights) w *= invTotal;
            }
        }

        // Linearly interpolate between ramp samples for position in [0,1]
        float interpAt(float pos) const {
            assert(pos >= 0.f && pos <= 1.f);
            if (pos <= 0.f) return mRampSamples->front();
            if (pos >= 1.f) return mRampSamples->back();

            float idx = pos * (mRampSamples->size() - 1);
            size_t i = static_cast<size_t>(std::floor(idx));
            float frac = idx - i;

            float a = (*mRampSamples)[i];
            float b = (*mRampSamples)[i + 1];
            return a + frac * (b - a);
        }

        std::shared_ptr<std::vector<float>> mRampSamples;
    };

    ///
    /// @brief Returns the blur parameters from the node at the current time,
    /// this doesn't include the camera.
    ///
    template<typename NodeT>
    BlurParms getBlurParms(NodeT* node, fpreal now)
    {
        const float timestepDt = node->evalFloat("shutter", 0, now);

        BlurParms blurParms;

        if (timestepDt != 0.f) {
            const int numTimeSamples = node->evalInt("timesamples", 0, now);
            const float substepDt = timestepDt / (numTimeSamples - 1);
            const float startTime = timestepDt * 0.5 * (node->evalFloat("shutteroffset", 0, now) - 1.0);

            // clear default timesamples
            blurParms.timesamples = std::vector<float>(numTimeSamples);
            // get times for timesamples
            for (int t = 0; t < numTimeSamples; ++t) {
                blurParms.timesamples[t] = startTime + t * substepDt;
            }
            blurParms.maxsteps = node->evalInt("maxsteps", 0, now);
            blurParms.subsampler = openvdb::tools::blur::Rand01Subsampler(node->evalInt("subsamples", 0, now),
                                        node->evalInt("subsampleseed", 0, now));
        }
        return blurParms;
    }

    ///
    /// @brief Run blur operation on the input geometry
    ///
    template <typename NodeT>
    OP_ERROR doBlur(NodeT* node, GU_Detail* gdp, const GU_Detail* velGeo,
        const BlurParms& blurParms, fpreal now) {

        if (blurParms.timesamples.empty()) return node->error();

        hvdb::HoudiniInterrupter boss("Performing motion blur");

        const GA_PrimitiveGroup* vdbGroup = node->matchGroup(*gdp, node->evalStdString("group", now));

        const openvdb::Vec3fGrid* vel = nullptr;
        const openvdb::Vec3fGrid* accel = nullptr;

        const bool useAccel = static_cast<bool>(node->evalInt("enableaccel", 0, now));
        UT_String velPrimNameIdx("");
        UT_String accelPrimNameIdx("");
        GA_Index velIdx, accelIdx;

        std::string velGroupStr = node->evalStdString("velgroup", now);
        std::string accelGroupStr = node->evalStdString("accelgroup", now);
        if (velGroupStr.empty() && useAccel && accelGroupStr.empty()) {
            node->addWarning(SOP_MESSAGE, "No vel or acceleration group specified, will assume velocity first grid, acceleration second");
        }
        hvdb::VdbPrimCIterator vecIt{velGeo, node->matchGroup(*velGeo, velGroupStr)};
        if (vecIt) {
            if (vecIt->getStorageType() != UT_VDB_VEC3F) {
                node->addError(SOP_MESSAGE, "Unrecognised velocity grid type, expecting 32-bit vector float");
                return node->error();
            }
            vel = openvdb::gridConstPtrCast<openvdb::Vec3fGrid>(vecIt->getConstGridPtr()).get();
            velPrimNameIdx = vecIt.getPrimitiveIndexAndName();
            velIdx = vecIt.getIndex();
        }

        if (useAccel) {
            hvdb::VdbPrimCIterator accelIt{velGeo, node->matchGroup(*velGeo, accelGroupStr)};
            if (accelIt) {
                if (accelGroupStr.empty() && accelIt == vecIt) {
                    // if no accel group specified, and vel and accel groups are the same, try next grid
                    ++accelIt;
                }
                if (accelIt) {
                    if (accelIt->getStorageType() != UT_VDB_VEC3F) {
                        node->addError(SOP_MESSAGE, "Unrecognised acceleration grid type, expecting 32-bit vector float");
                        return node->error();
                    }
                    accel = openvdb::gridConstPtrCast<openvdb::Vec3fGrid>(accelIt->getConstGridPtr()).get();
                    accelPrimNameIdx = accelIt.getPrimitiveIndexAndName();
                    accelIdx = accelIt.getIndex();
                }
            }
        }

        if (!vel && !accel && !blurParms.camera.isValid()) {
            node->addWarning(SOP_MESSAGE, "No velocity/acceleration grid, or camera");
            return node->error();
        }

        if (static_cast<bool>(node->evalInt("mode", 0, now))) {
            // STREAK
            const bool rebuild = static_cast<bool>(node->evalInt("rebuild", 0, now));
            const bool combineMode = node->evalInt("combinemode", 0, now);
            blur::UniformWeight<false> weight(1.0);
            for (hvdb::VdbPrimIterator it(gdp, vdbGroup); it; ++it) {
                openvdb::GridBase::ConstPtr grid = it->getConstGridPtr();
                if (!grid) continue;

                // for level set grids, do a min
                const bool levelset = grid->getGridClass() == openvdb::GRID_LEVEL_SET;
                openvdb::GridBase::Ptr streakedGrid;
                if (levelset) {
                    streakedGrid = blur::blurAlongStreamlines<CompType::MIN>(*grid, weight, blurParms, vel, accel, &boss);
                    if (rebuild) {
                        openvdb::FloatGrid::ConstPtr floatGrid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid);
                        if (!floatGrid) {
                            node->addError(SOP_MESSAGE, "Unsupported levelset grid type");
                            return node->error();
                        }
                        float narrowband = floatGrid->background() / floatGrid->transform().voxelSize().x();
                        openvdb::FloatGrid::Ptr streakedFloatGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(streakedGrid);
                        streakedGrid->setTree(openvdb::tools::levelSetRebuild(*streakedFloatGrid, 0, narrowband, narrowband)->treePtr());
                        streakedGrid->setGridClass(openvdb::GRID_LEVEL_SET);
                    }
                }
                else {
                    if (combineMode == 0) streakedGrid = blur::blurAlongStreamlines<CompType::MAX>(*grid, weight, blurParms, vel, accel, &boss);
                    else streakedGrid = blur::blurAlongStreamlines<CompType::ADD>(*grid, weight, blurParms, vel, accel, &boss);
                }

                hvdb::replaceVdbPrimitive(*gdp, streakedGrid, **it);
                if (boss.wasInterrupted()) break;
            }
        }
        else {
            // BLUR
            if (static_cast<bool>(node->evalInt("shuttershape", 0, now))) {
                // get the ramp value from the interface
                UT_Ramp ramp;
                node->evalRamp(ramp, now);
                if (static_cast<bool>(node->evalInt("subsampleramp", 0, now))) {
                    const int rampSamples = node->evalInt("rampsubsamples", 0, now);
                    ApproxRampWeightCache<true> weights(&ramp, rampSamples);
                    for (hvdb::VdbPrimIterator it(gdp, vdbGroup); it; ++it) {
                        openvdb::GridBase::ConstPtr grid = it->getConstGridPtr();
                        if (!grid) continue;

                        openvdb::GridBase::Ptr blurredGrid = blur::blurAlongStreamlines<CompType::ADD>(*grid, weights, blurParms, vel, accel, &boss);
                        hvdb::replaceVdbPrimitive(*gdp, blurredGrid, **it);
                        if (boss.wasInterrupted()) break;
                    }
                }
                else {
                    RampWeightCache<true>  weights(&ramp);
                    for (hvdb::VdbPrimIterator it(gdp, vdbGroup); it; ++it) {
                        openvdb::GridBase::ConstPtr grid = it->getConstGridPtr();
                        if (!grid) continue;

                        openvdb::GridBase::Ptr blurredGrid = blur::blurAlongStreamlines<CompType::ADD>(*grid, weights, blurParms, vel, accel, &boss);
                        hvdb::replaceVdbPrimitive(*gdp, blurredGrid, **it);
                        if (boss.wasInterrupted()) break;
                    }
                }
            }
            else {
                blur::UniformWeight<true> weight(1.0);
                for (hvdb::VdbPrimIterator it(gdp, vdbGroup); it; ++it) {
                    openvdb::GridBase::ConstPtr grid = it->getConstGridPtr();
                    if (!grid) continue;

                    openvdb::GridBase::Ptr blurredGrid = blur::blurAlongStreamlines<CompType::ADD>(*grid, weight, blurParms, vel, accel, &boss);
                    hvdb::replaceVdbPrimitive(*gdp, blurredGrid, **it);
                    if (boss.wasInterrupted()) break;
                }
            }
        }
        return node->error();
    }

    template<typename NodeT>
    bool updateNodeParmsFlags(NodeT* node) {

        bool changed = false;

        const bool rampFalloff = node->evalInt("shuttershape", 0, 0)==1;
        const bool subsampleRamp = static_cast<bool>(node->evalInt("subsampleramp", 0, 0));
        const bool enableAccel = static_cast<bool>(node->evalInt("enableaccel", 0, 0));

        changed |= node->enableParm("shutterramp", rampFalloff);
        changed |= node->enableParm("subsampleramp", rampFalloff);
        changed |= node->enableParm("rampsubsamples", rampFalloff && subsampleRamp);
        changed |= node->enableParm("accelgroup", enableAccel);

        return changed;
    }

}

class SOP_OpenVDB_Motion_Blur_Camera: public hvdb::SOP_NodeVDB
{
public:
    SOP_OpenVDB_Motion_Blur_Camera(OP_Network*, const char* name, OP_Operator*);
    ~SOP_OpenVDB_Motion_Blur_Camera() override {}

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);
    bool updateParmsFlags() override;

    OP_ERROR cookVDBSop(OP_Context&) override;
    void evalRamp(UT_Ramp& ramp, fpreal now);
};

// compilable version of the SOP w/o camera input
class SOP_OpenVDB_Motion_Blur: public hvdb::SOP_NodeVDB
{
public:
    SOP_OpenVDB_Motion_Blur(OP_Network*, const char* name, OP_Operator*);
    ~SOP_OpenVDB_Motion_Blur() override {}

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);
    bool updateParmsFlags() override;

    class Cache: public SOP_VDBCacheOptions {
        OP_ERROR cookVDBSop(OP_Context&) override;
    public:
        void evalRamp(UT_Ramp& ramp, fpreal now);
    };
};


////////////////////////////////////////


// Build UI and register these operators

void
newSopOperator(OP_OperatorTable* table)
{
    if (table == nullptr) return;

    auto getParmList = [](bool withCamera = false) -> hutil::ParmList {
        hutil::ParmList parmlist;

        parmlist.add(hutil::ParmFactory(PRM_STRING, "group", "Group")
            .setChoiceList(&hutil::PrimGroupMenuInput1)
            .setTooltip("VDB grid(s) to blur.")
            .setDocumentation(
                "A subset of VDBs in the first input to blur using the velocity field"
                " (see [specifying volumes|/model/volumes#group])"));

        if (withCamera) {
            parmlist.add(hutil::ParmFactory(PRM_STRING, "cameragroup", "Camera Frustum VDB")
                .setChoiceList(&hutil::PrimGroupMenuInput3)
                .setTooltip("Camera frustum grids")
                .setDocumentation(
                    "The camera frustum to use to define the camera output."
                    " Will be sampled to compute camera motion."));
        }

        parmlist.add(hutil::ParmFactory(PRM_SEPARATOR,"sepaccel", ""));

        // Velocity
        parmlist.add(hutil::ParmFactory(PRM_STRING, "velgroup", "Velocity VDB")
            .setChoiceList(&hutil::PrimGroupMenuInput2)
            .setTooltip("Velocity grid")
            .setDocumentation(
                "The name of a VDB primitive in the second input to use as"
                " the velocity field (see [specifying volumes|/model/volumes#group])\n\n"
                "This must be a vector-valued VDB primitive."
                " You can use the [Vector Merge node|Node:sop/DW_OpenVDBVectorMerge]"
                " to turn a `vel.[xyz]` triple into a single primitive."));


        parmlist.add(hutil::ParmFactory(PRM_TOGGLE, "enableaccel", "Enable Acceleration")
            .setDefault(PRMzeroDefaults)
            .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
            .setTooltip("Enable acceleration"));

        parmlist.add(hutil::ParmFactory(PRM_STRING, "accelgroup", "Acceleration VDB")
            .setChoiceList(&hutil::PrimGroupMenuInput2)
            .setTooltip("Acceleration grid")
            .setDocumentation(
                "The name of a VDB primitive in the second input to use as"
                " the acceleration of the velocity field (see [specifying volumes|/model/volumes#group])\n\n"
                "This must be a vector-valued VDB primitive."
                " You can use the [Vector Merge node|Node:sop/DW_OpenVDBVectorMerge]"
                " to turn a `vel.[xyz]` triple into a single primitive."));

        parmlist.add(hutil::ParmFactory(PRM_SEPARATOR,"sepsettings", ""));

        parmlist.add(hutil::ParmFactory(PRM_FLT_J, "shutter", "Shutter")
            .setDefault("1/$FPS")
            .setRange(PRM_RANGE_UI, 0, PRM_RANGE_UI, 3)
            .setTooltip("Scale the velocity blur length. This can also be seen as the shutter time."));

        parmlist.add(hutil::ParmFactory(PRM_FLT_J, "shutteroffset", "Shutter Offset")
            .setDefault(PRMzeroDefaults)
            .setRange(PRM_RANGE_UI, -1, PRM_RANGE_UI, 1)
            .setTooltip("Offset the blur centre. A value of 0 is centred around each input value,"
            " -1 will offset the blur to end at the original value, 1 will start at its centre."));

        parmlist.add(hutil::ParmFactory(PRM_INT_J, "timesamples", "Time Samples")
            .setDefault(PRMtwoDefaults)
            .setRange(PRM_RANGE_RESTRICTED, 2, PRM_RANGE_UI, 10)
            .setTooltip("Number of time samples to use for defining the blur streamlines. This will be 1 more than the number of blurred segments created."));

        parmlist.add(hutil::ParmFactory(PRM_SEPARATOR,"sepvoxelsettings", ""));

        parmlist.add(hutil::ParmFactory(PRM_INT_J, "subsamples", "Voxel Subsamples")
            .setDefault(PRMoneDefaults)
            .setRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 10)
            .setTooltip("Number of subsamples to use per voxel of the input grid."));

        parmlist.add(hutil::ParmFactory(PRM_INT_J, "subsampleseed", "Seed")
            .setDefault(PRMoneDefaults)
            .setRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10)
            .setTooltip("Seed for the random scatter within the voxel of the subsamples."));

        parmlist.add(hutil::ParmFactory(PRM_INT_LOG, "maxsteps", "Max Steps")
            .setDefault(1000)
            .setRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 1000)
            .setTooltip("Values will be distributed evenly along the streamline, contributing to up to this number of voxels."));

        parmlist.beginExclusiveSwitcher("mode", "Mode");
        parmlist.addFolder("Blur");

        parmlist.add(hutil::ParmFactory(PRM_ORD, "shuttershape", "Shutter Shape")
            .setDefault(PRMzeroDefaults)
            .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
                "constant", "Constant",
                "ramp", "From Ramp",
            })
            .setTooltip("Method to use for calculating the falloff shape along the blur a.k.a the shutter shape."));

        {
            std::map<std::string, std::string> rampSpare;
            rampSpare[PRM_SpareData::getFloatRampDefaultToken()] =
                "1pos ( 0.0 ) 1value ( 0.0 ) 1interp ( monotonecubic ) "
                "2pos ( 0.5 ) 2value ( 1.0 ) 2interp ( monotonecubic ) "
                "3pos ( 1.0 ) 3value ( 0.0 ) 3interp ( monotonecubic )";

            rampSpare[PRM_SpareData::getRampShowControlsDefaultToken()] = "0";

            parmlist.add(hutil::ParmFactory(PRM_MULTITYPE_RAMP_FLT, "shutterramp", "Shutter Ramp")
                .setDefault(PRMthreeDefaults)
                .setSpareData(rampSpare)
                .setTooltip("X Axis: 0 = input range minimum, 1 = input range maximum.\n"
                    "Y Axis: 0 = output range minimum, 1 = output range maximum.\n"));
        }
        parmlist.add(hutil::ParmFactory(PRM_TOGGLE, "subsampleramp", "")
            .setDefault(PRMoneDefaults)
            .setTypeExtended(PRM_TYPE_TOGGLE_JOIN)
            .setTooltip("Uniformly subsample the ramp, ramp values are then linearly interpolated between these samples."));

        parmlist.add(hutil::ParmFactory(PRM_INT_J, "rampsubsamples", "Ramp Subsamples")
            .setDefault(100)
            .setRange(PRM_RANGE_RESTRICTED, 2, PRM_RANGE_UI, 100)
            .setTooltip("Number of subsamples to use to approximate the ramp."));

        parmlist.addFolder("Streak");
        parmlist.add(hutil::ParmFactory(PRM_TOGGLE, "rebuild", "Rebuild Level Sets")
            .setDefault(PRMoneDefaults)
            .setTooltip(
                "Rebuild any levelsets after streaking."));

        parmlist.add(hutil::ParmFactory(PRM_ORD, "combinemode", "Combine Operation")
            .setDefault(PRMzeroDefaults)
            .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
                "max", "Max",
                "add", "Add",
            })
            .setTooltip("Method to use when multiple streaks overlap on the same voxel, for non-levelset grid types."));

        parmlist.endSwitcher();
        return parmlist;
    };

    hutil::ParmList parms = getParmList(/*camera*/true);
    // Register this operator.
    hvdb::OpenVDBOpFactory("VDB Motion Blur Camera",
        SOP_OpenVDB_Motion_Blur_Camera::factory, parms, *table)
        .setNativeName("vdbmotionblurcamera")
        .addInput("VDBs to Blur")
        .addOptionalInput("Optional Velocity/Acceleration VDBs")
        .addOptionalInput("Optional Camera Frustum VDBs")
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Blur a VDB along streamlines from a camera, velocity and/or acceleration values.\"\"\"\n\
\n\
@overview\n\
\n\
This node takes an input VDB and blurs with respect to a camera and/or velocity/acceleration values.\n\
Also allows the extrusion of SDFs along these streamlines.\n\
\n\
@examples\n\
\n\
See [openvdb.org|http://www.openvdb.org/download/] for source code\n\
and usage examples.\n");

    hutil::ParmList velocityParms = getParmList(/*camera*/false);
    // Register this operator.
    hvdb::OpenVDBOpFactory("VDB Motion Blur",
        SOP_OpenVDB_Motion_Blur::factory, velocityParms, *table)
        .setNativeName("vdbmotionblur")
        .addAlias("vdbvelocityblur")
        .addAlias("vdbvectorblur")
        .addInput("VDBs to Blur")
        .addInput("Velocity/Acceleration VDBs")
        .setVerb(SOP_NodeVerb::COOK_INPLACE, []() { return new SOP_OpenVDB_Motion_Blur::Cache; })
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Blur a VDB along streamlines from a vector-valued velocity VDB.\"\"\"\n\
\n\
@overview\n\
\n\
This node takes an input VDB and blurs along the vectors defined by\n\
a secondary vector-valued velocity/acceleration VDBs.\n\
\n\
@examples\n\
\n\
See [openvdb.org|http://www.openvdb.org/download/] for source code\n\
and usage examples.\n");
}

// Enable/disable or show/hide parameters in the UI.
bool
SOP_OpenVDB_Motion_Blur_Camera::updateParmsFlags()
{
    return updateNodeParmsFlags(this);
}

bool
SOP_OpenVDB_Motion_Blur::updateParmsFlags()
{
    return updateNodeParmsFlags(this);
}

////////////////////////////////////////


OP_Node*
SOP_OpenVDB_Motion_Blur_Camera::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Motion_Blur_Camera(net, name, op);
}


SOP_OpenVDB_Motion_Blur_Camera::SOP_OpenVDB_Motion_Blur_Camera(OP_Network* net,
    const char* name, OP_Operator* op):
    hvdb::SOP_NodeVDB(net, name, op)
{
}

void
SOP_OpenVDB_Motion_Blur_Camera::evalRamp(UT_Ramp& ramp, fpreal now) {
    this->updateRampFromMultiParm(now, this->getParm("shutterramp"), ramp);
}

//////////////////////////////////////


OP_Node*
SOP_OpenVDB_Motion_Blur::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Motion_Blur(net, name, op);
}


SOP_OpenVDB_Motion_Blur::SOP_OpenVDB_Motion_Blur(OP_Network* net,
    const char* name, OP_Operator* op):
    hvdb::SOP_NodeVDB(net, name, op)
{
}

void
SOP_OpenVDB_Motion_Blur::Cache::evalRamp(UT_Ramp& ramp, fpreal now)
{
    // get the ramp value from the interface
    const auto rampStr = evalStdString("shutterramp", now);
    UT_IStream strm(rampStr.c_str(), rampStr.size(), UT_ISTREAM_ASCII);
    ramp.load(strm);
}

////////////////////////////////////////


OP_ERROR
SOP_OpenVDB_Motion_Blur_Camera::cookVDBSop(OP_Context& context)
{
    try {
        OP_AutoLockInputs inputs(this);
        if (inputs.lock(context) >= UT_ERROR_ABORT) return error();

        duplicateSource(0, context);

        const fpreal now = context.getTime();
        const GU_Detail* vecGeo = inputGeo(1, context);

        BlurParms blurParms = getBlurParms(this, now);

        // for camera blur, we need to sample the camera transform at each time sample
        // this requires unlocking the input to re-evaluate the camera transform
        const GU_Detail* cameraGeo = inputGeo(2, context);
        if (blurParms.timesamples.size() > 0 && cameraGeo) {
            hvdb::VdbPrimCIterator camIt{cameraGeo, matchGroup(*cameraGeo, evalStdString("cameragroup", now))};
            if (camIt) {
                // Similar to SOP_OpenVDB_Rasterize_Frustum
                auto& camera = blurParms.camera;
                camera.setTransformNow((*camIt)->getGrid().transform());

                // explicitly unlock reference geo input to be able to re-evaluate with a different context
                inputs.unlockInput(2);
                OP_Context refContext = context;

                const float frame = static_cast<float>(refContext.getFloatFrame());

                for (const float timesample : blurParms.timesamples) {
                    // Sample the transform at the time relative to the current frame
                    refContext.setFrame(frame + timesample);

                    if (inputs.lockInput(2, refContext) >= UT_ERROR_ABORT) return error();
                    const GU_Detail* cameraGeo = inputGeo(2);

                    hvdb::VdbPrimCIterator camItOffset{cameraGeo, matchGroup(*cameraGeo, evalStdString("cameragroup", now))};
                    if (camItOffset) {
                        auto transform = (*camItOffset)->getGrid().transform().copy();
                        if (!transform) {
                            throw std::runtime_error{"Cannot extract camera transform from VDB on second input."};
                        }
                        camera.appendTransform(*transform);
                    }
                    inputs.unlockInput(2);
                }
                bool staticCamera = true;
                for (const auto& camTransform : camera.transforms()) {
                    if (camTransform != camera.transformNow()) {
                        staticCamera = false;
                        break;
                    }
                }
                if (staticCamera) camera.clear();
            }
        }

        return doBlur(this, gdp, vecGeo, blurParms, now);

    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }

    return error();
}


OP_ERROR
SOP_OpenVDB_Motion_Blur::Cache::cookVDBSop(OP_Context& context)
{
    namespace blur = openvdb::tools::blur;

    try {
        const fpreal now = context.getTime();

        const GU_Detail* vecGeo = inputGeo(1, context);

        BlurParms blurParms = getBlurParms(this, now);
        return doBlur(this, gdp, vecGeo, blurParms, now);

    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }

    return error();
}