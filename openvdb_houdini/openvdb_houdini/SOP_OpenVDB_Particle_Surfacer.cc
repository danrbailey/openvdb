// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0
//
/// @author Nick Avramoussis, Richard Jones
///
/// @file SOP_OpenVDB_Particle_Surfacer.cpp
///
/// @brief Surface points into a VDB Level Set using a variety of methods

#include <houdini_utils/ParmFactory.h>
#include <openvdb_houdini/Utils.h>
#include <openvdb_houdini/PointUtils.h>
#include <openvdb_houdini/SOP_NodeVDB.h>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/points/IndexFilter.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointStatistics.h>
#include <openvdb/points/PointRasterizeSDF.h>
#include <openvdb/tools/LevelSetRebuild.h>
#include <openvdb/tools/Merge.h>
#include <openvdb/util/NullInterrupter.h>

#include <CH/CH_Manager.h>
#include <PRM/PRM_Parm.h>

namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;

using SupportedGridT =
    openvdb::TypeList<bool, int32_t, int64_t, float, double,
        openvdb::Vec3f, openvdb::Vec3d, openvdb::Vec3i>;

class SOP_OpenVDB_Particle_Surfacer: public openvdb_houdini::SOP_NodeVDB
{
public:
    SOP_OpenVDB_Particle_Surfacer(OP_Network*, const char* name, OP_Operator*);
    ~SOP_OpenVDB_Particle_Surfacer() override {}
    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);
    class Cache: public SOP_VDBCacheOptions { OP_ERROR cookVDBSop(OP_Context&) override; };

protected:
    bool updateParmsFlags() override;
};


////////////////////////////////////////

namespace
{

enum SurfaceType
{
    Spheres,
    ParticleFluid
};

}

////////////////////////////////////////

void
newSopOperator(OP_OperatorTable* table)
{
    if (table == nullptr) return;

    hutil::ParmList parms;

    // INPUT PARMS

    parms.add(hutil::ParmFactory(PRM_STRING, "group", "Group")
        .setChoiceList(&hutil::PrimGroupMenu)
        .setTooltip("Specify a subset of the input points/VDB Points grids to surface.")
        .setDocumentation(
            "A subset of the input points to be processed - points or VDB Points grids"));

    parms.add(hutil::ParmFactory(PRM_STRING, "vdbpointsgroups", "VDB Points Groups")
        .setChoiceList(&hvdb::VDBPointsGroupMenuInput1)
        .setDefault("")
        .setHelpText("Specify VDB Points Groups to use. (Default is all groups)"));

    parms.add(hutil::ParmFactory(PRM_STRING, "referencegroup", "Reference VDB")
        .setChoiceList(&hutil::PrimGroupMenuInput2)
        .setTooltip(
            "Give the output VDB the same orientation and voxel size as the selected VDB."));

    // SURFACE PARMS
    parms.add(hutil::ParmFactory(PRM_STRING, "surfacevdbname", "Output Surface VDB")
        .setDefault("surface")
        .setTooltip("The name of the surface VDB to be created."));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "voxelsize", "Voxel Size")
        .setDefault(PRMpointOneDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 1e-5, PRM_RANGE_UI, 5)
        .setTooltip("Uniform voxel edge length in world units.  "
            "Decrease the voxel size to increase the volume resolution."));

    parms.add(hutil::ParmFactory(PRM_INT_J, "halfbandvoxels", "Half-Band Voxels")
        .setDefault(PRMthreeDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 10)
        .setTooltip("Half the width of the narrow band in voxel units.  "
            "The default value 3 is recommended for level set volumes."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "rebuildlevelset", "Rebuild Level Set")
        .setDefault(PRMoneDefaults)
        .setTooltip("Rebuild the level set after running the surfacing algorithm"));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "mergeoutput", "Merge Output VDBs")
        .setDefault(PRMoneDefaults)
        .setTooltip("If enabled, all surfaces from different points VDBs or from Houdini points "
                    "are merged into a single VDB, otherwise will output as separate VDBs"));

    parms.add(hutil::ParmFactory(PRM_SEPARATOR,"sepOutput", ""));

    parms.add(hutil::ParmFactory(PRM_STRING, "radiusattribute", "Radius Attribute")
        .setDefault("pscale")
        .setTooltip("The point attribute representing the particle radius,"
                    " if the attribute does not exist, a uniform value of 1 is assumed."));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "particleradius", "Radius Scale")
        .setDefault(PRMoneDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0.0, PRM_RANGE_UI, 2.0)
        .setTooltip("A multiplier on the radius of the particles to be surfaced,"
                    " if no radius attribute is supplied this becomes the particle radius."));

    parms.add(hutil::ParmFactory(PRM_SEPARATOR,"sepRadius", ""));

    parms.add(hutil::ParmFactory(PRM_ORD, "mode", "Mode")
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
              "spherical",          "Spherical",
              "particlefluid",   "Particle Fluid"
            })
        .setDefault(PRMoneDefaults)
        .setDocumentation("The method used to create a surface from the points.\n\n"
                    "*Spherical* - stamps spheres into a signed distance field."
                    " This is very fast and gives a good approximation of the surface suitable for simulation or for use with further post-processing.\n\n"
                    "*Particle Fluid* - uses a weighted-average method to create smooth surfaces from the points."
                    " This is good for slow moving and viscous fluids as it gives fast smooth results but can smooth out droplets and fine-details."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "useworldspaceinfluence", "Use World Space Influence Radius")
        .setDefault(PRMzeroDefaults)
        .setTooltip("If enabled, specify the influence radius explicitly in world space units, "
                    "otherwise is specified as a scale on the average (scaled by above) particle radius."));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "influencescale", "Influence Radius Scale")
        .setDefault(PRMtwoDefaults)
        .setRange(PRM_RANGE_UI, 1.0, PRM_RANGE_UI, 4.0)
        .setTooltip("The distance at which particles interact is this value multiplied by the final average particle radius."
                    "Suggested values are around 2-4. "
                    "Values much larger than this can be very inefficient and give undesirable results."));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "influenceradius", "Influence Radius")
        .setDefault(PRMpointOneDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0.0, PRM_RANGE_UI, 1.0)
        .setTooltip("The absolute world space value for the distance at which particles interact."
                    "Suggested values are of around 2-4x the average particle radius."
                    "Values much larger than this can be very inefficient and give undesirable results."));

    hvdb::OpenVDBOpFactory("VDB Particle Surfacer",
        SOP_OpenVDB_Particle_Surfacer::factory, parms, *table)
        .addInput("Points to surface")
        .addOptionalInput("Optional VDB grid that defines the output transform. "
            "The half-band width is matched if the input grid is a level set.")
        .setVerb(SOP_NodeVerb::COOK_GENERATOR,
            []() { return new SOP_OpenVDB_Particle_Surfacer::Cache; })
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Converts points and points VDBs to a levelset surface.\"\"\"\n\
\n\
@overview\n\
\n\
This node converts particles (points or VDB Points grids) to a levelset surface. Its modes\n\
to allow different methods of performing this conversion. Points can stamp a simple spherical\n\
footprint or use some more advanced methods to average (and smooth) their footprint, creating a\n\
fluid-like surface.\n\
");
}


bool
SOP_OpenVDB_Particle_Surfacer::updateParmsFlags()
{
    bool changed = false;
    const fpreal t = CHgetEvalTime();
    const SurfaceType mode = static_cast<SurfaceType>(evalInt("mode", 0, t));

    const bool particleFluid = mode == SurfaceType::ParticleFluid;
    const bool hasRefInput = this->nInputs() == 2;
    const bool requiresInfluence = particleFluid;
    const bool absoluteInfluence = static_cast<bool>(evalInt("useworldspaceinfluence", 0, t));

    changed |= enableParm("voxelsize", !hasRefInput);
    changed |= enableParm("referencegroup", hasRefInput);
    changed |= setVisibleState("useworldspaceinfluence", requiresInfluence);
    changed |= setVisibleState("sepInfluence", requiresInfluence);
    changed |= enableParm("influencescale", requiresInfluence && !absoluteInfluence);
    changed |= setVisibleState("influencescale", requiresInfluence && !absoluteInfluence);
    changed |= enableParm("influenceradius", requiresInfluence && absoluteInfluence);
    changed |= setVisibleState("influenceradius", requiresInfluence && absoluteInfluence);
    return changed;
}

////////////////////////////////////////


OP_Node*
SOP_OpenVDB_Particle_Surfacer::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Particle_Surfacer(net, name, op);
}


SOP_OpenVDB_Particle_Surfacer::SOP_OpenVDB_Particle_Surfacer(OP_Network* net,
    const char* name, OP_Operator* op)
    : hvdb::SOP_NodeVDB(net, name, op)
{}

////////////////////////////////////////

template <typename FilterT, typename ...Args>
inline openvdb::FloatGrid::Ptr raster(const Args&... args)
{
    return openvdb::points::rasterizeSpheres<
                openvdb::points::PointDataGrid,
                openvdb::FloatGrid,
                FilterT,
                hvdb::HoudiniInterrupter>
                    (args...);
}

template <typename FilterT, typename ...Args>
inline openvdb::FloatGrid::Ptr rasterP(const Args&... args)
{
    return openvdb::points::rasterizeSpheres<
                openvdb::points::PointDataGrid,
                float,
                openvdb::FloatGrid,
                FilterT,
                hvdb::HoudiniInterrupter>
                    (args...);
}

template <typename FilterT, typename ...Args>
inline openvdb::FloatGrid::Ptr rasterZb(const Args&... args)
{
    return openvdb::points::rasterizeSmoothSpheres<
                openvdb::points::PointDataGrid,
                openvdb::FloatGrid,
                FilterT,
                hvdb::HoudiniInterrupter>
                    (args...);
}

template <typename FilterT, typename ...Args>
inline openvdb::FloatGrid::Ptr rasterZbP(const Args&... args)
{
    return openvdb::points::rasterizeSmoothSpheres<
                openvdb::points::PointDataGrid,
                float,
                openvdb::FloatGrid,
                FilterT,
                hvdb::HoudiniInterrupter>
                    (args...);
}

OP_ERROR
SOP_OpenVDB_Particle_Surfacer::Cache::cookVDBSop(OP_Context& context)
{
    using namespace openvdb;
    using namespace openvdb::points;

    try {
        hvdb::HoudiniInterrupter boss("VDB Particle Surfacer");

        const fpreal time = context.getTime();

        const GU_Detail* pointGeo = inputGeo(0);
        const GU_Detail* refGeo = inputGeo(1);

        const std::string groupStr = evalStdString("group", time);
        const GA_PrimitiveGroup* group = matchGroup(*pointGeo, evalStdString("group", time));

        math::Transform::Ptr sdfTransform;
        if (refGeo) {
            // Get the first grid in the group's transform
            const GA_PrimitiveGroup *refGroup = matchGroup(*refGeo, evalStdString("referencegroup", time));

            hvdb::VdbPrimCIterator gridIter(refGeo, refGroup);

            if (gridIter) {
                sdfTransform = (*gridIter)->getGrid().transform().copy();
            } else {
                addError(SOP_MESSAGE, "Could not find a reference grid");
                return error();
            }
        }
        else {
            auto voxelSize = evalFloat("voxelsize", 0, time);
            sdfTransform = math::Transform::createLinearTransform(voxelSize);
        }

        if (!sdfTransform->isLinear()) throw std::runtime_error("Surfacing only supports uniform voxels");

        const std::string surfaceName = evalStdString("surfacevdbname", time);
        const Real halfBand = Real(evalInt("halfbandvoxels", 0, time));
        const SurfaceType mode = static_cast<SurfaceType>(evalInt("mode", 0, time));
        const bool absoluteInfluence = static_cast<bool>(evalInt("useworldspaceinfluence", 0, time));
        const Real influenceRadius = Real(evalFloat("influenceradius", 0, time));
        const Real influenceScale = Real(evalFloat("influencescale", 0, time));
        const std::string radiusAttributeName = evalStdString("radiusattribute", time);
        const Real radiusScale = Real(evalFloat("particleradius", 0, time));
        const bool rebuildLevelSet = static_cast<bool>(evalInt("rebuildlevelset", 0, time));
        const bool mergeoutput = static_cast<bool>(evalInt("mergeoutput", 0, time));

        std::vector<openvdb::points::PointDataGrid::ConstPtr> pointGrids;
        std::vector<GA_Offset> vdbPrimOffsets;
        for (hvdb::VdbPrimCIterator vdbIt(pointGeo, group); vdbIt; ++vdbIt) {
            const GU_PrimVDB* vdbPrim = *vdbIt;

            // mark point offset as a point referencing a VDB
            vdbPrimOffsets.emplace_back(vdbPrim->getPointOffset(0));
        }

        for (hvdb::VdbPrimCIterator vdbIt(pointGeo, group); vdbIt; ++vdbIt) {

            const GU_PrimVDB* vdbPrim = *vdbIt;

            // only process if grid is a PointDataGrid
            auto gridPtr = openvdb::gridConstPtrCast<openvdb::points::PointDataGrid>(vdbPrim->getConstGridPtr());
            if(!gridPtr) continue;
            pointGrids.emplace_back(gridPtr);
        }

        // Convert all Houdini points that don't reference a VDB into a new VDB

        if (pointGeo->getNumPoints() > vdbPrimOffsets.size()) {
            // compute auto voxel-size based on point distribution
            openvdb::math::Mat4d matrix(openvdb::math::Mat4d::identity());
            const float voxelSize = hvdb::computeVoxelSizeFromHoudini(*pointGeo, /*pointsPerVoxel=*/8,
                matrix, /*rounding=*/5, boss);
            matrix.preScale(openvdb::Vec3d(voxelSize) / openvdb::math::getScale(matrix));
            auto pointsTransform = openvdb::math::Transform::createLinearTransform(matrix);

            // convert Houdini points to VDB Points
            openvdb_houdini::AttributeInfoMap attributes;
            if (!radiusAttributeName.empty()) {
                attributes[radiusAttributeName] = {0, false};
            }

            openvdb::points::PointDataGrid::Ptr houdiniPointsAsGridNonConst = hvdb::convertHoudiniToPointDataGrid(
                *pointGeo, /*compression=*/1, attributes, *pointsTransform);
            openvdb::points::PointDataGrid::ConstPtr houdiniPointsAsGrid = openvdb::ConstPtrCast<
                const openvdb::points::PointDataGrid>(houdiniPointsAsGridNonConst);
            pointGrids.emplace_back(houdiniPointsAsGrid);
        }

        std::vector<openvdb::FloatGrid::Ptr> gridsToMerge;
        openvdb::FloatGrid::Ptr output;

        // surface all point data grids
        for (const auto& points : pointGrids) {
            const auto iter = points->constTree().cbeginLeaf();

            if (!iter) continue;
            if (boss.wasInterrupted()) break;

            const points::AttributeSet::Descriptor&
                descriptor = iter->attributeSet().descriptor();
            const bool hasPscale(iter->hasAttribute(radiusAttributeName));
            if (hasPscale && descriptor.valueType(descriptor.find(radiusAttributeName)) !=
                std::string("float")) {
                throw std::runtime_error("Wrong attribute type for attribute " + radiusAttributeName + ", expected float");
            }

            const std::string groupStr(evalStdString("vdbpointsgroups", time));
            std::vector<std::string> include, exclude;
            points::AttributeSet::Descriptor::parseNames(include, exclude, groupStr);

            // determine attributes to transfer

            if (mode == SurfaceType::Spheres) {
                if (exclude.empty() && include.empty()) {
                    NullFilter filter;
                    if (hasPscale) output = rasterP<NullFilter>(*points, radiusAttributeName, radiusScale, halfBand, sdfTransform, filter, &boss);
                    else           output = raster<NullFilter>(*points, radiusScale, halfBand, sdfTransform, filter, &boss);
                }
                else if (exclude.empty() && include.size() == 1) {
                    GroupFilter filter(include.front(), iter->attributeSet());
                    if (hasPscale) output = rasterP<GroupFilter>(*points, radiusAttributeName, radiusScale, halfBand, sdfTransform, filter, &boss);
                    else           output = raster<GroupFilter>(*points, radiusScale, halfBand, sdfTransform, filter, &boss);
                }
                else {
                    MultiGroupFilter filter(include, exclude, iter->attributeSet());
                    if (hasPscale) output = rasterP<MultiGroupFilter>(*points, radiusAttributeName, radiusScale, halfBand, sdfTransform, filter, &boss);
                    else           output = raster<MultiGroupFilter>(*points, radiusScale, halfBand, sdfTransform, filter, &boss);
                }
            }
            else { //mode == SurfaceType::ParticleFluid

                double scale;
                if (absoluteInfluence) {
                    scale = influenceRadius;
                }
                else {
                    scale = influenceScale * radiusScale;
                    if (hasPscale) {
                        double avg;
                        if (openvdb::points::evalAverage<float>(points->tree(), radiusAttributeName, avg)) scale *= avg;
                    }
                }

                if (exclude.empty() && include.empty()) {
                    NullFilter filter;
                    if (hasPscale) output = rasterZbP<NullFilter>(*points, radiusAttributeName, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                    else           output = rasterZb<NullFilter>(*points, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                }
                else if (exclude.empty() && include.size() == 1) {
                    GroupFilter filter(include.front(), iter->attributeSet());
                    if (hasPscale) output = rasterZbP<GroupFilter>(*points, radiusAttributeName, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                    else           output = rasterZb<GroupFilter>(*points, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                }
                else {
                    MultiGroupFilter filter(include, exclude, iter->attributeSet());
                    if (hasPscale) output = rasterZbP<MultiGroupFilter>(*points, radiusAttributeName, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                    else           output = rasterZb<MultiGroupFilter>(*points, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                }
            }

            if (output) {
                if (rebuildLevelSet && !mergeoutput) {
                    output = tools::levelSetRebuild(*output, 0, float(halfBand), float(halfBand));
                }
                output->setName(surfaceName);
                if (mergeoutput) gridsToMerge.emplace_back(output);
                else hvdb::createVdbPrimitive(*gdp, output);
            }
        }
        if (mergeoutput && !gridsToMerge.empty()) {
            assert(gridsToMerge.front());
            output = openvdb::FloatGrid::create(*gridsToMerge.front());
            output->setName(surfaceName);
            openvdb::tree::DynamicNodeManager<openvdb::FloatTree> nodeManager(output->tree());
            std::vector<openvdb::tools::TreeToMerge<openvdb::FloatTree>> treesToMerge;
            for (const auto& grid : gridsToMerge) treesToMerge.emplace_back(grid->tree(), openvdb::Steal());
            nodeManager.foreachTopDown(openvdb::tools::CsgUnionOp<openvdb::FloatTree>(treesToMerge));
            if (rebuildLevelSet) {
                output = tools::levelSetRebuild(*output, 0, float(halfBand), float(halfBand));
            }
            output->setName(surfaceName);
            hvdb::createVdbPrimitive(*gdp, output);
        }
    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }

    return error();
}
