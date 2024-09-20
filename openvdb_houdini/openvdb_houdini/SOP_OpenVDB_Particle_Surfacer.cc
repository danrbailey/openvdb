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
#include <openvdb/points/PrincipalComponentAnalysis.h>
#include <openvdb/points/PointGroup.h>
#include <openvdb/tools/LevelSetRebuild.h>
#include <openvdb/tools/Merge.h>
#include <openvdb/util/NullInterrupter.h>

#include <CH/CH_Manager.h>
#include <PRM/PRM_Parm.h>

namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;

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
    Ellipsoids,
    ParticleFluid,
    EllipsoidFluid
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
        .setDefault(PRMzeroDefaults)
        .setTooltip("Rebuild the level set after running the surfacing algorithm"));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "outputseparate", "Output Separate VDBs")
        .setDefault(PRMoneDefaults)
        .setTooltip("If enabled, all surfaces from different point clouds (or VDB Points grids)"
                    "will be output as individual SDFs, otherwise output will be the merged result."));

    parms.add(hutil::ParmFactory(PRM_SEPARATOR,"sepOutput", ""));

    parms.add(hutil::ParmFactory(PRM_STRING, "radiusattribute", "Radius Attribute")
        .setDefault("pscale")
        .setTooltip("The point attribute representing the particle radius,"
                    " if the attribute does not exist, a uniform value of 1 is assumed."));

    parms.add(hutil::ParmFactory(PRM_XYZ_J, "radiusscale", "Radius Scale")
        .setVectorSize(1)
        .setDefault(PRMoneDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0.0, PRM_RANGE_UI, 2.0)
        .setTooltip("A multiplier on the radius of the particles to be surfaced,"
                    " if no radius attribute is supplied this becomes the particle radius."));

    parms.add(hutil::ParmFactory(PRM_SEPARATOR,"sepRadius", ""));

    parms.add(hutil::ParmFactory(PRM_ORD, "mode", "Mode")
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
              "spherical",          "Spheres",
              "ellipsoids",         "Ellipsoids",
              "particlefluid",   "Particle Fluid",
              "ellipsoidfluid",  "Ellipsoid Fluid"
            })
        .setDefault("particlefluid")
        .setDocumentation("The method used to create a surface from the points.\n\n"
                    "*Spherical* - stamps spheres into a signed distance field."
                    " This is very fast and gives a good approximation of the surface suitable for simulation or for use with further post-processing.\n\n"
                    "*Particle Fluid* - uses a weighted-average method to create smooth surfaces from the points."
                    " This is good for slow moving and viscous fluids as it gives fast smooth results but can smooth out droplets and fine-details."));

    parms.add(hutil::ParmFactory(PRM_STRING, "vectorradiusattribute", "Vector Radius Attribute")
        .setDefault("scale")
        .setTooltip("The point attribute representing the particle radius,"
                    " if the attribute does not exist, a value of {1,1,1} is assumed."));

    parms.add(hutil::ParmFactory(PRM_XYZ_J, "vectorradiusscale", "Vector Radius Scale")
        .setVectorSize(3)
        .setDefault(PRMoneDefaults)
        .setRange(PRM_RANGE_RESTRICTED, 0.0, PRM_RANGE_UI, 2.0)
        .setTooltip("A multiplier on the radius of the particles to be surfaced,"
                    " if no radius attribute is supplied this becomes the particle radius."));

    parms.add(hutil::ParmFactory(PRM_STRING, "rotationattribute", "Rotation Attribute")
        .setDefault("orient")
        .setTooltip("The point attribute representing the ellipsoid rotation, "
                    "this must be a 3x3 rotation matrix (mat3s) and is mandatory."));

    parms.add(hutil::ParmFactory(PRM_TOGGLE, "useworldspaceinfluence", "Use World Space Influence Radius")
        .setDefault(PRMzeroDefaults)
        .setTooltip("If enabled, specify the influence radius explicitly in world space units, "
                    "otherwise is specified as a scale on the average particle radius."));

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

    parms.add(hutil::ParmFactory(PRM_FLT_J, "allowedstretch", "Minimum Sphericity")
        .setDefault(0.3f)
        .setRange(PRM_RANGE_RESTRICTED, 0.01, PRM_RANGE_RESTRICTED, 1.0)
        .setTooltip("To avoid particle imprints being flattened to a disk, "
                    " limit the allowed ratio of the minimum to maximum radii of ellipsoids created (as a fraction). "
                    "A value of 0 would effectively allow a particle's imprint to be completely flattened to a disk. "
                    "A value of 1 will instead only allow spherical imprints to be created."));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "dropletscale", "Droplet Scale")
        .setDefault(0.75f)
        .setRange(PRM_RANGE_RESTRICTED, 0.0, PRM_RANGE_UI, 1.0)
        .setTooltip("The radius of isolated particles that have a simple spherical "
                     "imprint is calculated by scaling the initial spherical radius by "
                     "this value."));

    parms.add(hutil::ParmFactory(PRM_INT_J, "minneighbours", "Neighbour Threshold")
        .setDefault(25)
        .setRange(PRM_RANGE_RESTRICTED, 1, PRM_RANGE_UI, 200)
        .setTooltip("If particle has less neighbours than this amount, "
                     "it will be treated as an isolated dropet."));

    parms.add(hutil::ParmFactory(PRM_FLT_J, "averagepositions", "Smooth Positions")
        .setDefault(0.9f)
        .setRange(PRM_RANGE_RESTRICTED, 0.0, PRM_RANGE_RESTRICTED, 1.0)
        .setTooltip("Linearly blends between Laplacian smoothed (averaged) positions of the "
                     "particles and their original positions."
                     "Blends between original (0) and average positions (1)."));

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
\"\"\"Converts point clouds to SDF VDBs.\"\"\"\n\
\n\
@overview\n\
\n\
This node converts point clouds (and VDB Points) to SDFs. \n\
Points can stamp a simple spherical footprint or use an\n\
averaged-position method and smooth their footprint, creating\n\
fluid-like surfaces.\n\
");
}


bool
SOP_OpenVDB_Particle_Surfacer::updateParmsFlags()
{
    bool changed = false;
    const fpreal t = CHgetEvalTime();
    const SurfaceType mode = static_cast<SurfaceType>(evalInt("mode", 0, t));

    const bool particleFluid = mode == SurfaceType::ParticleFluid;
    const bool ellipsoidFluid = mode == SurfaceType::EllipsoidFluid;
    const bool ellipsoids = mode == SurfaceType::Ellipsoids;

    const bool hasRefInput = this->nInputs() == 2;
    const bool requiresInfluence = particleFluid || ellipsoidFluid;
    const bool absoluteInfluence = static_cast<bool>(evalInt("useworldspaceinfluence", 0, t));

    changed |= enableParm("voxelsize", !hasRefInput);
    changed |= enableParm("referencegroup", hasRefInput);
    changed |= enableParm("vectorradiusscale",ellipsoids);
    changed |= setVisibleState("vectorradiusscale", ellipsoids);
    changed |= enableParm("vectorradiusattribute", ellipsoids);
    changed |= setVisibleState("vectorradiusattribute", ellipsoids);
    changed |= enableParm("radiusscale", !ellipsoids);
    changed |= enableParm("radiusattribute", !ellipsoids);
    changed |= enableParm("rotationattribute", ellipsoids);
    changed |= setVisibleState("rotationattribute", ellipsoids);
    changed |= setVisibleState("useworldspaceinfluence", requiresInfluence);
    changed |= setVisibleState("sepInfluence", requiresInfluence);
    changed |= enableParm("influencescale", requiresInfluence && !absoluteInfluence);
    changed |= setVisibleState("influencescale", requiresInfluence && !absoluteInfluence);
    changed |= enableParm("influenceradius", requiresInfluence && absoluteInfluence);
    changed |= setVisibleState("influenceradius", requiresInfluence && absoluteInfluence);
    changed |= setVisibleState("dropletscale", ellipsoidFluid);
    changed |= enableParm("dropletscale", ellipsoidFluid);
    changed |= setVisibleState("minneighbours", ellipsoidFluid);
    changed |= enableParm("minneighbours", ellipsoidFluid);
    changed |= setVisibleState("averagepositions", ellipsoidFluid);
    changed |= enableParm("averagepositions", ellipsoidFluid);
    changed |= setVisibleState("allowedstretch", ellipsoidFluid);
    changed |= enableParm("allowedstretch", ellipsoidFluid);

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

template <typename FilterT>
openvdb::FloatGrid::Ptr rasterSpheres(const openvdb::points::PointDataGrid& points,
    const std::string& radiusAttributeName,
    const float radiusScale,
    const float halfBand,
    const openvdb::math::Transform::Ptr sdfTransform,
    const FilterT& filter,
    hvdb::HoudiniInterrupter* boss)
{
    openvdb::points::SphereSettings<openvdb::TypeList<>, float, FilterT, hvdb::HoudiniInterrupter> settings;
    settings.radiusScale = radiusScale;
    settings.radius = radiusAttributeName;
    settings.transform = sdfTransform;
    settings.halfband = halfBand;
    settings.filter = &filter;
    settings.interrupter = boss;

    openvdb::GridPtrVec resultsVec = openvdb::points::rasterizeSdf(points, settings);
    return openvdb::gridPtrCast<openvdb::FloatGrid>(resultsVec[0]);
}

template <typename FilterT>
openvdb::FloatGrid::Ptr rasterSmoothSpheres(const openvdb::points::PointDataGrid& points,
    const std::string& radiusAttributeName,
    const float radiusScale,
    const float searchRadius,
    const float halfBand,
    const openvdb::math::Transform::Ptr sdfTransform,
    const FilterT& filter,
    hvdb::HoudiniInterrupter* boss)
{
    openvdb::points::SmoothSphereSettings<openvdb::TypeList<>, float, FilterT, hvdb::HoudiniInterrupter> settings;
    settings.radiusScale = radiusScale;
    settings.radius = radiusAttributeName;
    settings.searchRadius = searchRadius;
    settings.transform = sdfTransform;
    settings.halfband = halfBand;
    settings.filter = &filter;
    settings.interrupter = boss;

    openvdb::GridPtrVec resultsVec = openvdb::points::rasterizeSdf(points, settings);
    return openvdb::gridPtrCast<openvdb::FloatGrid>(resultsVec[0]);
}

template <typename FilterT>
openvdb::FloatGrid::Ptr rasterEllipsoids(const openvdb::points::PointDataGrid& points,
    const std::string& vectorRadiusAttributeName,
    const openvdb::Vec3f& vectorRadiusScale,
    const std::string& rotationAttributeName,
    const std::string& posWSAttributeName,
    const float halfBand,
    const openvdb::math::Transform::Ptr sdfTransform,
    const FilterT& filter,
    hvdb::HoudiniInterrupter* boss)
{
    if (boss) boss->start("Stamping ellipsoids into surface");
    openvdb::points::EllipsoidSettings<openvdb::TypeList<>, openvdb::Vec3f, FilterT, hvdb::HoudiniInterrupter> settings;
    settings.interrupter = boss;
    settings.radiusScale = vectorRadiusScale;
    settings.halfband = halfBand;
    settings.transform = sdfTransform;
    settings.filter = &filter;

    settings.radius = vectorRadiusAttributeName;
    settings.rotation = rotationAttributeName;
    settings.pws = posWSAttributeName;

    openvdb::GridPtrVec resultsVec = openvdb::points::rasterizeSdf(points, settings);
    return openvdb::gridPtrCast<openvdb::FloatGrid>(resultsVec[0]);
}


////////////////////////////////////////

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
            const GA_PrimitiveGroup* refGroup = matchGroup(*refGeo, evalStdString("referencegroup", time));

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
        const std::string vectorRadiusAttributeName = evalStdString("vectorradiusattribute", time);
        const float radiusScale = evalFloat("radiusscale", 0, time);
        const openvdb::Vec3f vectorRadiusScale = evalVec3f("vectorradiusscale", time);
        const bool rebuild = static_cast<bool>(evalInt("rebuildlevelset", 0, time));
        const bool separate = static_cast<bool>(evalInt("outputseparate", 0, time));
        const float averagePositions = evalFloat("averagepositions", 0, time);
        const int neighbourThreshold = evalInt("minneighbours", 0, time);
        const float dropletScale = evalFloat("dropletscale", 0, time);
        const float allowedStretch = evalFloat("allowedstretch", 0, time);

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
            auto gridPtr = openvdb::gridConstPtrCast<openvdb::points::PointDataGrid>(vdbPrim->getGridPtr());
            if (!gridPtr) continue;
            pointGrids.emplace_back(gridPtr);
        }

        // Convert all Houdini points that don't reference a VDB into a new VDB Points grid

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
                *pointGeo, /*compression=*/0, attributes, *pointsTransform);
            openvdb::points::PointDataGrid::ConstPtr houdiniPointsAsGrid = openvdb::ConstPtrCast<
                const openvdb::points::PointDataGrid>(houdiniPointsAsGridNonConst);
            pointGrids.emplace_back(houdiniPointsAsGrid);
        }

        std::vector<openvdb::FloatGrid::Ptr> outputs;
        // surface all point data grids
        for (const auto& points : pointGrids) {
            const auto iter = points->tree().cbeginLeaf();

            if (!iter) continue;
            if (boss.wasInterrupted()) break;

            const std::string groupStr(evalStdString("vdbpointsgroups", time));
            std::vector<std::string> include, exclude;
            points::AttributeSet::Descriptor::parseNames(include, exclude, groupStr);

            openvdb::FloatGrid::Ptr output;
            const points::AttributeSet::Descriptor&
                descriptor = iter->attributeSet().descriptor();
            const bool hasPscale(iter->hasAttribute(radiusAttributeName));
            const std::string& radNameOrEmpty = hasPscale ? radiusAttributeName : "";
            if (mode == SurfaceType::Spheres) {
                if (exclude.empty() && include.empty()) {
                    NullFilter filter;
                    output = rasterSpheres<NullFilter>(*points, radNameOrEmpty, radiusScale, halfBand, sdfTransform, filter, &boss);
                }
                else if (exclude.empty() && include.size() == 1) {
                    GroupFilter filter(include.front(), iter->attributeSet());
                    output = rasterSpheres<GroupFilter>(*points, radNameOrEmpty, radiusScale, halfBand, sdfTransform, filter, &boss);
                }
                else {
                    MultiGroupFilter filter(include, exclude, iter->attributeSet());
                    output = rasterSpheres<MultiGroupFilter>(*points, radNameOrEmpty, radiusScale, halfBand, sdfTransform, filter, &boss);
                }
            }
            else if (mode == SurfaceType::Ellipsoids) {
                const size_t vectorRadiusIdx = descriptor.find(vectorRadiusAttributeName);
                const std::string& vectorRadNameOrEmpty = vectorRadiusIdx != openvdb::points::AttributeSet::INVALID_POS ? vectorRadiusAttributeName : "";
                if (vectorRadiusIdx != openvdb::points::AttributeSet::INVALID_POS && descriptor.valueType(vectorRadiusIdx) !=
                    std::string("vec3s")) {
                    throw std::runtime_error("Wrong attribute type for attribute " + vectorRadiusAttributeName + ", expected vec3s");
                }

                const std::string& rotationAttributeName = evalStdString("rotationattribute", time);
                const size_t rotationIdx = descriptor.find(rotationAttributeName);

                if (rotationIdx != openvdb::points::AttributeSet::INVALID_POS && descriptor.valueType(rotationIdx) !=
                    std::string("mat3s")) {
                    throw std::runtime_error("Wrong attribute type for attribute " + rotationAttributeName + ", expected mat3s");
                }

                if (exclude.empty() && include.empty()) {
                    NullFilter filter;
                    output = rasterEllipsoids<NullFilter>(*points, vectorRadNameOrEmpty, vectorRadiusScale, rotationAttributeName,  "", halfBand, sdfTransform, filter, &boss);
                }
                else if (exclude.empty() && include.size() == 1) {
                    GroupFilter filter(include.front(), iter->attributeSet());
                    output = rasterEllipsoids<GroupFilter>(*points, vectorRadNameOrEmpty, vectorRadiusScale, rotationAttributeName, "",  halfBand, sdfTransform, filter, &boss);
                }
                else {
                    MultiGroupFilter filter(include, exclude, iter->attributeSet());
                    output = rasterEllipsoids<MultiGroupFilter>(*points, vectorRadNameOrEmpty, vectorRadiusScale, rotationAttributeName, "", halfBand, sdfTransform, filter, &boss);
                }
            }
            else if (mode == SurfaceType::ParticleFluid) {
                if (hasPscale && descriptor.valueType(descriptor.find(radiusAttributeName)) !=
                    std::string("float")) {
                    throw std::runtime_error("Wrong attribute type for attribute " + radiusAttributeName + ", expected float");
                }

                float scale = 1.0f;
                if (absoluteInfluence) {
                    scale = influenceRadius;
                }
                else {
                    scale = influenceScale * radiusScale;
                    if (hasPscale) {
                        scale *= openvdb::points::evalAverage<float>(points->tree(), radiusAttributeName);
                    }
                }

                if (exclude.empty() && include.empty()) {
                    NullFilter filter;
                    output = rasterSmoothSpheres<NullFilter>(*points, radNameOrEmpty, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                }
                else if (exclude.empty() && include.size() == 1) {
                    GroupFilter filter(include.front(), iter->attributeSet());
                    output = rasterSmoothSpheres<GroupFilter>(*points, radNameOrEmpty, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                }
                else {
                    MultiGroupFilter filter(include, exclude, iter->attributeSet());
                    output = rasterSmoothSpheres<MultiGroupFilter>(*points, radNameOrEmpty, radiusScale, scale, halfBand, sdfTransform, filter, &boss);
                }
            }
            else { //mode == SurfaceType::EllipsoidFluid
                // allow pscale and scale and multiply two together
                if (hasPscale && descriptor.valueType(descriptor.find(radiusAttributeName)) !=
                    std::string("float")) {
                    throw std::runtime_error("Wrong attribute type for attribute " + radiusAttributeName + ", expected float");
                }

                // need to add attributes to the points
                openvdb::points::PointDataGrid::Ptr pointsCopy = points->deepCopy();
                // @todo: drop all unnecessary copied attributes

                // only uses pscale for average pscale, should we incorporate scale?
                float scale = 1.0f;
                if (absoluteInfluence) {
                    scale = influenceRadius;
                }
                else {
                    scale = influenceScale * radiusScale;
                    if (hasPscale) {
                        scale *= openvdb::points::evalAverage<float>(pointsCopy->tree(), radiusAttributeName);
                    }
                }

                // Calculate ellipsoids from local neighbourhood
                boss.start("Calculating ellipsoid deformations from point distribution");

                openvdb::points::PcaAttributes a;
                openvdb::points::PcaSettings s;
                s.searchRadius = scale;
                s.neighbourThreshold = neighbourThreshold;
                s.allowedAnisotropyRatio = allowedStretch;
                s.averagePositions = averagePositions;
                s.nonAnisotropicStretch = dropletScale;
                openvdb::points::pca<PointDataGrid, openvdb::points::NullFilter, hvdb::HoudiniInterrupter>(*pointsCopy, s, a, &boss);
                openvdb::tree::LeafManager<openvdb::points::PointDataGrid::TreeType> manager(pointsCopy->tree());
                // scale the stretch attribute by the radius attribute
                if (hasPscale) {
                    manager.foreach([&](openvdb::points::PointDataTree::LeafNodeType& leafnode, size_t) {
                        openvdb::points::AttributeWriteHandle<openvdb::Vec3f> stretchHandle(leafnode.attributeArray(a.stretch));
                        openvdb::points::AttributeHandle<float> radHandle(leafnode.constAttributeArray(radiusAttributeName));
                        for (openvdb::Index i = 0; i < radHandle.size(); ++i)
                        {
                            stretchHandle.set(i, stretchHandle.get(i) * radHandle.get(i));
                        }
                    });
                }
                if (boss.wasInterrupted()) return error();

                const std::string positionWS = averagePositions > 0 ? a.positionWS : "";
                if (exclude.empty() && include.empty()) {
                    NullFilter filter;
                    output = rasterEllipsoids<NullFilter>(*pointsCopy, a.stretch, openvdb::Vec3f(radiusScale), a.rotation, positionWS, halfBand, sdfTransform, filter, &boss);
                }
                else if (exclude.empty() && include.size() == 1) {
                    GroupFilter filter(include.front(), iter->attributeSet());
                    output = rasterEllipsoids<GroupFilter>(*pointsCopy, a.stretch,  openvdb::Vec3f(radiusScale), a.rotation, positionWS,  halfBand, sdfTransform, filter, &boss);
                }
                else {
                    MultiGroupFilter filter(include, exclude, iter->attributeSet());
                    output = rasterEllipsoids<MultiGroupFilter>(*pointsCopy, a.stretch,  openvdb::Vec3f(radiusScale), a.rotation, positionWS,  halfBand, sdfTransform, filter, &boss);
                }
            }

            if (output) {
                outputs.emplace_back(output);
            }
        }
        // if not outputting separate grids, merge results
        if (!separate && outputs.size() > 1) {
            assert(outputs.front());

            openvdb::FloatGrid::Ptr output = openvdb::FloatGrid::create(outputs.front()->background());
            output->setTransform(outputs.front()->transform().copy());
            output->setGridClass(openvdb::GRID_LEVEL_SET);

            openvdb::tree::DynamicNodeManager<openvdb::FloatTree> nodeManager(output->tree());
            std::vector<openvdb::tools::TreeToMerge<openvdb::FloatTree>> treesToMerge;

            for (const auto& grid : outputs) {
                assert(grid);
                treesToMerge.emplace_back(grid->tree(), openvdb::Steal());
            }
            nodeManager.foreachTopDown(openvdb::tools::CsgUnionOp<openvdb::FloatTree>(treesToMerge));

            outputs.clear();
            outputs.emplace_back(output);
        }

        for (auto& grid : outputs) {
            if (rebuild) {
                assert(grid);
                grid = openvdb::tools::levelSetRebuild(*grid, 0, float(halfBand), float(halfBand));
            }
            grid->setName(surfaceName);
            hvdb::createVdbPrimitive(*gdp, grid);
        }
        // remove points and point grids that have been surfaced

    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }

    return error();
}
