// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

/// @author Dan Bailey, Rick Hankins
///
/// @file PointRasterizeFrustum.h
///
/// @brief Volume rasterization of VDB Points using velocity and camera motion-blur

#ifndef OPENVDB_POINTS_POINT_RASTERIZE_FRUSTUM_HAS_BEEN_INCLUDED
#define OPENVDB_POINTS_POINT_RASTERIZE_FRUSTUM_HAS_BEEN_INCLUDED

#include <openvdb/math/Ray.h>
#include <openvdb/math/DDA.h>
#include <openvdb/util/NullInterrupter.h>
#include <openvdb/util/Assert.h>
#include <openvdb/thread/Threading.h>
#include <openvdb/tools/GridTransformer.h> // for tools::resampleToMatch()
#include <openvdb/tools/Interpolation.h>
#include "PointCount.h"
#include "PointDataGrid.h"

#include <cmath>
#include <memory>
#include <vector>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace points {


/// @brief How to composite points into a volume.
enum class RasterMode
{
    ACCUMULATE = 0,
    MAXIMUM,
    AVERAGE
};


/// @brief A camera class that provides an interface for camera motion blur when rasterizing
class RasterCamera
{
public:
    explicit RasterCamera(const math::Transform& transform);

    bool isStatic() const;

    void clear();
    void appendTransform(const math::Transform&, float weight = 1.0f);

    size_t size() const;

    void simplify();

    bool hasWeight(Index i) const;
    float weight(Index i) const;

    const math::Transform& transform(Index i) const;
    const math::Transform& firstTransform() const;
    const math::Transform& lastTransform() const;

    void setShutter(float start, float end);
    float shutterStart() const;
    float shutterEnd() const;

private:
    std::deque<math::Transform> mTransforms;
    std::deque<float> mWeights;
    // default to 180 degree film shutter
    float mShutterStart = -0.25f,
          mShutterEnd = 0.25f;
}; // class RasterCamera


/// @brief Pre-sampled, Houdini-free representation of a transfer ramp/curve.
///
/// The rasterization kernels live in core OpenVDB and must not depend on
/// Houdini types such as @c UT_Ramp. Callers (e.g. the Rasterize Frustum SOP)
/// evaluate their ramp once into a flat buffer of uniform samples and pass
/// the resulting @c RampSampleCache through @c FrustumRasterizerSettings.
///
/// The sample buffer is stored behind a @c shared_ptr so the cache is cheap
/// to copy and safe to share across TBB tasks. An empty cache is treated as
/// "no ramp" and is the default, ensuring existing call sites keep their
/// original behaviour without any code changes.
struct RampSampleCache
{
    /// Shared buffer of uniform ramp samples over the normalised range [0, 1].
    /// The first element corresponds to @c pos=0 and the last to @c pos=1.
    std::shared_ptr<const std::vector<float>> samples;

    /// Return true if the cache contains one or more samples.
    explicit operator bool() const
    {
        return samples && !samples->empty();
    }

    /// Evaluate the ramp at a normalised position in @c [0,1] using linear
    /// interpolation between adjacent samples. Positions outside the range
    /// are clamped. When the cache is empty a value of @c 1.0 is returned so
    /// that callers can treat the ramp as an optional multiplicative factor.
    float at(float pos) const
    {
        if (!samples || samples->empty())   return 1.0f;
        const auto& s = *samples;
        if (s.size() == 1)                  return s.front();
        if (pos <= 0.0f)                    return s.front();
        if (pos >= 1.0f)                    return s.back();
        const float idx = pos * static_cast<float>(s.size() - 1);
        const size_t i = static_cast<size_t>(std::floor(idx));
        const float frac = idx - static_cast<float>(i);
        return s[i] + frac * (s[i + 1] - s[i]);
    }
}; // struct RampSampleCache


/// @brief A group of shared settings to be used in the Volume Rasterizer
/// @param scaleByVoxelVolume   scale particle contributions by the volume of the receiving voxel
/// @param velocityAttribute    the name of the velocity attribute
/// @param velocityMotionBlur   bake the point velocities into the volume
/// @param clipToFrustum        if enabled and the transform is a frustum transform, eliminate
///                             points whose position does not lie within the frustum
/// @param clipBBox             an optional world-space bounding box to clip the points
///                             during rasterization
/// @param clipMask             an optional mask, each point samples the mask using a
///                             nearest-neighbor sampling and is only rasterized if active
/// @param invertMask           if mask is provided, only rasterize if sample is inactive
/// @param framesPerSecond      the global value for frames / second for computing motion blur
/// @param threaded             if enabled, use threading to accelerate rasterization
/// @param shutterShape         optional pre-sampled transfer ramp; when populated the
///                             rasterizer will shape density contributions across the
///                             shutter interval through it
/// @note rasterization can clip can using any combination of bounding box, mask and frustum
struct FrustumRasterizerSettings
{
    FrustumRasterizerSettings() = delete;

    explicit FrustumRasterizerSettings(const math::Transform& _transform)
        : transform(new math::Transform(_transform))
        , camera(_transform) { }

    math::Transform::Ptr transform;
    RasterCamera camera;
    bool scaleByVoxelVolume = false,
         useRadius = false,
         accurateFrustumRadius = false,
         accurateSphereMotionBlur = false,
         velocityMotionBlur = false,
         accelerationMotionBlur = false,
         threaded = true;
    float threshold = 1e-6f,
          radiusScale = 1.0f,
          framesPerSecond = 24.0f;
    Name velocityAttribute = "v",
         accelerationAttribute = "accel",
         radiusAttribute = "pscale";
    int motionSamples = 2;
    // Optional ramp used to shape density contributions across the shutter
    // interval. An empty cache (the default) leaves density untouched, so
    // existing callers see no behavioural change. The DSO layer is
    // responsible for populating this from a Houdini ramp before calling
    // the rasterizer.
    RampSampleCache shutterShape;
}; // struct FrustumRasterizerSettings


struct FrustumRasterizerMask
{
    using AccessorT = const tree::ValueAccessor<const MaskTree>;

    FrustumRasterizerMask() = default;

    explicit FrustumRasterizerMask(
        const math::Transform& transform,
        const MaskGrid* mask = nullptr,
        const BBoxd& bbox = BBoxd(),
        const bool clipToFrustum = true,
        const bool invert = false);

    operator bool() const;

    MaskTree::ConstPtr getTreePtr() const;

    bool valid(const Coord& ijk, AccessorT* acc) const;

    const CoordBBox& clipBBox() const;

private:
    MaskGrid::Ptr mMask;
    CoordBBox mClipBBox;
    bool mInvert = false;
}; // struct FrustumRasterizerMask


namespace point_rasterize_internal {

template <typename PointDataGridT>
class GridToRasterize;

} // namespace point_rasterize_internal


/// @brief  Efficient rasterization of one or more VDB Points grids into a linear
/// or frustum volume with the option to bake in camera or geometry motion blur.
///
/// @details The camera transform can be provided using a RasterCamera object to
/// offer linear camera motion blur and geometry motion blur is computed from reading
/// a velocity attribute on the points. Sub-sampled camera motion blur is planned.
///
/// @note For maximum memory efficiency, the data can optionally be streamed from
/// disk where the input VDB point grids are collapsed as they are read.
///
/// @note The total contribution for each point is spread across all the voxels being
/// rasterized into and weighted by the total volume represented by each voxel. In an
/// example use case where a point is moving away from a camera that is used to
/// generate a frustum volume being rasterized into, each successive voxel is larger in
/// size.
template<typename PointDataGridT>
class FrustumRasterizer
{
public:
    using GridPtr = typename PointDataGridT::Ptr;
    using GridConstPtr = typename PointDataGridT::ConstPtr;
    using GridToRasterize = point_rasterize_internal::GridToRasterize<PointDataGridT>;

    /// @brief main constructor
    /// @param settings             the shared settings for rasterizing, see class for more details
    /// @param mask                 a spatial mask to use to define the areas of rasterization
    /// @param interrupt            a pointer adhering to the util::NullInterrupter interface
    explicit FrustumRasterizer(
        const FrustumRasterizerSettings& settings,
        const FrustumRasterizerMask& mask = FrustumRasterizerMask(),
        util::NullInterrupter* interrupt = nullptr);

    /// @brief Append a PointDataGrid to the rasterizer (but don't rasterize yet).
    /// @param points   the PointDataGrid
    void addPoints(GridConstPtr& points);

    /// @brief Append a PointDataGrid to the rasterizer (but don't rasterize yet).
    /// @param points   the non-const PointDataGrid
    /// @param stream   if true, will destructively collapse attributes while
    ///                 accessing so as to minimize the memory footprint.
    void addPoints(GridPtr& points, bool stream = false);

    /// @brief Clear all PointDataGrids in the rasterizer.
    void clear();

    /// @brief Return number of PointDataGrids in the rasterizer.
    size_t size() const;

    /// @brief Return memory usage of the rasterizer.
    size_t memUsage() const;

    template <typename FilterT = points::NullFilter>
    FloatGrid::Ptr
    rasterizeUniformDensity(RasterMode mode=RasterMode::MAXIMUM,
        bool reduceMemory = false, float scale = 1.0f, const FilterT& filter = FilterT());

    template <typename FilterT = points::NullFilter>
    FloatGrid::Ptr
    rasterizeDensity(const openvdb::Name& attribute, RasterMode mode=RasterMode::MAXIMUM,
        bool reduceMemory = false, float scale = 1.0f, const FilterT& filter = FilterT());

    template <typename FilterT = points::NullFilter>
    GridBase::Ptr
    rasterizeAttribute(const Name& attribute, RasterMode mode=RasterMode::ACCUMULATE,
        bool reduceMemory = false, float scale = 1.0f, const FilterT& filter = FilterT());

    template <typename GridT, typename AttributeT, typename FilterT = points::NullFilter>
    typename GridT::Ptr
    rasterizeAttribute(const Name& attribute, RasterMode mode=RasterMode::ACCUMULATE,
        bool reduceMemory = false, float scale = 1.0f, const FilterT& filter = FilterT());

    template <typename GridT, typename FilterT = points::NullFilter>
    typename GridT::Ptr
    rasterizeMask(bool reduceMemory = false, const FilterT& filter = FilterT());

private:
    template <typename AttributeT, typename GridT, typename FilterT>
    void
    performRasterization(
        GridT& grid, RasterMode mode, const openvdb::Name& attribute,
        bool reduceMemory, float scale, const FilterT& filter);

private:
    FrustumRasterizerSettings mSettings;
    FrustumRasterizerMask mMask;

    util::NullInterrupter* mInterrupter;
    std::vector<GridToRasterize> mPointGrids;
}; // class FrustumRasterizer


/// @brief A struct that stores all include/exclude attribute names as strings
/// and is internally converted into the resolved MultiGroupFilter
struct RasterGroups
{
    std::vector<Name> includeNames;
    std::vector<Name> excludeNames;
}; // class RasterGroups


} // namespace points
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#include "impl/PointRasterizeFrustumImpl.h"

#endif // OPENVDB_POINTS_POINT_RASTERIZE_FRUSTUM_HAS_BEEN_INCLUDED
