// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0
//
/// @file MotionBlur.h
/// @author Richard Jones
/// @brief Methods for blurring grids along velocity, acceleration and cameras.

#ifndef OPENVDB_MOTION_BLUR_WAS_INCLUDED
#define OPENVDB_MOTION_BLUR_WAS_INCLUDED

#include <openvdb/openvdb.h>
#include <openvdb/Types.h>
#include <openvdb/Grid.h>
#include <openvdb/tree/ValueAccessor.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/tree/NodeManager.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/thread/Threading.h>
#include <openvdb/util/NullInterrupter.h>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace tools {

namespace blur
{
    /// @brief Representation of camera for comparing timesampled views against a reference view
    ///     when calculating blur
    /// @todo: consolidate with similar camera in PointRasterizeFrustum.h
    struct BlurCamera {

        BlurCamera();
        void setTransformNow(const openvdb::math::Transform& transform);
        void appendTransform(const openvdb::math::Transform& transform);
        const openvdb::math::Transform& transformNow() const;
        const openvdb::math::Transform& transform(int idx) const;
        const std::vector<openvdb::math::Transform>& transforms() const;
        size_t size() const;
        bool isValid() const;
        void clear();

    private:

        openvdb::math::Transform::Ptr mTransformNow;
        std::vector<openvdb::math::Transform> mTransforms;
    };

    /// @brief Parameters used for blurring, e.g, time and voxel subsamples
    ///    and the camera to use for calculating relative blur
    template <typename SubSamplerT>
    struct BlurParms {
        std::vector<float> timesamples = {};
        int maxsteps = 1000;
        BlurCamera camera;
        SubSamplerT subsampler;
    };

    /// @brief Composition operators for combining values from different input voxels
    ///     along streamlines.
    ///    MIN: take the minimum value from contributing voxels
    ///    MAX: take the maximum value from contributing voxels
    ///    ADD: add the values from contributing voxels
    enum class CompositionOperator
    {
        MIN,
        MAX,
        ADD
    };

    // @brief WeightT interface is used for weighting values being blurred along their streamline
    ///     These are calculated upfront for each line (containing potentially multiple segments)
    ///     to allow for normalisation of their values. Assumes a discrete collection of weights
    ///     to be stored for each line, e.g. for a line of length n, each voxel has a value to return,
    ///     accessible through its index in 0 to n.
    ///  struct WeightT {
    ///  void clear() {}
    ///  @brief calculate all the weights for this line composed of a collection of
    //      variable length segments
    ///  void initFromSegments(const std::vector<size_t>& segmentlengths) {}
    /// //@brief Return the value at idx position in the cache
    ///  float at(const size_t idx) {}
    ///  }
    /// e.g.
    // template<bool Average>
    // struct CubicKernelWeight
    // {
    //     CubicKernelWeight(const float center = 0.0) : mCenter(center) {}
    //     void clear() {}
    //
    //     void initFromSegments(const std::vector<size_t>& segments) {
    //         const size_t samples = std::reduce(segments.begin(), segments.end());
    //         mValues.resize(samples);
    //         mLength = static_cast<float>(samples);
    //         std::transform(mValues.begin(), mValues.end(), mValues.begin(), [this](float pos) {
    //             if (pos > 1.0 || pos < 0.0) return 0.0;
    //             else return 1 - std::pow(std::fabs(pos - mCenter) / mLength, 3.0);
    //         });
    //         if (Average) {
    //             mInvTotal = 1.f / std::accumulate(mValues.begin(), mValues.end(), 0.0);
    //             std::transform(mValues.begin(), mValues.end(), mValues.begin(), [this](float x) { return x * mInvTotal; });
    //         }
    //     }
    //
    //     float at(const size_t idx) const { return mValues[idx]; }
    //
    // private:
    //
    //     const float mCenter;
    //     std::vector<float> mValues;
    //     size_t mLength;
    //     float mInvTotal;
    // };

    /// @brief Weight interface that assumes a uniform weight for all positions along the streamline
    template<bool Average>
    struct UniformWeight
    {
        UniformWeight(const float value = 1.0) : mValue(value) {}
        void clear() {}

        void initFromSegments(const std::vector<size_t>& segments) {
            if (Average) mWeight = mValue / std::reduce(segments.begin(), segments.end());
            else mWeight = mValue;
        }

        float at(const size_t) const { return mWeight; }

        const float mValue;
        float mWeight;
    };


    /// @brief Subsampler struct that defines how to subsample a voxel from the input
    ///   this specifies the number of subsamples that should be used and the locations
    ///   of the samples in a voxel. This takes random samples in the voxel space, after
    ///   an initial sample at the voxel centre.
    ///
    struct Rand01Subsampler
    {
        Rand01Subsampler(const size_t subsamples = 1, const size_t seed = 0)
            : mSubsamples(subsamples)
            , mSeed(seed)
            , mFirstSample(true)
            , mRand01(seed)
            , mPos(openvdb::Vec3d(0.0)) {}

        Rand01Subsampler(const Rand01Subsampler& other)
            : Rand01Subsampler(other.mSubsamples, other.mSeed) {}

        inline void initLeaf(const openvdb::MaskTree::LeafNodeType& leaf, size_t leafpos) {
            mRand01.setSeed(mSeed + leafpos);
        }

        inline void initPos(const openvdb::Vec3d& pos) {
            mFirstSample = true;
            mPos = pos;
        }
        inline size_t subsamples(const openvdb::Vec3d&) const { return mSubsamples; }

        inline openvdb::Vec3d subsamplePos()
        {
            if (!mFirstSample) return mPos + openvdb::Vec3d(mRand01() - 0.5,
                                                            mRand01() - 0.5,
                                                            mRand01() - 0.5);
            else {
                mFirstSample = false;
            }
            return mPos;
        }

        Rand01Subsampler& operator=(const Rand01Subsampler& other) {
            if (this != &other) {
                mSubsamples = other.mSubsamples;
                mSeed = other.mSeed;
                mFirstSample = other.mFirstSample;
                mPos = other.mPos;
                mRand01 = other.mRand01;
            }
            return *this;
        }

        private:
            size_t mSubsamples;
            size_t mSeed;
            bool mFirstSample;
            openvdb::Vec3d mPos;
            openvdb::math::Random01 mRand01;
    };

    /// @brief Blur a grid along streamlines defined by an input velocity
    ///     and/or acceleration. Velocity and acceleration are sampled through some
    ///     convenience structures with an interface following that of the
    ///     UniformValueSampler. The distribution of the input values along the
    ///     streamlines are defined by the weights at each voxel sample along the
    ///     streamline. This is calculated using a structure with the interface
    ///     following that of the UniformWeight. The input grid is unchanged and
    ///     a blurred version is returned. The output grid may require pruning.
    ///
    /// @tparam CompositionT the composition operator to use
    /// @tparam WeightT the weight structure to use
    /// @tparam SubsamplerT the subsampler to use
    /// @param grid  an OpenVDB grid or tree from which to run the blur operation
    /// @param parms the blur parameters
    /// @param vel a velocity grid or nullptr
    /// @param accel an acceleration grid or nullptr
    /// @param interrupt the interrupter
    /// @return a new grid containing the blurred values

    template <CompositionOperator CompositionT, typename WeightT, typename SubsamplerT>
    openvdb::GridBase::Ptr blurAlongStreamlines(const openvdb::GridBase& grid,
        const WeightT& weight,
        const BlurParms<SubsamplerT>& parms,
        const openvdb::Vec3fGrid* const vel,
        const openvdb::Vec3fGrid* const accel,
        openvdb::util::NullInterrupter* interrupt = nullptr);

/////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace blur
} // namespace tools
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#include "impl/MotionBlurImpl.h"

#endif // OPENVDB_MOTION_BLUR_WAS_INCLUDED