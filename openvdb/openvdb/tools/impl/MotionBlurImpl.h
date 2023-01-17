// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0
//
/// @author Richard Jones
///
/// @file MotionBlurImpl.h
///

#ifndef OPENVDB_TOOLS_MOTION_BLUR_IMPL_HAS_BEEN_INCLUDED
#define OPENVDB_TOOLS_MOTION_BLUR_IMPL_HAS_BEEN_INCLUDED

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

namespace tools {

namespace blur {
namespace blur_internal {
/// Copy of LeafManager that provides a deterministic parallel_reduce
template<typename TreeT>
class DeterministicLeafManager : public openvdb::tree::LeafManager<TreeT>
{
public:
    using BaseT = openvdb::tree::LeafManager<TreeT>;
    using LeafNodeType = typename BaseT::LeafNodeType;
    using LeafRange = typename BaseT::LeafRange;
    DeterministicLeafManager(const TreeT& tree)
        : BaseT(tree) {}

    template<typename LeafOp>
    struct DeterministicLeafReducer
    {
        DeterministicLeafReducer(LeafOp &leafOp) : mLeafOp(&leafOp) {}
        DeterministicLeafReducer(const DeterministicLeafReducer &other, tbb::split)
            : mLeafOpPtr(std::make_unique<LeafOp>(*(other.mLeafOp), tbb::split()))
            , mLeafOp(mLeafOpPtr.get()) {}
        void run(const LeafRange& range, bool threaded)
        {
            threaded ? tbb::parallel_deterministic_reduce(range, *this) : (*this)(range);
        }
        void operator()(const LeafRange& range)
        {
            LeafOp &op = *mLeafOp;//local registry
            for (typename LeafRange::Iterator it = range.begin(); it; ++it) op(*it, it.pos());
        }
        void join(const DeterministicLeafReducer& other) { mLeafOp->join(*(other.mLeafOp)); }
        std::unique_ptr<LeafOp> mLeafOpPtr;
        LeafOp *mLeafOp = nullptr;
    };// DeterministicLeafReducer

    template<typename LeafOp>
    void deterministicReduce(LeafOp& op, bool threaded = true, size_t grainSize=1)
    {
        DeterministicLeafReducer<LeafOp> transform(op);
        transform.run(this->leafRange(grainSize), threaded);
    }
};

template <typename GridT,
    CompositionOperator CompositionT,
    typename WeightsT,
    typename SubsamplerT>
struct MotionBlurOp {

    using ValueT = typename GridT::TreeType::ValueType;
    using Vec3fDualSampler = DualGridSampler<Vec3fGrid::ConstAccessor, openvdb::tools::BoxSampler>;

    MotionBlurOp(const MaskGrid& topology,
        const GridT& grid,
        const WeightsT& weights,
        const Vec3fGrid* const velocity,
        const Vec3fGrid* const acceleration,
        const BlurParms<SubsamplerT>& parms,
        openvdb::util::NullInterrupter* interrupt = nullptr)
        : mTopology(&topology)
        , mGrid(&grid)
        , mWeights(weights)
        , mVelGrid(velocity)
        , mAccelGrid(acceleration)
        , mParms(parms)
        , mInterrupter(interrupt)
        , mTransformsMatch(parms.camera.isValid() && (grid.transform() == parms.camera.transformNow()))
        , mNewGrid(grid.copyWithNewTree()) {
            if (auto frustumMap = mNewGrid->transform().template constMap<openvdb::math::NonlinearFrustumMap>()) {
                const openvdb::BBoxd& bbox = frustumMap->getBBox();
                mFrustum.reset(openvdb::Coord::floor(bbox.min()), openvdb::Coord::ceil(bbox.max()));
            }
        }

    MotionBlurOp(const MotionBlurOp& other, tbb::split)
        : mTopology(other.mTopology)

        , mGrid(other.mGrid)
        , mWeights(other.mWeights)
        , mVelGrid(other.mVelGrid)
        , mAccelGrid(other.mAccelGrid)
        , mParms(other.mParms)
        , mInterrupter(other.mInterrupter)
        , mFrustum(other.mFrustum)
        , mTransformsMatch(other.mTransformsMatch)
        , mNewGrid(other.mNewGrid->copyWithNewTree()) {}

    inline bool validCoord(const openvdb::Coord& coord) const {
        return !mFrustum || mFrustum.isInside(coord);
    }

    void operator()(const openvdb::MaskTree::LeafNodeType& leaf, size_t pos) const
    {
        if (mInterrupter && mInterrupter->wasInterrupted()) {
            openvdb::thread::cancelGroupExecution();
            return;
        }
        // make a local weight cache as it could be used to store local data
        WeightsT weights(mWeights);

        assert(mNewGrid);
        typename GridT::Accessor resultAccessor = mNewGrid->getAccessor();
        const openvdb::math::Transform& outTransform = mNewGrid->transform();

        // mGrid has same transform as mask grid being iterated over
        const openvdb::math::Transform& inTransform = mGrid->transform();

        // Read accessors
        typename GridT::ConstAccessor accessor(mGrid->tree());
        openvdb::tools::GridSampler<typename GridT::ConstAccessor, openvdb::tools::BoxSampler> sampler(accessor, inTransform);
        // Samplers wrt index space positions from inTransform
        std::unique_ptr<Vec3fGrid::ConstAccessor> velAccessor;
        std::unique_ptr<Vec3fDualSampler> velSampler;
        std::unique_ptr<Vec3fGrid::ConstAccessor> accelAccessor;
        std::unique_ptr<Vec3fDualSampler> accelSampler;
        if (mVelGrid) {
            // hold on to an accessor as the sampler simply stores a pointer to it
            velAccessor = std::make_unique<Vec3fGrid::ConstAccessor>(mVelGrid->tree());
            velSampler = std::make_unique<Vec3fDualSampler>(*velAccessor, mVelGrid->transform(), inTransform);
        }
        if (mAccelGrid) {
            accelAccessor = std::make_unique<Vec3fGrid::ConstAccessor>(mAccelGrid->tree());
            accelSampler = std::make_unique<Vec3fDualSampler>(*accelAccessor, mAccelGrid->transform(), inTransform);
        }
        SubsamplerT subsampler(mParms.subsampler);
        subsampler.initLeaf(leaf, pos);
        const int numTimeSamples = mParms.timesamples.size();
        const int numSegments =  numTimeSamples - 1;
        const int maxSamplesPerSegment = openvdb::math::Ceil(static_cast<float>(mParms.maxsteps) / numSegments);
        const bool hasCamera = mParms.camera.isValid();

        // Use equations of motion to define blur trajectory
        // We store the locations of the segment start/ends, the vectors that define their direction/accurate length
        // and the number of samples along those lines to stamp
        std::vector<openvdb::Coord> coords(numTimeSamples);
        std::vector<openvdb::Vec3d> segments(numSegments);
        std::vector<size_t> samplesInSegment(numSegments);

        // The blur loops over each voxel in the input leaf, may be broken into further subsamples aka 'lines'.
        // From here it calculates the segments for each line, and then stamps these using a simple point-based rasterization.
        for (auto iter = leaf.cbeginValueOn(); iter; ++iter) {
            const openvdb::Vec3d posIS = iter.getCoord().asVec3s();

            const size_t subsamples = subsampler.subsamples(posIS);
            float subsampleDensityInv = 1.0f;
            // only add mode needs to account for number of subsamples to maintain same density
            if (CompositionT == CompositionOperator::ADD) {
                subsampleDensityInv /= subsamples;
            }
            subsampler.initPos(posIS);
            for (int sub = 0; sub < subsamples; ++sub) {
                openvdb::Vec3d subPosIS = subsampler.subsamplePos();
                // sample values wrt the current position from the input grid
                const ValueT gridValue = sampler.isSample(subPosIS) * subsampleDensityInv;
                openvdb::Vec3f vecWS = velSampler ? velSampler->isSample(subPosIS) : openvdb::Vec3f::zero();
                openvdb::Vec3f accelWS = accelSampler ? accelSampler->isSample(subPosIS) : openvdb::Vec3f::zero();
                // Get velocity at this position, sampler takes index space coord of input grid
                openvdb::Vec3d posWS = inTransform.indexToWorld(subPosIS);

                // get the end position of divisions for all timesamples
                for (int t = 0; t < numTimeSamples; ++t) {
                    openvdb::Vec3d pos = posWS +  mParms.timesamples[t] * (vecWS + 0.5 *  mParms.timesamples[t] * accelWS);
                    if (hasCamera) {
                        pos = mParms.camera.transform(t).worldToIndex(pos);
                        // index space position of camera transformed point wrt camera at time 0
                        if (!mTransformsMatch) {
                            // transform to relative to camera at time 0
                            pos = mParms.camera.transformNow().indexToWorld(pos);
                            // put in output transform index space
                            pos = outTransform.worldToIndex(pos);
                        }// else pos is already in output index space, do nothing
                    }
                    else {
                        // coords of line segment boundaries in result index space
                        pos = outTransform.worldToIndex(pos);
                    }
                    coords[t] = openvdb::Coord::round(pos);
                }

                // find line voxel boundaries in index space of our destination grid and calculate sample weights
                // for each coord, calculate the number of line samples required i.e. number of voxels stamped
                for (size_t i = 0; i < numSegments; ++i) {
                    // Get the number of samples given the vector value and start position
                    // This uses the fact we are rasterizing a one voxel wide line for each input voxel.
                    // Requires a single sample for each voxel along longest axis for continuous blur lines.
                    // Uses difference in start and end position to account for nonlinear transformation
                    // of world space velocity and assumes the change in voxel size is arbitrarily small
                    // across the length of the blur
                    const openvdb::Coord dX = coords[i+1] - coords[i];
                    int samples = std::max({std::abs(dX[0]), std::abs(dX[1]), std::abs(dX[2])}) + 1;
                    // @todo: allow other limits on the number of samples, e.g. length of streamline
                    samples = std::min(maxSamplesPerSegment, samples);
                    openvdb::Vec3d sampleVec = (samples > 1) ? dX.asVec3d() / (samples - 1) : dX.asVec3d();
                    // move start coord of any segments after the initial one to the next voxel in the line
                    if (i > 0) samples -= 1;
                    segments[i] = sampleVec;
                    samplesInSegment[i] = samples;
                }
                weights.clear();
                weights.initFromSegments(samplesInSegment);

                size_t weightOffset = 0;
                // iterate over the line segments and stamp their contributions to the result grid
                for (int idx = 0; idx < numSegments; ++idx) {
                    const openvdb::Vec3d startCoordAsVec = coords[idx].asVec3d();
                    const openvdb::Vec3d& sampleVec = segments[idx];
                    const size_t numSamples = samplesInSegment[idx];
                    // for this segment stamp the samples in a straight line along the segment vector direction
                    // first sample for subsequent segments is the last sample of the previous segment, skip this
                    size_t sampleOffset = (idx > 0 ? 1 : 0);
                    for (size_t voxelIdx = 0; voxelIdx < numSamples; ++voxelIdx) {
                        const openvdb::Coord coord = openvdb::Coord::round(startCoordAsVec + (voxelIdx + sampleOffset) * sampleVec);
                        const ValueT weight = weights.at(weightOffset + voxelIdx) * gridValue;
                        resultAccessor.modifyValue(coord, [weight](ValueT& value) {
                            if (CompositionT == CompositionOperator::ADD) {
                                value += weight;
                            }
                            else if (CompositionT == CompositionOperator::MIN) {
                                value = openvdb::tools::composite::min(value, weight);
                            }
                            else { // (CompositionT::COMP == CompositionOperator::MAX) {
                                value = openvdb::tools::composite::max(value, weight);
                            }
                        });
                    }
                    weightOffset += numSamples;
                }
            }
        }
    }

    void join(MotionBlurOp& other) {
        if (CompositionT == CompositionOperator::ADD) {
            openvdb::tools::compSum(*mNewGrid, *(other.mNewGrid));
        }
        else if (CompositionT == CompositionOperator::MIN) {
            openvdb::tools::compMin(*mNewGrid, *(other.mNewGrid));
        }
        else { // (CompositionT::COMP == CompositionOperator::MAX) {
        // @todo: use static_assert trick here and make else-if
            openvdb::tools::compMax(*mNewGrid, *(other.mNewGrid));
        }
    }

    bool execute(bool threaded = true) {
        // floating point addition is order dependent, so we need to use a deterministic reduce
        if constexpr (std::is_floating_point<typename GridT::ValueType>()
            && (CompositionT == CompositionOperator::ADD)) {
            blur_internal::DeterministicLeafManager<const openvdb::MaskTree> leafManager(mTopology->tree());
            leafManager.deterministicReduce(*this, threaded);
        }
        else {
            openvdb::tree::LeafManager<const openvdb::MaskTree> leafManager(mTopology->tree());
            leafManager.reduce(*this, threaded);
        }
        if (mInterrupter && mInterrupter->wasInterrupted()) { return false; }
        return true;
    }

    typename GridT::Ptr result() {
        return mNewGrid;
    }

private:

    const openvdb::MaskGrid* const mTopology;
    const GridT* const mGrid;
    const WeightsT mWeights;
    const Vec3fGrid* const mVelGrid;
    const Vec3fGrid* const mAccelGrid;
    const BlurParms<SubsamplerT> mParms;
    openvdb::util::NullInterrupter* mInterrupter;
    openvdb::CoordBBox mFrustum;
    bool mTransformsMatch;
    typename GridT::Ptr mNewGrid;

};

template<CompositionOperator CompositionT, typename WeightsT, typename SubsamplerT>
struct ApplyBlur
{
    ApplyBlur(const WeightsT& weights,
               const openvdb::Vec3fGrid* const velocity,
               const openvdb::Vec3fGrid* const accel,
               const BlurParms<SubsamplerT>& parms,
               openvdb::util::NullInterrupter* interrupter = nullptr)
        : mWeights(weights)
        , mVelGrid(velocity)
        , mAccelGrid(accel)
        , mParms(parms)
        , mInterrupter(interrupter) {}

    template<typename GridT>
    void operator()(GridT& values) {

        // copy topology of the grid, but with voxelized tiles to iterate over
        openvdb::MaskGrid::Ptr topology = openvdb::MaskGrid::create(values);
        topology->topologyUnion(values);
        topology->tree().voxelizeActiveTiles();

        MotionBlurOp<GridT, CompositionT, WeightsT, SubsamplerT>
            op(*topology, values, mWeights, mVelGrid, mAccelGrid, mParms, mInterrupter);

        if (op.execute()) {
            mResult = op.result();
        }
    }

    openvdb::GridBase::Ptr result() {
        return mResult;
    }

private:
    openvdb::GridBase::Ptr mResult;
    const WeightsT& mWeights;
    const openvdb::Vec3fGrid* const mVelGrid;
    const openvdb::Vec3fGrid* const mAccelGrid;
    const BlurParms<SubsamplerT> mParms;
    openvdb::util::NullInterrupter* mInterrupter;
};

} // namespace blur_internal


BlurCamera::BlurCamera() : mTransforms() {}

void BlurCamera::setTransformNow(const openvdb::math::Transform& transform) {
    mTransformNow = transform.copy();
}

const openvdb::math::Transform& BlurCamera::transformNow() const {
    assert(mTransformNow);
    return *mTransformNow;
}

void BlurCamera::appendTransform(const openvdb::math::Transform& transform) {
    mTransforms.emplace_back(transform);
}

const openvdb::math::Transform& BlurCamera::transform(int idx) const {
    assert(idx < mTransforms.size());
    return mTransforms[idx];
}

const std::vector<openvdb::math::Transform>& BlurCamera::transforms() const {
    return mTransforms;
}

size_t BlurCamera::size() const { return mTransforms.size(); }

bool BlurCamera::isValid() const {
    if (mTransformNow && this->size() > 0) return true;
    else return false;
}

void BlurCamera::clear() {
    mTransformNow = nullptr;
    mTransforms.clear();
}


template <CompositionOperator CompositionT, typename WeightT, typename SubsamplerT>
openvdb::GridBase::Ptr blurAlongStreamlines(const openvdb::GridBase& grid,
    const WeightT& weights,
    const BlurParms<SubsamplerT>& parms,
    const openvdb::Vec3fGrid* const vel,
    const openvdb::Vec3fGrid* const accel,
    openvdb::util::NullInterrupter* interrupt)
{
    using GridTypes = openvdb::RealGridTypes
                        ::Append<openvdb::Vec3SGrid>
                        ::Append<openvdb::Vec3DGrid>;
    // create the resolver from the weight cache and the composite type
    blur_internal::ApplyBlur<CompositionT, WeightT, SubsamplerT>
        op(weights, vel, accel, parms, interrupt);
    grid.apply<GridTypes>(op);
    return op.result();
}

} // namespace blur
} // namespace tools
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
#endif