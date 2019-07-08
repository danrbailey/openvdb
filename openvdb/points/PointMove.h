///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DreamWorks Animation nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////

/// @author Dan Bailey
///
/// @file PointMove.h
///
/// @brief Ability to move VDB Points using a custom deformer.
///
/// Deformers used when moving points are in world space by default and must adhere
/// to the interface described in the example below:
/// @code
/// struct MyDeformer
/// {
///     // A reset is performed on each leaf in turn before the points in that leaf are
///     // deformed. A leaf and leaf index (standard leaf traversal order) are supplied as
///     // the arguments, which matches the functor interface for LeafManager::foreach().
///     template <typename LeafNoteType>
///     void reset(LeafNoteType& leaf, size_t idx);
///
///     // Evaluate the deformer and modify the given position to generate the deformed
///     // position. An index iterator is supplied as the argument to allow querying the
///     // point offset or containing voxel coordinate.
///     template <typename IndexIterT>
///     void apply(Vec3d& position, const IndexIterT& iter) const;
/// };
/// @endcode
///
/// @note The DeformerTraits struct (defined in PointMask.h) can be used to configure
/// a deformer to evaluate in index space.

#ifndef OPENVDB_POINTS_POINT_MOVE_HAS_BEEN_INCLUDED
#define OPENVDB_POINTS_POINT_MOVE_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>

#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointMask.h>

#include <tbb/concurrent_vector.h>

#include <algorithm>
#include <iterator> // for std::begin(), std::end()
#include <map>
#include <numeric> // for std::iota()
#include <tuple>
#include <unordered_map>
#include <vector>

class TestPointMove;


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace points {

// dummy object for future use
namespace future { struct Advect { }; }


template <typename PointDataGridT, typename FilterT = NullFilter>
struct MergePoints
{
    explicit MergePoints(PointDataGridT& _points)
        : points(_points) { }

    PointDataGridT& points;
    FilterT filter = FilterT();
    bool deform = false;
}; // struct MergePoints


/// @brief Move points in a PointDataGrid using a custom deformer
/// @param points           the PointDataGrid containing the points to be moved.
/// @param deformer         a custom deformer that defines how to move the points.
/// @param filter           an optional index filter
/// @param objectNotInUse   for future use, this object is currently ignored
/// @param threaded         enable or disable threading  (threading is enabled by default)
template <typename PointDataGridT, typename DeformerT, typename FilterT = NullFilter, typename MergeFilterT = FilterT>
inline void movePoints(PointDataGridT& points,
                       DeformerT& deformer,
                       const FilterT& filter = NullFilter(),
                       std::vector<MergePoints<PointDataGridT, MergeFilterT>>* mergePoints = nullptr,
                       bool threaded = true);


/// @brief Move points in a PointDataGrid using a custom deformer and a new transform
/// @param points           the PointDataGrid containing the points to be moved.
/// @param transform        target transform to use for the resulting points.
/// @param deformer         a custom deformer that defines how to move the points.
/// @param filter           an optional index filter
/// @param objectNotInUse   for future use, this object is currently ignored
/// @param threaded         enable or disable threading  (threading is enabled by default)
template <typename PointDataGridT, typename DeformerT, typename FilterT = NullFilter, typename MergeFilterT = FilterT>
inline void movePoints(PointDataGridT& points,
                       const math::Transform& transform,
                       DeformerT& deformer,
                       const FilterT& filter = NullFilter(),
                       std::vector<MergePoints<PointDataGridT, MergeFilterT>>* mergePoints = nullptr,
                       bool threaded = true);


// define leaf index in use as 64-bit
namespace point_move_internal { using LeafIndex = Index64; }


/// @brief A Deformer that caches the resulting positions from evaluating another Deformer
template <typename T>
class CachedDeformer
{
public:
    using LeafIndex = point_move_internal::LeafIndex;
    using Vec3T = typename math::Vec3<T>;
    using LeafVecT = std::vector<Vec3T>;
    using LeafMapT = std::unordered_map<LeafIndex, Vec3T>;

    // Internal data cache to allow the deformer to offer light-weight copying
    struct Cache
    {
        struct Leaf
        {
            /// @brief clear data buffers and reset counter
            void clear() {
                vecData.clear();
                mapData.clear();
                totalSize = 0;
            }

            LeafVecT vecData;
            LeafMapT mapData;
            Index totalSize = 0;
        }; // struct Leaf

        std::vector<Leaf> leafs;
    }; // struct Cache

    /// Cache is expected to be persistent for the lifetime of the CachedDeformer
    explicit CachedDeformer(Cache& cache);

    /// Caches the result of evaluating the supplied point grid using the deformer and filter
    /// @param grid         the points to be moved
    /// @param deformer     the deformer to apply to the points
    /// @param filter       the point filter to use when evaluating the points
    /// @param threaded     enable or disable threading  (threading is enabled by default)
    template <typename PointDataGridT, typename DeformerT, typename FilterT>
    void evaluate(PointDataGridT& grid, DeformerT& deformer, const FilterT& filter,
        bool threaded = true);

    /// Stores pointers to the vector or map and optionally expands the map into a vector
    /// @throw IndexError if idx is out-of-range of the leafs in the cache
    template <typename LeafT>
    void reset(const LeafT& leaf, size_t idx);

    /// Retrieve the new position from the cache
    template <typename IndexIterT>
    void apply(Vec3d& position, const IndexIterT& iter) const;

private:
    friend class ::TestPointMove;

    Cache& mCache;
    const LeafVecT* mLeafVec = nullptr;
    const LeafMapT* mLeafMap = nullptr;
}; // class CachedDeformer


////////////////////////////////////////


namespace point_move_internal {

using LeafIndexArray = std::vector<LeafIndex>;

using GlobalIndex = std::tuple<LeafIndex, Index, Index>;
using GlobalIndexArray = tbb::concurrent_vector<GlobalIndex>;

using LocalIndex = std::pair<Index, Index>;
using LocalIndexArray = std::vector<LocalIndex>;


template <typename LeafT>
inline Index
indexOffsetFromVoxel(const Index voxelOffset, const LeafT& leaf, LeafIndexArray& offsets)
{
    // compute the target point index by summing the point index of the previous
    // voxel with the current number of points added to this voxel, tracked by the
    // offsets array

    Index targetOffset = offsets[voxelOffset]++;
    if (voxelOffset > 0) {
        targetOffset += static_cast<Index>(leaf.getValue(voxelOffset - 1));
    }
    return targetOffset;
}


// A CopyIterator is designed to use the indices in a GlobalPointIndexMap for this leaf
// and match the interface required for AttributeArray::copyValues()
template <typename LeafT>
struct GlobalCopyIterator
{
    GlobalCopyIterator(const LeafT& leaf, const LeafIndexArray& sortedIndices,
        const GlobalIndexArray& moveIndices, LeafIndexArray& offsets)
        : mLeaf(leaf)
        , mSortedIndices(sortedIndices)
        , mMoveIndices(moveIndices)
        , mOffsets(offsets) { }

    operator bool() const { return bool(mIt); }

    void reset(Index startIndex, Index endIndex)
    {
        mIndex = startIndex;
        mEndIndex = endIndex;
        this->advance();
    }

    GlobalCopyIterator& operator++()
    {
        this->advance();
        return *this;
    }

    Index leafIndex(Index i) const
    {
        if (i < mSortedIndices.size()) {
            return std::get<0>(this->leafGlobalIndex(i));
        }
        return std::numeric_limits<Index>::max();
    }

    Index sourceIndex() const
    {
        assert(mIt);
        return std::get<2>(*mIt);
    }

    Index targetIndex() const
    {
        assert(mIt);
        return indexOffsetFromVoxel(std::get<1>(*mIt), mLeaf, mOffsets);
    }

private:
    void advance()
    {
        if (mIndex >= mEndIndex || mIndex >= mSortedIndices.size()) {
            mIt = nullptr;
        }
        else {
            mIt = &this->leafGlobalIndex(mIndex);
        }
        ++mIndex;
    }

    const GlobalIndex& leafGlobalIndex(Index i) const
    {
        return mMoveIndices[mSortedIndices[i]];
    }

private:
    const LeafT& mLeaf;
    Index mIndex;
    Index mEndIndex;
    const LeafIndexArray& mSortedIndices;
    const GlobalIndexArray& mMoveIndices;
    LeafIndexArray& mOffsets;
    const GlobalIndex* mIt = nullptr;
}; // struct GlobalCopyIterator


// A LocalCopyIterator is designed to use the indices in a LocalPointIndexMap
// for this leaf and match the interface required for AttributeArray::copyValues()
template <typename LeafT>
struct LocalCopyIterator
{
    LocalCopyIterator(const LeafT& leaf, const LocalIndexArray& indices, LeafIndexArray& offsets)
        : mLeaf(leaf)
        , mIndices(indices)
        , mOffsets(offsets) { }

    operator bool() const { return mIndex < static_cast<int>(mIndices.size()); }

    LocalCopyIterator& operator++() { ++mIndex; return *this; }

    Index sourceIndex() const
    {
        return mIndices[mIndex].second;
    }

    Index targetIndex() const
    {
        return indexOffsetFromVoxel(mIndices[mIndex].first, mLeaf, mOffsets);
    }

private:
    const LeafT& mLeaf;
    const LocalIndexArray& mIndices;
    LeafIndexArray& mOffsets;
    int mIndex = 0;
}; // struct LocalCopyIterator


} // namespace point_move_internal


////////////////////////////////////////


template <typename PointDataGridT, typename DeformerT, typename FilterT, typename MergeFilterT>
inline void movePoints( PointDataGridT& points,
                        const math::Transform& transform,
                        DeformerT& deformer,
                        const FilterT& filter,
                        std::vector<MergePoints<PointDataGridT, MergeFilterT>>* mergePoints,
                        bool threaded)
{
    using LeafIndex = point_move_internal::LeafIndex;
    using PointDataTreeT = typename PointDataGridT::TreeType;
    using LeafT = typename PointDataTreeT::LeafNodeType;
    using LeafManagerT = typename tree::LeafManager<PointDataTreeT>;

    using namespace point_move_internal;

    PointDataTreeT& tree = points.tree();

    // early exit if no LeafNodes

    PointDataTree::LeafCIter iter = tree.cbeginLeaf();

    if (!iter)      return;

    // build voxel topology taking into account any point group deletion

    point_mask_internal::Pool<PointDataGrid> pool(transform);

    point_mask_internal::convertPointsToScalar<PointDataGrid>(
        pool, points, transform, filter, deformer, threaded);

    // include all merge point grids in voxel data

    if (mergePoints) {
        for (const auto& merge: *mergePoints) {
            point_mask_internal::convertPointsToScalar<PointDataGrid>(
                pool, merge.points, transform, merge.filter, deformer, threaded);
        }
    }

    // merge all pool grids into one

    auto newPoints = pool.merge(threaded);
    auto& newTree = newPoints->tree();

    // create leaf managers for both trees

    std::vector<LeafManagerT> sourceLeafManagers;

    // tree::LeafManager is not currently moveable, so reserve *must* be called

    sourceLeafManagers.reserve(1 + (mergePoints ? mergePoints->size() : 0));

    sourceLeafManagers.emplace_back(points.tree());

    if (mergePoints) {
        for (const auto& merge: *mergePoints) {
            sourceLeafManagers.emplace_back(merge.points.tree());
        }
    }

    LeafManagerT targetLeafManager(newTree);

    // extract the existing attribute set
    const auto& existingAttributeSet = points.tree().cbeginLeaf()->attributeSet();

    // build a coord -> index map for looking up target leafs by origin and a faster
    // unordered map for finding the source index from a target index

    std::unordered_map<Coord, LeafIndex> targetLeafMap;
    std::vector<LeafIndexArray> sourceIndicesArray(sourceLeafManagers.size());
    for (auto& sourceIndices : sourceIndicesArray) {
        sourceIndices.resize(targetLeafManager.leafCount(),
            std::numeric_limits<LeafIndex>::max());
    }

    std::vector<LeafIndexArray> offsetMap(targetLeafManager.leafCount());

    {
        std::vector<std::unordered_map<Coord, LeafIndex>> sourceLeafMaps(sourceLeafManagers.size());
        for (size_t i = 0; i < sourceLeafMaps.size(); i++) {
            LeafManagerT& sourceLeafManager = sourceLeafManagers[i];
            auto& sourceLeafMap = sourceLeafMaps[i];
            auto sourceRange = sourceLeafManager.leafRange();
            for (auto leaf = sourceRange.begin(); leaf; ++leaf) {
                sourceLeafMap.insert({leaf->origin(), LeafIndex(static_cast<LeafIndex>(leaf.pos()))});
            }
        }
        auto targetRange = targetLeafManager.leafRange();
        for (auto leaf = targetRange.begin(); leaf; ++leaf) {
            targetLeafMap.insert({leaf->origin(), LeafIndex(static_cast<LeafIndex>(leaf.pos()))});
        }

         // acquire registry lock to avoid locking when appending attributes in parallel

        AttributeArray::ScopedRegistryLock lock;

        // perform four independent per-leaf operations in parallel
        targetLeafManager.foreach(
            [&](LeafT& leaf, size_t idx) {
                // map frequency => cumulative histogram
                auto* buffer = leaf.buffer().data();
                for (Index i = 1; i < leaf.buffer().size(); i++) {
                    buffer[i] = buffer[i-1] + buffer[i];
                }
                // replace attribute set with a copy of the existing one
                leaf.replaceAttributeSet(
                    new AttributeSet(existingAttributeSet, leaf.getLastValue(), &lock),
                    /*allowMismatchingDescriptors=*/true);
                for (size_t i = 0; i < sourceLeafMaps.size(); i++) {
                    const auto& sourceLeafMap = sourceLeafMaps[i];
                    auto& sourceIndices = sourceIndicesArray[i];
                    // store the index of the source leaf in a corresponding target leaf array
                    const auto it = sourceLeafMap.find(leaf.origin());
                    if (it != sourceLeafMap.end()) {
                        sourceIndices[idx] = it->second;
                    }
                }
                // allocate offset maps
                offsetMap[idx].resize(LeafT::SIZE);
            },
        threaded);
    }

    // moving leaf

    std::vector<std::vector<GlobalIndexArray>> globalMoveLeafMaps(sourceLeafManagers.size());
    for (auto& globalMoveLeafMap : globalMoveLeafMaps) {
        globalMoveLeafMap.resize(targetLeafManager.leafCount());
    }
    std::vector<std::vector<LocalIndexArray>> localMoveLeafMaps(sourceLeafManagers.size());
    for (auto& localMoveLeafMap : localMoveLeafMaps) {
        localMoveLeafMap.resize(targetLeafManager.leafCount());
    }

    // build global and local move leaf maps and update local positions

    const bool useIndexSpace = DeformerTraits<DeformerT>::IndexSpace;

    auto updateMoveMaps = [&](std::vector<GlobalIndexArray>& globalMoveLeafMap, std::vector<LocalIndexArray>& localMoveLeafMap,
        const Coord& leafOrigin, const size_t leafIndex, AttributeWriteHandle<Vec3f>& sourceHandle,
        const Index offset, const Vec3d positionWS)
    {
        // transform to index-space position of target grid

        Vec3d positionIS = transform.worldToIndex(positionWS);

        // determine target voxel and offset

        Coord targetVoxel = Coord::round(positionIS);
        Index targetOffset = LeafT::coordToOffset(targetVoxel);

        // set new local position in source transform space (if point has been deformed)

        Vec3d voxelPosition(positionIS - targetVoxel.asVec3d());
        sourceHandle.set(offset, voxelPosition);

        // determine target leaf node origin and offset in the target leaf vector

        Coord targetLeafOrigin = targetVoxel & ~(LeafT::DIM - 1);
        assert(targetLeafMap.find(targetLeafOrigin) != targetLeafMap.end());
        const LeafIndex targetLeafOffset(targetLeafMap.at(targetLeafOrigin));

        // insert into move map based on whether point ends up in a new leaf node or not

        if (targetLeafOrigin == leafOrigin) {
            localMoveLeafMap[targetLeafOffset].emplace_back(targetOffset, offset);
        }
        else {
            globalMoveLeafMap[targetLeafOffset].push_back(GlobalIndex(
                LeafIndex(static_cast<LeafIndex>(leafIndex)), targetOffset, offset));
        }
    };

    // TODO: refactor using a generic lambda when C++14 is available

    for (size_t i = 0; i < sourceLeafManagers.size(); i++) {
        auto& sourceLeafManager(sourceLeafManagers[i]);
        auto& globalMoveLeafMap = globalMoveLeafMaps[i];
        auto& localMoveLeafMap = localMoveLeafMaps[i];

        sourceLeafManager.foreach(
            [&](LeafT& leaf, size_t idx) {
                DeformerT newDeformer(deformer);
                newDeformer.reset(leaf, idx);

                auto sourceHandle = AttributeWriteHandle<Vec3f>::create(leaf.attributeArray("P"));

                for (auto iter = leaf.beginIndexOn(filter); iter; iter++) {

                    // extract index-space position and apply index-space deformation (if applicable)

                    Vec3d positionIS = sourceHandle->get(*iter) + iter.getCoord().asVec3d();
                    if (useIndexSpace) {
                       newDeformer.apply(positionIS, iter);
                    }

                    // transform to world-space position and apply world-space deformation (if applicable)

                    Vec3d positionWS = points.transform().indexToWorld(positionIS);
                    if (!useIndexSpace) {
                        newDeformer.apply(positionWS, iter);
                    }

                    updateMoveMaps(globalMoveLeafMap, localMoveLeafMap,
                        leaf.origin(), idx, *sourceHandle, *iter, positionWS);
                }
            }, threaded
        );
    }

    // build a sorted index vector for each leaf that references the global move map
    // indices in order of their source leafs and voxels to ensure determinism in the
    // resulting point orders

    std::vector<std::vector<LeafIndexArray>> globalMoveLeafIndicesArray(sourceLeafManagers.size());

    targetLeafManager.foreach(
        [&](LeafT& /*leaf*/, size_t idx) {
            for (size_t i = 0; i < sourceLeafManagers.size(); i++) {
                const GlobalIndexArray& moveIndices = globalMoveLeafMaps[i][idx];
                if (moveIndices.empty())  return;

                LeafIndexArray& sortedIndices = globalMoveLeafIndicesArray[i][idx];
                sortedIndices.resize(moveIndices.size());
                std::iota(std::begin(sortedIndices), std::end(sortedIndices), 0);
                std::sort(std::begin(sortedIndices), std::end(sortedIndices),
                    [&](int i, int j)
                    {
                        const Index& indexI0(std::get<0>(moveIndices[i]));
                        const Index& indexJ0(std::get<0>(moveIndices[j]));
                        if (indexI0 < indexJ0)          return true;
                        if (indexI0 > indexJ0)          return false;
                        return std::get<2>(moveIndices[i]) < std::get<2>(moveIndices[j]);
                    }
                );
            }
        },
    threaded);

    for (const auto& it : existingAttributeSet.descriptor().map()) {

        const Index attributeIndex = static_cast<Index>(it.second);

        // zero offsets
        targetLeafManager.foreach(
            [&offsetMap](const LeafT& /*leaf*/, size_t idx) {
                std::fill(offsetMap[idx].begin(), offsetMap[idx].end(), 0);
            },
        threaded);

        // move points between leaf nodes

        targetLeafManager.foreach(
            [&](LeafT& leaf, size_t idx)
            {
                for (size_t i = 0; i < sourceLeafManagers.size(); i++) {
                    const LeafManagerT& sourceLeafManager = sourceLeafManagers[i];
                    const auto& globalMoveLeafMap = globalMoveLeafMaps[i];
                    const auto& globalMoveLeafIndices = globalMoveLeafIndicesArray[i];

                    const GlobalIndexArray& moveIndices = globalMoveLeafMap[idx];
                    if (moveIndices.empty())  return;
                    const LeafIndexArray& sortedIndices = globalMoveLeafIndices[idx];

                    // extract per-voxel offsets for this leaf

                    LeafIndexArray& offsets = offsetMap[idx];

                    // extract target array and ensure data is out-of-core and non-uniform

                    auto& targetArray = leaf.attributeArray(attributeIndex);
                    targetArray.loadData();
                    targetArray.expand();

                    // perform the copy

                    GlobalCopyIterator<LeafT> copyIterator(
                        leaf, sortedIndices, moveIndices, offsets);

                    // use the sorted indices to track the index of the source leaf

                    Index sourceLeafIndex = copyIterator.leafIndex(0);
                    Index startIndex = 0;

                    for (size_t i = 1; i <= sortedIndices.size(); i++) {
                        Index endIndex = static_cast<Index>(i);
                        Index newSourceLeafIndex = copyIterator.leafIndex(endIndex);

                        // when it changes, do a batch-copy of all the indices that lie within this range
                        // TODO: this step could use nested parallelization for cases where there are a
                        // large number of points being moved per attribute

                        if (newSourceLeafIndex > sourceLeafIndex) {
                            copyIterator.reset(startIndex, endIndex);

                            const LeafT& sourceLeaf = sourceLeafManager.leaf(sourceLeafIndex);
                            const auto& sourceArray = sourceLeaf.constAttributeArray(attributeIndex);
                            sourceArray.loadData();

                            targetArray.copyValuesUnsafe(sourceArray, copyIterator);

                            sourceLeafIndex = newSourceLeafIndex;
                            startIndex = endIndex;
                        }
                    }
                }
            }, threaded
        );

        // move points within leaf nodes

        targetLeafManager.foreach(
            [&](LeafT& leaf, size_t idx)
            {
                for (size_t i = 0; i < sourceLeafManagers.size(); i++) {
                    const auto& sourceLeafManager = sourceLeafManagers[i];
                    const auto& localMoveLeafMap = localMoveLeafMaps[i];
                    const auto& sourceIndices = sourceIndicesArray[i];

                    const LocalIndexArray& moveIndices = localMoveLeafMap[idx];
                    if (moveIndices.empty())  return;

                    // extract per-voxel offsets for this leaf

                    LeafIndexArray& offsets = offsetMap[idx];

                    // extract source array that has the same origin as the target leaf

                    assert(idx < sourceIndices.size());
                    const Index sourceLeafOffset(sourceIndices[idx]);
                    LeafT& sourceLeaf = sourceLeafManager.leaf(sourceLeafOffset);
                    const auto& sourceArray = sourceLeaf.constAttributeArray(attributeIndex);
                    sourceArray.loadData();

                    // extract target array and ensure data is out-of-core and non-uniform

                    auto& targetArray = leaf.attributeArray(attributeIndex);
                    targetArray.loadData();
                    targetArray.expand();

                    // perform the copy

                    LocalCopyIterator<LeafT> copyIterator(leaf, moveIndices, offsets);
                    targetArray.copyValuesUnsafe(sourceArray, copyIterator);
                }
            }, threaded
        );
    }

    points.setTree(newPoints->treePtr());
}


template <typename PointDataGridT, typename DeformerT, typename FilterT, typename MergeFilterT>
inline void movePoints( PointDataGridT& points,
                        DeformerT& deformer,
                        const FilterT& filter,
                        std::vector<MergePoints<PointDataGridT, MergeFilterT>>* mergePoints,
                        bool threaded)
{
    movePoints(points, points.transform(), deformer, filter, mergePoints, threaded);
}


////////////////////////////////////////


template <typename T>
CachedDeformer<T>::CachedDeformer(Cache& cache)
    : mCache(cache) { }


template <typename T>
template <typename PointDataGridT, typename DeformerT, typename FilterT>
void CachedDeformer<T>::evaluate(PointDataGridT& grid, DeformerT& deformer, const FilterT& filter,
    bool threaded)
{
    using TreeT = typename PointDataGridT::TreeType;
    using LeafT = typename TreeT::LeafNodeType;
    using LeafManagerT = typename tree::LeafManager<TreeT>;
    LeafManagerT leafManager(grid.tree());

    // initialize cache
    auto& leafs = mCache.leafs;
    leafs.resize(leafManager.leafCount());

    const auto& transform = grid.transform();

    // insert deformed positions into the cache

    auto cachePositionsOp = [&](const LeafT& leaf, size_t idx) {

        const Index64 totalPointCount = leaf.pointCount();
        if (totalPointCount == 0)   return;

        // deformer is copied to ensure that it is unique per-thread

        DeformerT newDeformer(deformer);

        newDeformer.reset(leaf, idx);

        auto handle = AttributeHandle<Vec3f>::create(leaf.constAttributeArray("P"));

        auto& cache = leafs[idx];
        cache.clear();

        // only insert into a vector directly if the filter evaluates all points
        // and all points are stored in active voxels
        const bool useVector = filter.state() == index::ALL &&
            (leaf.isDense() || (leaf.onPointCount() == leaf.pointCount()));
        if (useVector) {
            cache.vecData.resize(totalPointCount);
        }

        for (auto iter = leaf.beginIndexOn(filter); iter; iter++) {

            // extract index-space position and apply index-space deformation (if defined)

            Vec3d position = handle->get(*iter) + iter.getCoord().asVec3d();

            // if deformer is designed to be used in index-space, perform deformation prior
            // to transforming position to world-space, otherwise perform deformation afterwards

            if (DeformerTraits<DeformerT>::IndexSpace) {
                newDeformer.apply(position, iter);
                position = transform.indexToWorld(position);
            }
            else {
                position = transform.indexToWorld(position);
                newDeformer.apply(position, iter);
            }

            // insert new position into the cache

            if (useVector) {
                cache.vecData[*iter] = static_cast<Vec3T>(position);
            }
            else {
                cache.mapData.insert({*iter, static_cast<Vec3T>(position)});
            }
        }

        // store the total number of points to allow use of an expanded vector on access

        if (!cache.mapData.empty()) {
            cache.totalSize = static_cast<Index>(totalPointCount);
        }
    };

    leafManager.foreach(cachePositionsOp, threaded);
}


template <typename T>
template <typename LeafT>
void CachedDeformer<T>::reset(const LeafT& /*leaf*/, size_t idx)
{
    if (idx >= mCache.leafs.size()) {
        if (mCache.leafs.empty()) {
            throw IndexError("No leafs in cache, perhaps CachedDeformer has not been evaluated?");
        } else {
            throw IndexError("Leaf index is out-of-range of cache leafs.");
        }
    }
    auto& cache = mCache.leafs[idx];
    if (!cache.mapData.empty()) {
        mLeafMap = &cache.mapData;
        mLeafVec = nullptr;
    }
    else {
        mLeafVec = &cache.vecData;
        mLeafMap = nullptr;
    }
}


template <typename T>
template <typename IndexIterT>
void CachedDeformer<T>::apply(Vec3d& position, const IndexIterT& iter) const
{
    assert(*iter >= 0);

    if (mLeafMap) {
        auto it = mLeafMap->find(*iter);
        if (it == mLeafMap->end())      return;
        position = static_cast<openvdb::Vec3d>(it->second);
    }
    else {
        assert(mLeafVec);

        if (mLeafVec->empty())          return;
        assert(*iter < mLeafVec->size());
        position = static_cast<openvdb::Vec3d>((*mLeafVec)[*iter]);
    }
}


} // namespace points
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_POINTS_POINT_MOVE_HAS_BEEN_INCLUDED

// Copyright (c) DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
