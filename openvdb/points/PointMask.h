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

/// @file points/PointMask.h
///
/// @author Dan Bailey
///
/// @brief  Methods for extracting masks from VDB Point grids.

#ifndef OPENVDB_POINTS_POINT_MASK_HAS_BEEN_INCLUDED
#define OPENVDB_POINTS_POINT_MASK_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>
#include <openvdb/tools/ValueTransformer.h> // valxform::SumOp

#include "PointDataGrid.h"
#include "IndexFilter.h"

#include <tbb/combinable.h>

#include <type_traits>
#include <vector>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace points {


/// @brief Extract a Mask Grid from a Point Data Grid
/// @param grid         the PointDataGrid to extract the mask from.
/// @param filter       an optional index filter
/// @param threaded     enable or disable threading  (threading is enabled by default)
/// @note this method is only available for Bool Grids and Mask Grids
template <typename PointDataGridT,
          typename MaskT = typename PointDataGridT::template ValueConverter<bool>::Type,
          typename FilterT = NullFilter>
inline typename std::enable_if<std::is_same<typename MaskT::ValueType, bool>::value,
    typename MaskT::Ptr>::type
convertPointsToMask(const PointDataGridT& grid,
                    const FilterT& filter = NullFilter(),
                    bool threaded = true);


/// @brief Extract a Mask Grid from a Point Data Grid using a new transform
/// @param grid         the PointDataGrid to extract the mask from.
/// @param transform    target transform for the mask.
/// @param filter       an optional index filter
/// @param threaded     enable or disable threading  (threading is enabled by default)
/// @note this method is only available for Bool Grids and Mask Grids
template <typename PointDataGridT,
          typename MaskT = typename PointDataGridT::template ValueConverter<bool>::Type,
          typename FilterT = NullFilter>
inline typename std::enable_if<std::is_same<typename MaskT::ValueType, bool>::value,
    typename MaskT::Ptr>::type
convertPointsToMask(const PointDataGridT& grid,
                    const openvdb::math::Transform& transform,
                    const FilterT& filter = NullFilter(),
                    bool threaded = true);


/// @brief No-op deformer (adheres to the deformer interface documented in PointMove.h)
struct NullDeformer
{
    template <typename LeafT>
    void reset(LeafT&, size_t /*idx*/ = 0) { }

    template <typename IterT>
    void apply(Vec3d&, IterT&) const { }
};

/// @brief Deformer Traits for optionally configuring deformers to be applied
/// in index-space. The default is world-space.
template <typename DeformerT>
struct DeformerTraits
{
    static const bool IndexSpace = false;
};


////////////////////////////////////////


namespace point_mask_internal {


template <typename LeafT>
struct SumLeafOp
{
    using LeafType = LeafT;
    using ValueType = typename LeafT::ValueType;

    static void apply(LeafType& target, const Index offset, const ValueType& value)
    {
        target.modifyValue(offset, tools::valxform::SumOp<typename LeafT::ValueType>(value));
    }

    static void apply(LeafType& target, const LeafType& source)
    {
        for (auto iter = source.cbeginValueOn(); iter; ++iter) {
            apply(target, iter.offset(), *iter);
        }
    }
}; // struct SumLeafOp


// overload PointDataLeaf access to use setOffsetOn(), as modifyValue()
// is intentionally disabled to avoid accidental usage
template <typename T, Index Log2Dim>
struct SumLeafOp<PointDataLeafNode<T, Log2Dim>>
{
    using LeafType = PointDataLeafNode<T, Log2Dim>;
    using ValueType = typename LeafType::ValueType;

    static void apply(LeafType& target, const Index offset, const ValueType& value)
    {
        target.setOffsetOn(offset, target.getValue(offset) + value);
    }

    static void apply(LeafType& target, const LeafType& source)
    {
        for (auto iter = source.cbeginValueOn(); iter; ++iter) {
            apply(target, iter.offset(), *iter);
        }
    }
}; // struct SumLeafOp<PointDataLeafNode>


template <typename ContainerT, typename MergeLeafT>
struct MergeTreeOp
{
    using TreeT = typename ContainerT::value_type;
    using LeafT = typename TreeT::LeafNodeType;

    MergeTreeOp(ContainerT& container)
        : mContainer(container) { }

    typename TreeT::Ptr operator()(bool threaded = true)
    {
        // threading is currently disabled
        threaded = false;

        auto tree = typename TreeT::Ptr(new TreeT());

        for (auto iter = mContainer.begin(); iter != mContainer.end(); ++iter) {
            tree::LeafManager<TreeT> leafManager(*iter);
            leafManager.foreach(
                [&](LeafT& leaf, size_t /*idx*/)
                {
                    auto* newLeaf = tree->probeLeaf(leaf.origin());
                    if (!newLeaf) {
                        // if the leaf doesn't yet exist in the new tree, steal it
                        tree->addLeaf(iter->template stealNode<LeafT>(leaf.origin(),
                            zeroVal<typename LeafT::ValueType>(), false));
                    }
                    else {
                        MergeLeafT::apply(*newLeaf, leaf);
                    }
                }, threaded
            );
        }

        return tree;
    }

private:
    ContainerT& mContainer;
}; // struct MergeTreeOp


template <typename GridT,
          typename MergeLeafT = SumLeafOp<typename GridT::TreeType::LeafNodeType>,
          typename PoolT = tbb::enumerable_thread_specific<typename GridT::TreeType>>
class Pool
{
public:
    using GridType = GridT;
    using PoolType = PoolT;
    using value_type = typename PoolT::value_type;

    explicit Pool(const math::Transform& transform)
        : mTransform(transform) { }

    typename PoolT::value_type& local() { return mPool.local(); }
    typename PoolT::value_type& local(bool& exists) { return mPool.local(exists); }

    typename PoolT::size_type size() const { return mPool.size(); }

    const math::Transform& transform() { return mTransform; }

    typename GridT::Ptr merge(bool threaded = true)
    {
        MergeTreeOp<PoolT, MergeLeafT> mergeOp(mPool);
        auto tree = mergeOp(threaded);
        auto grid = GridT::create(tree);
        grid->setTransform(mTransform.copy());
        return grid;
    }

private:
    const math::Transform& mTransform;
    PoolT mPool;
}; // class Pool


template<typename GridT, typename PointDataGridT, typename FilterT>
inline typename GridT::Ptr convertPointsToScalar(
    const PointDataGridT& points,
    const FilterT& filter,
    bool threaded = true)
{
    using TreeT = typename GridT::TreeType;
    using LeafT = typename TreeT::LeafNodeType;
    using ValueT = typename LeafT::ValueType;

    auto tree = typename TreeT::Ptr(new TreeT(points.constTree(), false,
        openvdb::TopologyCopy()));

    tree::LeafManager<TreeT> leafManager(*tree);

    leafManager.foreach(
        [&](LeafT& leaf, size_t /*idx*/)
        {
            // disable all voxels and early-exit
            if (filter.state(leaf) == index::NONE) {
                leaf.setValuesOff();
                return;
            }

            const auto* const pointLeaf =
                points.constTree().probeConstLeaf(leaf.origin());

            for (auto value = leaf.beginValueOn(); value; ++value) {
                const Index64 count = points::iterCount(
                    pointLeaf->beginIndexVoxel(value.getCoord(), filter));
                if (count > Index64(0)) {
                    value.setValue(ValueT(count));
                } else {
                    // disable any empty voxels
                    value.setValueOn(false);
                }
            }
        }, threaded
    );

    auto grid = GridT::create(tree);
    grid->setTransform(points.constTransform().copy());

    return grid;
}


template<typename GridT, typename PointDataGridT, typename FilterT>
inline void convertPointsToScalar(
    Pool<GridT>& pool,
    const PointDataGridT& points,
    const FilterT& filter,
    bool threaded = true)
{
    // TODO: if pool.empty(), call convertPointsToScalar and populate first grid?

    using TreeT = typename GridT::TreeType;
    using LeafT = typename TreeT::LeafNodeType;
    using ValueT = typename LeafT::ValueType;

    tree::LeafManager<const typename PointDataGridT::TreeType> leafManager(points.constTree());

    leafManager.foreach(
        [&](const typename PointDataGridT::TreeType::LeafNodeType& pointLeaf, size_t /*idx*/)
        {
            // disable all voxels and early-exit
            if (filter.state(pointLeaf) == index::NONE) {
                return;
            }

            auto& tree = pool.local();
            auto* leaf = tree.touchLeaf(pointLeaf.origin());

            for (auto value = pointLeaf.cbeginValueOn(); value; ++value) {
                const Index64 count = points::iterCount(
                    pointLeaf.beginIndexVoxel(value.getCoord(), filter));
                SumLeafOp<LeafT>::apply(*leaf, value.offset(), ValueT(*value));
            }
        }, threaded
    );
}


template<typename GridT, typename PointDataGridT, typename FilterT, typename DeformerT>
inline void convertPointsToScalar(
    Pool<GridT>& pool,
    PointDataGridT& points,
    const openvdb::math::Transform& transform,
    const FilterT& filter,
    const DeformerT& deformer,
    bool threaded = true)
{
    // early exit if no leaves

    if (points.constTree().leafCount() == 0)   return;

    // use the simpler method if the requested transform matches the existing one

    const bool sameTransform = pool.transform() == points.constTransform();

    if (sameTransform && std::is_same<NullDeformer, DeformerT>()) {
        convertPointsToScalar<GridT>(pool, points, filter, threaded);
        return;
    }

    using TreeT = typename GridT::TreeType;
    using LeafT = typename TreeT::LeafNodeType;

    // compute mask grids in parallel using new transform

    tree::LeafManager<typename PointDataGridT::TreeType> leafManager(points.tree());

    leafManager.foreach(
        [&](typename PointDataGridT::TreeType::LeafNodeType& leaf, size_t idx)
        {
            auto& tree = pool.local();
            tree::ValueAccessor<typename GridT::TreeType> accessor(tree);

            DeformerT newDeformer(deformer);
            newDeformer.reset(leaf, idx);

            auto handle = AttributeHandle<Vec3f>::create(
                leaf.constAttributeArray("P"));

            auto updatePosition = [&](const Vec3d& position)
            {
                // determine coord of target grid

                const Coord ijk = pool.transform().worldToIndexCellCentered(position);

                // increment count in target voxel

                auto* newLeaf = accessor.touchLeaf(ijk);
                assert(newLeaf);
                SumLeafOp<LeafT>::apply(*newLeaf, newLeaf->coordToOffset(ijk),
                    typename GridT::ValueType(1));
            };

            // TODO: refactor using a generic lambda when C++14 is available

            if (filter.state() == index::ALL || filter.state(leaf) == index::ALL) {
                for (auto iter = leaf.beginIndexOn(); iter; iter++) {

                    // extract index-space position

                    Vec3d position = handle->get(*iter) + iter.getCoord().asVec3d();

                    // if deformer is designed to be used in index-space,
                    // perform deformation prior to transforming position to world-space,
                    // otherwise perform deformation afterwards

                    if (DeformerTraits<DeformerT>::IndexSpace) {
                        newDeformer.template apply<decltype(iter)>(position, iter);
                        position = points.constTransform().indexToWorld(position);
                    }
                    else {
                        position = points.constTransform().indexToWorld(position);
                        newDeformer.template apply<decltype(iter)>(position, iter);
                    }

                    updatePosition(position);
                }
            } else {
                for (auto iter = leaf.beginIndexOn(filter); iter; iter++) {

                    // extract index-space position

                    Vec3d position = handle->get(*iter) + iter.getCoord().asVec3d();

                    // if deformer is designed to be used in index-space,
                    // perform deformation prior to transforming position to world-space,
                    // otherwise perform deformation afterwards

                    if (DeformerTraits<DeformerT>::IndexSpace) {
                        newDeformer.template apply<decltype(iter)>(position, iter);
                        position = points.constTransform().indexToWorld(position);
                    }
                    else {
                        position = points.constTransform().indexToWorld(position);
                        newDeformer.template apply<decltype(iter)>(position, iter);
                    }

                    updatePosition(position);
                }
            }
        }, threaded
    );
}


template<typename GridT, typename PointDataGridT, typename FilterT, typename DeformerT>
inline typename GridT::Ptr convertPointsToScalar(
    PointDataGridT& points,
    const openvdb::math::Transform& transform,
    const FilterT& filter,
    const DeformerT& deformer,
    bool threaded = true)
{
    Pool<GridT> pool(transform);

    convertPointsToScalar(pool, points, transform, filter, deformer, threaded);

    return pool.merge(threaded);
}


} // namespace point_mask_internal


////////////////////////////////////////


template<typename PointDataGridT, typename MaskT, typename FilterT>
inline typename std::enable_if<std::is_same<typename MaskT::ValueType, bool>::value,
    typename MaskT::Ptr>::type
convertPointsToMask(
    const PointDataGridT& points,
    const FilterT& filter,
    bool threaded)
{
    return point_mask_internal::convertPointsToScalar<MaskT>(
        points, filter, threaded);
}


template<typename PointDataGridT, typename MaskT, typename FilterT>
inline typename std::enable_if<std::is_same<typename MaskT::ValueType, bool>::value,
    typename MaskT::Ptr>::type
convertPointsToMask(
    const PointDataGridT& points,
    const openvdb::math::Transform& transform,
    const FilterT& filter,
    bool threaded)
{
    // This is safe because the PointDataGrid can only be modified by the deformer
    using AdapterT = TreeAdapter<typename PointDataGridT::TreeType>;
    auto& nonConstPoints = const_cast<typename AdapterT::NonConstGridType&>(points);

    NullDeformer deformer;
    return point_mask_internal::convertPointsToScalar<MaskT>(
        nonConstPoints, transform, filter, deformer, threaded);
}


////////////////////////////////////////


// deprecated functions


template <typename PointDataGridT,
          typename MaskT = typename PointDataGridT::template ValueConverter<bool>::Type>
OPENVDB_DEPRECATED
inline typename std::enable_if<std::is_same<typename MaskT::ValueType, bool>::value,
    typename MaskT::Ptr>::type
convertPointsToMask(const PointDataGridT& grid,
                    const std::vector<Name>& includeGroups,
                    const std::vector<Name>& excludeGroups)
{
    auto leaf = grid.tree().cbeginLeaf();
    if (!leaf)  return MaskT::create();
    MultiGroupFilter filter(includeGroups, excludeGroups, leaf->attributeSet());
    return convertPointsToMask(grid, filter);
}


template <typename PointDataGridT,
          typename MaskT = typename PointDataGridT::template ValueConverter<bool>::Type>
OPENVDB_DEPRECATED
inline typename std::enable_if<std::is_same<typename MaskT::ValueType, bool>::value,
    typename MaskT::Ptr>::type
convertPointsToMask(const PointDataGridT& grid,
                    const openvdb::math::Transform& transform,
                    const std::vector<Name>& includeGroups,
                    const std::vector<Name>& excludeGroups)
{
    auto leaf = grid.tree().cbeginLeaf();
    if (!leaf)  return MaskT::create();
    MultiGroupFilter filter(includeGroups, excludeGroups, leaf->attributeSet());
    return convertPointsToMask(grid, transform, filter);
}


} // namespace points
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_POINTS_POINT_MASK_HAS_BEEN_INCLUDED

// Copyright (c) DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
