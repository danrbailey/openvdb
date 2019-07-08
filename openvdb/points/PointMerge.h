//
// TM & (c) Lucasfilm Entertainment Company Ltd. and Lucasfilm Ltd.
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//

#ifndef OPENVDB_POINTS_POINT_MERGE_HAS_BEEN_INCLUDED
#define OPENVDB_POINTS_POINT_MERGE_HAS_BEEN_INCLUDED

#include <openvdb/openvdb.h>

#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointMove.h>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace points {


template <typename PointDataGridT, typename FilterT = NullFilter>
inline void mergePoints(PointDataGridT& pointsA,
                        PointDataGridT& pointsB,
                        const FilterT& filterB = NullFilter(),
                        bool threaded = true)
{
    std::vector<MergePoints<PointDataGrid>> pointsToMerge;
    pointsToMerge.reserve(1);
    pointsToMerge.emplace_back(pointsB);

    NullDeformer nullDeformer;
    NullFilter nullFilter;
    movePoints(pointsA, pointsA.transform(), nullDeformer, nullFilter, &pointsToMerge);
}


} // namespace points
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_POINTS_POINT_MERGE_HAS_BEEN_INCLUDED
