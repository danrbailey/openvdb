//
// TM & (c) Lucasfilm Entertainment Company Ltd. and Lucasfilm Ltd.
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//

#include <cppunit/extensions/HelperMacros.h>
#include <openvdb/openvdb.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointMerge.h>
#include <algorithm>
#include <string>
#include <vector>

using namespace openvdb;
using namespace openvdb::points;

class TestPointMerge: public CppUnit::TestCase
{
public:

    void setUp() override { openvdb::initialize(); }
    void tearDown() override { openvdb::uninitialize(); }

    CPPUNIT_TEST_SUITE(TestPointMerge);
    CPPUNIT_TEST(testMerge);
    CPPUNIT_TEST_SUITE_END();

    void testMerge();

}; // class TestPointMerge

PointDataGrid::Ptr
positionsToGrid(const std::vector<Vec3s>& positions, const float voxelSize = 1.0)
{
    const PointAttributeVector<Vec3s> pointList(positions);

    openvdb::math::Transform::Ptr transform(
        openvdb::math::Transform::createLinearTransform(voxelSize));

    tools::PointIndexGrid::Ptr pointIndexGrid =
        tools::createPointIndexGrid<tools::PointIndexGrid>(pointList, *transform);

    PointDataGrid::Ptr points =
            createPointDataGrid<NullCodec, PointDataGrid>(*pointIndexGrid,
                                                          pointList, *transform);

    return points;
}

std::vector<Vec3s>
gridToPositions(const PointDataGrid::Ptr& points, bool sort = true)
{
    std::vector<Vec3s> positions;

    for (auto leaf = points->tree().beginLeaf(); leaf; ++leaf) {

        const openvdb::points::AttributeArray& positionArray =
            leaf->constAttributeArray("P");
        openvdb::points::AttributeHandle<openvdb::Vec3f> positionHandle(positionArray);

        for (auto iter = leaf->beginIndexOn(); iter; ++iter) {
            openvdb::Vec3f voxelPosition = positionHandle.get(*iter);
            openvdb::Vec3d xyz = iter.getCoord().asVec3d();
            openvdb::Vec3f worldPosition = points->transform().indexToWorld(voxelPosition + xyz);

            positions.push_back(worldPosition);
        }
    }

    if (sort)   std::sort(positions.begin(), positions.end());
    return positions;
}

void
TestPointMerge::testMerge()
{
    const float voxelSize = 1.0f;

    std::vector<Vec3s> positions1 =         {
                                                {8.1, 9.2, 10.3},
                                            };

    PointDataGrid::Ptr points1 = positionsToGrid(positions1, voxelSize);

    std::vector<Vec3s> positions2 =         {
                                                {114.4, 115.3, 116.2},
                                            };

    PointDataGrid::Ptr points2 = positionsToGrid(positions2, voxelSize);

    mergePoints(*points1, *points2);

	const Index64 leafCount = points1->tree().leafCount();

    std::cerr << "Leaf Count: " << leafCount << std::endl;
    std::cerr << "Point Count: " << pointCount(points1->tree()) << std::endl;

    std::vector<Vec3s> positions = gridToPositions(points1);

    for (const auto& position: positions) {
    	std::cerr << "Point: " << position << std::endl;
    }
}



CPPUNIT_TEST_SUITE_REGISTRATION(TestPointMerge);
