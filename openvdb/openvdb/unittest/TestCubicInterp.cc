// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: MPL-2.0

#include "gtest/gtest.h"
#include <openvdb/Exceptions.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>

namespace {
// Absolute tolerance for floating-point equality comparisons
const double TOLERANCE = 1.e-4;
}

class TestCubicInterp: public ::testing::Test
{
public:
    template<typename GridType>
    void test();
};


template<typename GridType>
void
TestCubicInterp::test()
{
    typename GridType::TreeType TreeType;
    float fillValue = 256.0f;

    GridType grid(fillValue);
    typename GridType::TreeType& tree = grid.tree();

    tree.setValue(openvdb::Coord(10, 10, 9), 2.0);
    tree.setValue(openvdb::Coord(11, 10, 9), 2.0);
    tree.setValue(openvdb::Coord(11, 11, 9), 2.0);
    tree.setValue(openvdb::Coord(10, 11, 9), 2.0);
    tree.setValue(openvdb::Coord( 9, 11, 9), 2.0);
    tree.setValue(openvdb::Coord( 9, 10, 9), 2.0);
    tree.setValue(openvdb::Coord( 9,  9, 9), 2.0);
    tree.setValue(openvdb::Coord(10,  9, 9), 2.0);
    tree.setValue(openvdb::Coord(11,  9, 9), 2.0);
    tree.setValue(openvdb::Coord(12,  9, 9), 2.0);
    tree.setValue(openvdb::Coord(12, 10, 9), 2.0);
    tree.setValue(openvdb::Coord(12, 11, 9), 2.0);
    tree.setValue(openvdb::Coord(12, 12, 9), 2.0);
    tree.setValue(openvdb::Coord(11, 12, 9), 2.0);
    tree.setValue(openvdb::Coord(10, 12, 9), 2.0);
    tree.setValue(openvdb::Coord( 9, 12, 9), 2.0);
    tree.setValue(openvdb::Coord( 9, 11, 9), 2.0);
    tree.setValue(openvdb::Coord( 9, 10, 9), 2.0);

    tree.setValue(openvdb::Coord(10, 10, 10), 1.0);

    tree.setValue(openvdb::Coord(11, 10, 10), 3.0);
    tree.setValue(openvdb::Coord(11, 11, 10), 3.0);
    tree.setValue(openvdb::Coord(10, 11, 10), 3.0);
    tree.setValue(openvdb::Coord( 9, 11, 10), 3.0);
    tree.setValue(openvdb::Coord( 9, 10, 10), 3.0);
    tree.setValue(openvdb::Coord( 9,  9, 10), 3.0);
    tree.setValue(openvdb::Coord(10,  9, 10), 3.0);
    tree.setValue(openvdb::Coord(11,  9, 10), 3.0);
    tree.setValue(openvdb::Coord(12,  9, 10), 3.0);
    tree.setValue(openvdb::Coord(12, 10, 10), 3.0);
    tree.setValue(openvdb::Coord(12, 11, 10), 3.0);
    tree.setValue(openvdb::Coord(12, 12, 10), 3.0);
    tree.setValue(openvdb::Coord(11, 12, 10), 3.0);
    tree.setValue(openvdb::Coord(10, 12, 10), 3.0);
    tree.setValue(openvdb::Coord( 9, 12, 10), 3.0);
    tree.setValue(openvdb::Coord( 9, 11, 10), 3.0);
    tree.setValue(openvdb::Coord( 9, 10, 10), 3.0);

    tree.setValue(openvdb::Coord(10, 10, 11), 4.0);
    tree.setValue(openvdb::Coord(11, 10, 11), 4.0);
    tree.setValue(openvdb::Coord(11, 11, 11), 4.0);
    tree.setValue(openvdb::Coord(10, 11, 11), 4.0);
    tree.setValue(openvdb::Coord( 9, 11, 11), 4.0);
    tree.setValue(openvdb::Coord( 9, 10, 11), 4.0);
    tree.setValue(openvdb::Coord( 9,  9, 11), 4.0);
    tree.setValue(openvdb::Coord(10,  9, 11), 4.0);
    tree.setValue(openvdb::Coord(11,  9, 11), 4.0);
    tree.setValue(openvdb::Coord(12,  9, 11), 4.0);
    tree.setValue(openvdb::Coord(12, 10, 11), 4.0);
    tree.setValue(openvdb::Coord(12, 11, 11), 4.0);
    tree.setValue(openvdb::Coord(12, 12, 11), 4.0);
    tree.setValue(openvdb::Coord(11, 12, 11), 4.0);
    tree.setValue(openvdb::Coord(10, 12, 11), 4.0);
    tree.setValue(openvdb::Coord( 9, 12, 11), 4.0);
    tree.setValue(openvdb::Coord( 9, 11, 11), 4.0);
    tree.setValue(openvdb::Coord( 9, 10, 11), 4.0);

    tree.setValue(openvdb::Coord(10, 10, 12), 5.0);
    tree.setValue(openvdb::Coord(11, 10, 12), 5.0);
    tree.setValue(openvdb::Coord(11, 11, 12), 5.0);
    tree.setValue(openvdb::Coord(10, 11, 12), 5.0);
    tree.setValue(openvdb::Coord( 9, 11, 12), 5.0);
    tree.setValue(openvdb::Coord( 9, 10, 12), 5.0);
    tree.setValue(openvdb::Coord( 9,  9, 12), 5.0);
    tree.setValue(openvdb::Coord(10,  9, 12), 5.0);
    tree.setValue(openvdb::Coord(11,  9, 12), 5.0);
    tree.setValue(openvdb::Coord(12,  9, 12), 5.0);
    tree.setValue(openvdb::Coord(12, 10, 12), 5.0);
    tree.setValue(openvdb::Coord(12, 11, 12), 5.0);
    tree.setValue(openvdb::Coord(12, 12, 12), 5.0);
    tree.setValue(openvdb::Coord(11, 12, 12), 5.0);
    tree.setValue(openvdb::Coord(10, 12, 12), 5.0);
    tree.setValue(openvdb::Coord( 9, 12, 12), 5.0);
    tree.setValue(openvdb::Coord( 9, 11, 12), 5.0);
    tree.setValue(openvdb::Coord( 9, 10, 12), 5.0);

    {//using TricubicSampler

        // transform used for worldspace interpolation)
        openvdb::tools::GridSampler<GridType, openvdb::tools::CubicSampler>
            interpolator(grid);

        // GNU Octave has been used to validate the sampler's results.
        // where x,y,z are the difference between the coordinates of the position to sample and 10,10,10
        // and a,b,c,d are the values on the row (x,10,10), e.g.: 3.0, 1.0, 3.0, 3.0 in this case
        // e = (-(1/2)*a+(3/2)*b-(3/2)*c+(1/2)*d)*x^3+(a-(5/2)*b+2*c-(1/2)*d)*x^2+(-(1/2)*a+(1/2)*c)*x+b
        // g = (-(1/2)*a+(3/2)*e-(3/2)*c+(1/2)*d)*y^3+(a-(5/2)*e+2*c-(1/2)*d)*y^2+(-(1/2)*a+(1/2)*c)*y+e
        // This is the sample value:
        // (-(1/2)*f+(3/2)*g-(3/2)*h+(1/2)*i)*z^3+(f-(5/2)*g+2*h-(1/2)*i)*z^2+(-(1/2)*f+(1/2)*h)*z+g

        typename GridType::ValueType val =
            interpolator.sampleVoxel(10.5, 10.5, 10.5);
        EXPECT_NEAR(3.1441, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.0, 10.0, 10.0);
        EXPECT_NEAR(1.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(11.0, 10.0, 10.0);
        EXPECT_NEAR(3.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(11.0, 11.0, 10.0);
        EXPECT_NEAR(3.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(11.0, 11.0, 11.0);
        EXPECT_NEAR(4.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(9.0, 11.0, 9.0);
        EXPECT_NEAR(2.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(9.0, 10.0, 9.0);
        EXPECT_NEAR(2.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.1, 10.0, 10.0);
        EXPECT_NEAR(1.0470, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.8, 10.8, 10.8);
        EXPECT_NEAR(3.7905, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.1, 10.8, 10.5);
        EXPECT_NEAR(3.3154, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.8, 10.1, 10.5);
        EXPECT_NEAR(3.3154, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.5, 10.1, 10.8);
        EXPECT_NEAR(3.6154, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.5, 10.8, 10.1);
        EXPECT_NEAR(2.9154, val, TOLERANCE);

    }
    {//using Sampler<3>

        // transform used for worldspace interpolation)
        openvdb::tools::GridSampler<GridType, openvdb::tools::Sampler<3> >
            interpolator(grid);

        typename GridType::ValueType val =
            interpolator.sampleVoxel(10.5, 10.5, 10.5);
        EXPECT_NEAR(3.1441, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.0, 10.0, 10.0);
        EXPECT_NEAR(1.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(11.0, 10.0, 10.0);
        EXPECT_NEAR(3.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(11.0, 11.0, 10.0);
        EXPECT_NEAR(3.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(11.0, 11.0, 11.0);
        EXPECT_NEAR(4.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(9.0, 11.0, 9.0);
        EXPECT_NEAR(2.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(9.0, 10.0, 9.0);
        EXPECT_NEAR(2.0, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.1, 10.0, 10.0);
        EXPECT_NEAR(1.0470, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.8, 10.8, 10.8);
        EXPECT_NEAR(3.7905, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.1, 10.8, 10.5);
        EXPECT_NEAR(3.3154, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.8, 10.1, 10.5);
        EXPECT_NEAR(3.3154, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.5, 10.1, 10.8);
        EXPECT_NEAR(3.6154, val, TOLERANCE);

        val = interpolator.sampleVoxel(10.5, 10.8, 10.1);
        EXPECT_NEAR(2.9154, val, TOLERANCE);
    }
}
TEST_F(TestCubicInterp, testFloat) { test<openvdb::FloatGrid>(); }
TEST_F(TestCubicInterp, testDouble) { test<openvdb::DoubleGrid>(); }

TEST_F(TestCubicInterp, testVec3S)
{
    using namespace openvdb;

    Vec3s fillValue = Vec3s(256.0f, 256.0f, 256.0f);

    Vec3SGrid grid(fillValue);
    Vec3STree& tree = grid.tree();

    tree.setValue(openvdb::Coord(10, 10,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(11, 10,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(11, 11,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(10, 11,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord( 9, 11,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord( 9, 10,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord( 9,  9,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(10,  9,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(11,  9,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(12,  9,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(12, 10,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(12, 11,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(12, 12,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(11, 12,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord(10, 12,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord( 9, 12,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord( 9, 11,  9), Vec3s(2.0, 2.0, 2.0));
    tree.setValue(openvdb::Coord( 9, 10,  9), Vec3s(2.0, 2.0, 2.0));

    tree.setValue(openvdb::Coord(10, 10, 10), Vec3s(1.0, 1.0, 1.0));

    tree.setValue(openvdb::Coord(11, 10, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(11, 11, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(10, 11, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord( 9, 11, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord( 9, 10, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord( 9,  9, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(10,  9, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(11,  9, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(12,  9, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(12, 10, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(12, 11, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(12, 12, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(11, 12, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord(10, 12, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord( 9, 12, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord( 9, 11, 10), Vec3s(3.0, 3.0, 3.0));
    tree.setValue(openvdb::Coord( 9, 10, 10), Vec3s(3.0, 3.0, 3.0));

    tree.setValue(openvdb::Coord(10, 10, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(11, 10, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(11, 11, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(10, 11, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord( 9, 11, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord( 9, 10, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord( 9,  9, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(10,  9, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(11,  9, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(12,  9, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(12, 10, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(12, 11, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(12, 12, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(11, 12, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord(10, 12, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord( 9, 12, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord( 9, 11, 11), Vec3s(4.0, 4.0, 4.0));
    tree.setValue(openvdb::Coord( 9, 10, 11), Vec3s(4.0, 4.0, 4.0));

    tree.setValue(openvdb::Coord(10, 10, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(11, 10, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(11, 11, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(10, 11, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord( 9, 11, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord( 9, 10, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord( 9,  9, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(10,  9, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(11,  9, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(12,  9, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(12, 10, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(12, 11, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(12, 12, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(11, 12, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord(10, 12, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord( 9, 12, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord( 9, 11, 12), Vec3s(5.0, 5.0, 5.0));
    tree.setValue(openvdb::Coord( 9, 10, 12), Vec3s(5.0, 5.0, 5.0));


    openvdb::tools::GridSampler<Vec3SGrid, openvdb::tools::CubicSampler>
        interpolator(grid);

    Vec3SGrid::ValueType val = interpolator.sampleVoxel(10.5, 10.5, 10.5);
    std::cout << val << std::endl;
    EXPECT_TRUE(val.eq(Vec3s(3.144f), TOLERANCE));

    val = interpolator.sampleVoxel(10.0, 10.0, 10.0);
    EXPECT_TRUE(val.eq(Vec3s(1.f), TOLERANCE));

    val = interpolator.sampleVoxel(11.0, 10.0, 10.0);
    EXPECT_TRUE(val.eq(Vec3s(3.f), TOLERANCE));

    val = interpolator.sampleVoxel(11.0, 11.0, 10.0);
    EXPECT_TRUE(val.eq(Vec3s(3.f), TOLERANCE));

    val = interpolator.sampleVoxel(11.0, 11.0, 11.0);
    EXPECT_TRUE(val.eq(Vec3s(4.f), TOLERANCE));

    val = interpolator.sampleVoxel(9.0, 11.0, 9.0);
    EXPECT_TRUE(val.eq(Vec3s(2.f), TOLERANCE));

    val = interpolator.sampleVoxel(9.0, 10.0, 9.0);
    EXPECT_TRUE(val.eq(Vec3s(2.f), TOLERANCE));

    val = interpolator.sampleVoxel(10.1, 10.0, 10.0);
    EXPECT_TRUE(val.eq(Vec3s(1.0470f), TOLERANCE));

    val = interpolator.sampleVoxel(10.8, 10.8, 10.8);
    EXPECT_TRUE(val.eq(Vec3s(3.7905f), TOLERANCE));

    val = interpolator.sampleVoxel(10.1, 10.8, 10.5);
    EXPECT_TRUE(val.eq(Vec3s(3.3154f), TOLERANCE));

    val = interpolator.sampleVoxel(10.8, 10.1, 10.5);
    EXPECT_TRUE(val.eq(Vec3s(3.3154f), TOLERANCE));

    val = interpolator.sampleVoxel(10.5, 10.1, 10.8);
    EXPECT_TRUE(val.eq(Vec3s(3.6154f), TOLERANCE));

    val = interpolator.sampleVoxel(10.5, 10.8, 10.1);
    EXPECT_TRUE(val.eq(Vec3s(2.9154f), TOLERANCE));
}