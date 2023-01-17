// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

/// @file unittest/TestMotionBlur.cc

#include <openvdb/openvdb.h>
#include <openvdb/tools/MotionBlur.h>

#include <gtest/gtest.h>


class TestMotionBlur: public ::testing::Test
{
};


TEST_F(TestMotionBlur, testSingleVoxelFloat)
{
    using namespace openvdb;
    using namespace tools::blur;

    auto grid = FloatGrid::create(0.0);
    auto accessor = grid->getAccessor();
    accessor.setValue(Coord(0, 0, 0), 1.0);

    Vec3fGrid::Ptr vel = Vec3fGrid::create(Vec3f(1.0, 0.0, 0.0));
    Vec3fGrid::ConstPtr constVel = openvdb::gridConstPtrCast<Vec3fGrid>(vel);
    {
        // test motion blur with a single voxel and a velocity
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0};
        UniformWeight<false> weight(1.0);
        // test motion blur with a single voxel
        {
            auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        }
        {
            auto blurred = blurAlongStreamlines<CompositionOperator::MAX>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        }
        {
            auto blurred = blurAlongStreamlines<CompositionOperator::MIN>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 0.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        }
    }
    {
        // test motion blur with a single voxel and a velocity
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0};
        UniformWeight<true> weight(1.0);
        // test motion blur with a single voxel blur, averaged over both samples
        auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), nullptr);
        auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
        EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 0.5);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.5);
        EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
    }
}

TEST_F(TestMotionBlur, testMultiVoxel)
{
    using namespace openvdb;
    using namespace tools::blur;

    auto grid = FloatGrid::create(0.0);
    auto accessor = grid->getAccessor();
    accessor.setValue(Coord(-1, 0, 0), -0.5);
    accessor.setValue(Coord(0, 0, 0), 1.0);
    accessor.setValue(Coord(1, 0, 0), 0.5);

    auto vel = Vec3fGrid::create(Vec3f(1.0, 0.0, 0.0));
    auto constVel = openvdb::gridConstPtrCast<Vec3fGrid>(vel);
    {
        // test motion blur with a single voxel and a velocity
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0};
        UniformWeight<false> weight(1.0);
        {
            // test motion blur with a single voxel blur, averaged over both samples
            auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), -0.5);
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), -0.5 + 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 1.0 + 0.5);
            EXPECT_EQ(blurredAccessor.getValue(Coord(2, 0, 0)), 0.5);
        }
        {
            // test motion blur with a single voxel blur, averaged over both samples
            auto blurred = blurAlongStreamlines<CompositionOperator::MAX>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(2, 0, 0)), 0.5);
        }
        {
            // test motion blur with a single voxel blur, averaged over both samples
            auto blurred = blurAlongStreamlines<CompositionOperator::MIN>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), -0.5);
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), -0.5);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.0); // min of 1.0 & background value 0.0
            EXPECT_EQ(blurredAccessor.getValue(Coord(2, 0, 0)), 0.0);
        }
    }
    {
        // test motion blur with a single voxel and a velocity
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0};
        UniformWeight<true> weight(1.0);
        {
            // test motion blur with a single voxel blur, averaged over both samples
            auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), nullptr);
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), -0.25);
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), -0.25 + 0.5);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.5 + 0.25);
            EXPECT_EQ(blurredAccessor.getValue(Coord(2, 0, 0)), 0.25);
        }
    }
}

TEST_F(TestMotionBlur, testSingleVoxelFloatAccel)
{
    using namespace openvdb;
    using namespace tools::blur;

    auto grid = FloatGrid::create(0.0);
    auto accessor = grid->getAccessor();
    accessor.setValue(Coord(0, 0, 0), 1.0);

    auto vel = Vec3fGrid::create(Vec3f(1.0, 0.0, 0.0));
    auto constVel = openvdb::gridConstPtrCast<Vec3fGrid>(vel);
    auto accel = Vec3fGrid::create(Vec3f(0.0, 2.0, 0.0));
    openvdb::Vec3fGrid::ConstPtr constAccel = openvdb::gridConstPtrCast<Vec3fGrid>(accel);
    {
        // test motion blur with a single voxel and a velocity
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0};
        UniformWeight<false> weight(1.0);
        // test motion blur with a single voxel
        {
            auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), constAccel.get());
            auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
            EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 1.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
            EXPECT_EQ(blurredAccessor.getValue(Coord(1, 1, 0)), 1.0);
        }
    }
    {
        // test motion blur with a single voxel and a velocity
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0};
        UniformWeight<true> weight(1.0);
        // test motion blur with a single voxel blur, averaged over both samples
        auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), constAccel.get());
        auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
        EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 0.5);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 1, 0)), 0.5);
    }
    {
        // test motion blur with a single voxel and a velocity, multiple segments
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 2.5, 5.0};
        UniformWeight<false> weight(1.0);
        // test motion blur with a single voxel blur, uniform over all samples
        auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), nullptr);
        auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
        EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(3, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(4, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(5, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(6, 0, 0)), 0.0);
    }
    {
        // test motion blur with a single voxel and a velocity and accel, multiple segments
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0, 2.0};
        UniformWeight<false> weight(1.0);
        // test motion blur with a single voxel blur and 3 time samples with acceleration, uniform
        auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), constAccel.get());
        auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
        EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 1, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 2, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 2, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 3, 0)), 1.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 4, 0)), 1.0);

    }
    {
        // test motion blur with a single voxel and a velocity and accel, multiple segments
        BlurParms parms = BlurParms<Rand01Subsampler>();
        parms.timesamples = {0.0, 1.0, 2.0};
        UniformWeight<true> weight(1.0);
        // test motion blur with a single voxel blur and 3 timesamples acceleration, averaged over all samples
        auto blurred = blurAlongStreamlines<CompositionOperator::ADD>(*grid, weight, parms, constVel.get(), constAccel.get());
        auto blurredAccessor = openvdb::gridPtrCast<FloatGrid>(blurred)->getConstAccessor();
        EXPECT_EQ(blurredAccessor.getValue(Coord(0, 0, 0)), 0.2f);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(-1, 0, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 1, 0)), 0.2f);
        EXPECT_EQ(blurredAccessor.getValue(Coord(1, 2, 0)), 0.2f);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 2, 0)), 0.0);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 3, 0)), 0.2f);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 4, 0)), 0.2f);
        EXPECT_EQ(blurredAccessor.getValue(Coord(2, 5, 0)), 0.0);
    }
}
