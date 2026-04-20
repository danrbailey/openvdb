// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#include <openvdb/io/Codec.h>
#include <openvdb/io/File.h>
#include <openvdb/openvdb.h>
#include <openvdb/Exceptions.h>
#include <openvdb/tools/Clip.h>
#include <gtest/gtest.h>

namespace {

struct MockCodec : public openvdb::io::Codec
{
    static std::string name() { return "mock"; }

    openvdb::io::CodecData::Ptr createData() final { return nullptr; }
};

class TestCodec: public ::testing::Test
{
};

} // unnamed namespace


TEST_F(TestCodec, testCodecRegistry)
{
    using namespace openvdb::io;

    // Start clean
    CodecRegistry::clear();

    // Test isRegistered on empty registry
    EXPECT_FALSE(CodecRegistry::isRegistered("mock"));

    // Test registerCodecByName
    EXPECT_NO_THROW(
        CodecRegistry::registerCodecByName("mock", std::make_unique<MockCodec>())
    );

    EXPECT_TRUE(CodecRegistry::isRegistered("mock"));
    EXPECT_FALSE(CodecRegistry::isRegistered("nonexistent"));

    // Test duplicate registration throws KeyError
    EXPECT_THROW(
        CodecRegistry::registerCodecByName("mock", std::make_unique<MockCodec>()),
        openvdb::KeyError
    );

    // Test registerCodec template form also throws on duplicate
    EXPECT_THROW(
        CodecRegistry::registerCodec<MockCodec>(),
        openvdb::KeyError
    );

    // Test get
    EXPECT_NE(CodecRegistry::get("mock"), nullptr);
    EXPECT_EQ(CodecRegistry::get("nonexistent"), nullptr);

    // Test clear
    CodecRegistry::clear();
    EXPECT_FALSE(CodecRegistry::isRegistered("mock"));
    EXPECT_NO_THROW(CodecRegistry::clear());  // Clear on empty registry

    // Test registerCodec template form on fresh registry
    EXPECT_NO_THROW(CodecRegistry::registerCodec<MockCodec>());
    EXPECT_TRUE(CodecRegistry::isRegistered("mock"));

    // Test io::initialize and io::uninitialize
    CodecRegistry::clear();
    EXPECT_FALSE(CodecRegistry::isRegistered(openvdb::BoolGrid::gridType()));

    EXPECT_NO_THROW(internal::initialize());
    EXPECT_TRUE(CodecRegistry::isRegistered(openvdb::BoolGrid::gridType()));

    EXPECT_NO_THROW(internal::uninitialize());
    EXPECT_FALSE(CodecRegistry::isRegistered(openvdb::BoolGrid::gridType()));
}


TEST_F(TestCodec, testReadDiagnostics)
{
    using namespace openvdb;
    using namespace openvdb::io;

    // ReadDiagnostics struct: disabled by default, addWarning is a no-op until enabled
    {
        ReadDiagnostics diags;
        EXPECT_FALSE(diags.enabled());
        diags.addWarning("grid_a", "something went wrong");
        EXPECT_TRUE(diags.diagnostics().empty());

        diags.enable();
        diags.addWarning("grid_a", "something went wrong");
        ASSERT_EQ(diags.diagnostics().size(), size_t(1));
        EXPECT_EQ(diags.diagnostics()[0].severity, DiagnosticSeverity::Warning);

        diags.clear();
        EXPECT_TRUE(diags.diagnostics().empty());
    }

    // Archive API and getGrids() with diagnostics
    openvdb::initialize();

    BoolGrid::Ptr srcGrid = BoolGrid::create(false);
    srcGrid->setName("bool_grid");
    srcGrid->fill(CoordBBox(Coord(-5), Coord(5)), true, true);

    const std::string codecPath = "testReadDiagnostics.vdb";
    {
        io::File f(codecPath);
        f.write(GridPtrVec{srcGrid});
    }

    // Disabled by default; enabling produces no warnings on a clean read
    {
        io::File f(codecPath);
        f.open();
        EXPECT_FALSE(f.readDiagnostics().enabled());
        f.enableReadDiagnostics();
        EXPECT_TRUE(f.readDiagnostics().enabled());
        f.readGrid("bool_grid");
        EXPECT_TRUE(f.readDiagnostics().diagnostics().empty());
        f.close();
    }

    // clearReadDiagnostics() resets entries but keeps diagnostics enabled
    {
        io::File f(codecPath);
        f.open();
        f.enableReadDiagnostics();
        GridPtrVecPtr grids = f.getGrids();
        ASSERT_TRUE(grids && !grids->empty());
        f.clearReadDiagnostics();
        EXPECT_TRUE(f.readDiagnostics().enabled());
        EXPECT_TRUE(f.readDiagnostics().diagnostics().empty());
        f.close();
    }

    std::remove(codecPath.c_str());
}


template <typename GridT>
void testIOImpl(
    const std::string& gridName,
    const typename GridT::ValueType& bgValue,
    const typename GridT::ValueType& fillValue)
{
    using namespace openvdb;
    using namespace openvdb::io;

    const bool codecRegistered = CodecRegistry::isRegistered(GridT::gridType());
    std::cerr << "[testIOImpl] gridName=" << gridName
              << " gridType=" << GridT::gridType()
              << " codecRegistered=" << codecRegistered << "\n";

    typename GridT::Ptr srcGrid = GridT::create(bgValue);
    srcGrid->setName(gridName);
    srcGrid->fill(CoordBBox(Coord(-5), Coord(5)), fillValue, true);
    std::cerr << "[testIOImpl] srcGrid created: activeVoxelCount="
              << srcGrid->activeVoxelCount()
              << " leafCount=" << srcGrid->tree().leafCount() << "\n";

    std::stringstream ss("test");
    if (codecRegistered) {
        ss << "_codec";
    } else {
        ss << "_tree";
    }
    ss << "_" << GridT::gridType() << ".vdb";
    const std::string path = ss.str();
    std::cerr << "[testIOImpl] writing to path=" << path << "\n";
    {
        io::File f(path);
        f.write(GridPtrVec{srcGrid});
    }
    std::cerr << "[testIOImpl] write complete\n";

    typename GridT::Ptr readGrid;
    {
        std::cerr << "[testIOImpl] opening for full read\n";
        io::File f(path);
        f.open();
        readGrid = gridPtrCast<GridT>(f.readGrid(gridName));
        f.close();
    }
    std::cerr << "[testIOImpl] full read complete: readGrid="
              << (readGrid ? "non-null" : "null") << "\n";
    ASSERT_TRUE(readGrid);
    std::cerr << "[testIOImpl] checking topology match\n";
    EXPECT_TRUE(srcGrid->tree().hasSameTopology(readGrid->tree()));
    {
        auto readAcc = readGrid->getConstAccessor();
        Index64 checkedVoxels = 0;
        for (typename GridT::ValueOnCIter it = srcGrid->cbeginValueOn(); it; ++it) {
            EXPECT_EQ(*it, readAcc.getValue(it.getCoord()));
            ++checkedVoxels;
        }
        std::cerr << "[testIOImpl] full read value check done: checkedVoxels="
                  << checkedVoxels << "\n";
    }

    // clip read
    const BBoxd clipBBox(Vec3d(0.0), Vec3d(3.5));
    std::cerr << "[testIOImpl] clipping srcGrid with bbox ["
              << clipBBox.min() << ", " << clipBBox.max() << "]\n";
    auto srcClipped = tools::clip(*srcGrid, clipBBox);
    std::cerr << "[testIOImpl] srcClipped: activeVoxelCount="
              << srcClipped->activeVoxelCount()
              << " leafCount=" << srcClipped->tree().leafCount() << "\n";

    typename GridT::Ptr readClipped;
    {
        std::cerr << "[testIOImpl] opening for clip read\n";
        io::File f(path);
        f.open();
        readClipped = gridPtrCast<GridT>(f.readGrid(gridName, clipBBox));
        f.close();
    }
    std::cerr << "[testIOImpl] clip read complete: readClipped="
              << (readClipped ? "non-null" : "null") << "\n";
    ASSERT_TRUE(readClipped);
    std::cerr << "[testIOImpl] readClipped: activeVoxelCount="
              << readClipped->activeVoxelCount()
              << " leafCount=" << readClipped->tree().leafCount() << "\n";
    std::cerr << "[testIOImpl] checking clipped topology match\n";
    EXPECT_TRUE(srcClipped->tree().hasSameTopology(readClipped->tree()));
    {
        auto readAcc = readClipped->getConstAccessor();
        Index64 checkedVoxels = 0;
        for (typename GridT::ValueOnCIter it = srcClipped->cbeginValueOn(); it; ++it) {
            EXPECT_EQ(*it, readAcc.getValue(it.getCoord()));
            ++checkedVoxels;
        }
        std::cerr << "[testIOImpl] clip read value check done: checkedVoxels="
                  << checkedVoxels << "\n";
    }

    // topology-only read
    ReadOptions topoOpts;
    topoOpts.readMode = ReadMode::TopologyOnly;

    typename GridT::Ptr readTopo;
    {
        std::cerr << "[testIOImpl] opening for topology-only read\n";
        io::File f(path);
        f.open();
        GridBase::Ptr base;
        EXPECT_NO_THROW(base = f.readGrid(gridName, topoOpts));
        readTopo = gridPtrCast<GridT>(base);
        f.close();
    }
    std::cerr << "[testIOImpl] topology-only read complete: readTopo="
              << (readTopo ? "non-null" : "null") << "\n";
    ASSERT_TRUE(readTopo);
    std::cerr << "[testIOImpl] readTopo: activeVoxelCount="
              << readTopo->activeVoxelCount()
              << " leafCount=" << readTopo->tree().leafCount()
              << " name=" << readTopo->getName() << "\n";
    EXPECT_EQ(readTopo->activeVoxelCount(), Index64(0));
    EXPECT_TRUE(readTopo->tree().leafCount() == 0);
    EXPECT_EQ(readTopo->getName(), gridName);

    // Cleanup
    std::cerr << "[testIOImpl] cleanup: removing " << path << "\n";
    std::remove(path.c_str());
    std::cerr << "[testIOImpl] done\n";
}

template <typename GridT>
void testCodecIOImpl(
    const std::string& gridName,
    const typename GridT::ValueType& bgValue,
    const typename GridT::ValueType& fillValue)
{
    // initialize to register all the grids and codecs
    openvdb::initialize();
    // ensure the codec is registered
    ASSERT_TRUE(openvdb::io::CodecRegistry::isRegistered(GridT::gridType()));
    // test the io implementation (codec)
    testIOImpl<GridT>(gridName, bgValue, fillValue);
    // clear the codec registry (now read/write falls back to Tree I/O)
    openvdb::io::CodecRegistry::clear();
    // ensure the codec is not registered
    ASSERT_FALSE(openvdb::io::CodecRegistry::isRegistered(GridT::gridType()));
    // test the io implementation (tree I/O)
    testIOImpl<GridT>(gridName, bgValue, fillValue);
}

TEST_F(TestCodec, testBoolCodecIO) { testCodecIOImpl<openvdb::BoolGrid>("bool_grid", false, true); }
