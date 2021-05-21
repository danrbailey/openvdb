
#include <openvdb/openvdb.h>

#include <openvdb/util/CpuTimer.h>

#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tree/LeafManager.h>

#include <tbb/global_control.h>

using namespace openvdb;

struct DoubleOp
{
    template <typename T>
    bool operator()(T&, size_t = 0) const { return true; }

    bool operator()(FloatTree::LeafNodeType& leaf, size_t idx = 0) const
    {
        for (auto iter = leaf.beginValueOn(); iter; ++iter) {
            iter.setValue(iter.getValue() * 2);
        }
        return true;
    }
};

int
main(int argc, char *argv[])
{
    openvdb::initialize();

    util::CpuTimer timer;

    size_t count = 100*1000*1000;

    std::vector<Coord> ijks;
    ijks.reserve(count*8);

    auto openCloud = [&]() -> FloatTree
    {
        io::File file("wdas_cloud.vdb");
        file.open();
        auto grids = file.getGrids();
        file.close();
        auto gridBase = (*grids)[0];
        auto grid = GridBase::grid<FloatGrid>(gridBase);
        FloatTree tree(grid->tree());
        tree.voxelizeActiveTiles();
        return tree;
    };

    auto addCoalescedIJKs = [&]()
    {
        ijks.clear();

        for (int tileIndex = 0; tileIndex < 8; tileIndex++) {
            Int32 i(0);
            Int32 j(0);
            Int32 k(0);
            if ((tileIndex & 1) == 1)   i -= 4096;
            if ((tileIndex & 2) == 1)   j -= 4096;
            if ((tileIndex & 4) == 1)   k -= 4096;
            for (int n = 0; n < count; n++) {
                ijks.emplace_back(i, j, k);
            }
        }
    };

    auto addInterleavedIJKs = [&]()
    {
        ijks.clear();

        for (int n = 0; n < count; n++) {
            for (int tileIndex = 0; tileIndex < 8; tileIndex++) {
                Int32 i(0);
                Int32 j(0);
                Int32 k(0);
                if ((tileIndex & 1) == 1)   i -= 4096;
                if ((tileIndex & 2) == 1)   j -= 4096;
                if ((tileIndex & 4) == 1)   k -= 4096;
                ijks.emplace_back(i, j, k);
            }
        }
    };

    auto addSequentialVoxelIJKs = [&](auto& tree)
    {
        ijks.clear();

        auto& root = tree.root();

        for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
            for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                ijks.emplace_back(iter.getCoord());
            }
        }
    };

    auto addInterleavedVoxelIJKs = [&](auto& tree)
    {
        ijks.clear();

        auto& root = tree.root();

        std::vector<std::vector<Coord>> ijksRootChildren;
        ijksRootChildren.resize(root.childCount());

        size_t i = 0;
        for (auto iter1 = tree.cbeginRootChildren(); iter1; ++iter1) {
            auto& ijksRootChild = ijksRootChildren[i++];
            for (auto iter2 = iter1->cbeginChildOn(); iter2; ++iter2) {
                for (auto iter3 = iter2->cbeginChildOn(); iter3; ++iter3) {
                    for (auto iter4 = iter3->cbeginValueOn(); iter4; ++iter4) {
                        ijksRootChild.push_back(iter3.getCoord());
                    }
                }
            }
        }

        size_t maxSize = 0;
        for (const auto& ijksRootChild : ijksRootChildren) {
            maxSize = std::max(maxSize, ijksRootChild.size());
        }

        for (i = 0; i < maxSize; i++) {
            for (const auto& ijksRootChild : ijksRootChildren) {
                if (i < ijksRootChild.size()) {
                    ijks.push_back(ijksRootChild[i]);
                }
            }
        }
    };

    auto addOneTile = [&](auto& tree)
    {
        tree.addTile(0, Coord(0, 0, 0), 1.0f, true);
    };

    auto addEightTiles = [&](auto& tree)
    {
        for (int i = -4096; i <= 0; i += 4096) {
            for (int j = -4096; j <= 0; j += 4096) {
                for (int k = -4096; k <= 0; k += 4096) {
                    tree.addTile(0, Coord(i, j, k), 1.0f, true);
                }
            }
        }
    };

    auto addSixtyFourTiles = [&](auto& tree)
    {
        for (int i = -(4096*2); i <= 4096; i += 4096) {
            for (int j = -(4096*2); j <= 4096; j += 4096) {
                for (int k = -(4096*2); k <= 4096; k += 4096) {
                    tree.addTile(0, Coord(i, j, k), 1.0f, true);
                }
            }
        }
    };

    auto rootQueryDirect = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        int total = 0;

        { // warm up
            for (const auto& ijk : ijks) {
                total += root.getValueDepth(ijk);
            }
        }

        double time = 0.0f;

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (const auto& ijk : ijks) {
                total += root.getValueDepth(ijk);
            }

            time += timer.milliseconds();

            if (total == 0)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto rootQueryAccessor = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        int total = 0;

        { // warm up
            tree::ValueAccessor<FloatTree> valueAccessor(tree);
            for (const auto& ijk : ijks) {
                total += valueAccessor.getValueDepth(ijk);
            }
        }

        double time = 0.0f;

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::ValueAccessor<FloatTree> valueAccessor(tree);

            for (const auto& ijk : ijks) {
                total += valueAccessor.getValueDepth(ijk);
            }

            time += timer.milliseconds();

            if (total == 0)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto getValueSequential = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
                for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                    total += iter.getValue();
                }
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
                for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                    total += iter.getValue();
                }
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto getValueSequential2 = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter1 = tree.cbeginRootChildren(); iter1; ++iter1) {
                for (auto iter2 = iter1->cbeginChildOn(); iter2; ++iter2) {
                    for (auto iter3 = iter2->cbeginChildOn(); iter3; ++iter3) {
                        for (auto iter4 = iter3->cbeginValueOn(); iter4; ++iter4) {
                            total += iter4.getValue();
                        }
                    }
                }
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (auto iter1 = tree.cbeginRootChildren(); iter1; ++iter1) {
                for (auto iter2 = iter1->cbeginChildOn(); iter2; ++iter2) {
                    for (auto iter3 = iter2->cbeginChildOn(); iter3; ++iter3) {
                        for (auto iter4 = iter3->cbeginValueOn(); iter4; ++iter4) {
                            total += iter4.getValue();
                        }
                    }
                }
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto getValueSequential3 = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto getValueDirect = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0;

        { // warm up
            for (const auto& ijk : ijks) {
                total += root.getValue(ijk);
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (const auto& ijk : ijks) {
                total += root.getValue(ijk);
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto getValueAccessor = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0;

        { // warm up
            tree::ValueAccessor<FloatTree> valueAccessor(tree);
            for (const auto& ijk : ijks) {
                total += valueAccessor.getValue(ijk);
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::ValueAccessor<FloatTree> valueAccessor(tree);

            for (const auto& ijk : ijks) {
                total += valueAccessor.getValue(ijk);
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueSequential = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto leaf = tree.cbeginLeaf(); leaf; ++leaf) {
                for (auto iter = leaf->cbeginValueOn(); iter; ++iter) {
                    total += iter.getValue();
                }
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (auto leaf = tree.beginLeaf(); leaf; ++leaf) {
                for (auto iter = leaf->beginValueOn(); iter; ++iter) {
                    iter.setValue(iter.getValue() * 2);
                }
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueSequential3 = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            for (auto iter = tree.beginValueOn(); iter; ++iter) {
                iter.setValue(iter.getValue() * 2);
            }

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto valueIterRange = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::IteratorRange<FloatTree::ValueOnCIter> iterRange(tree.cbeginValueOn());

            total += iterRange.test() ? 1 : 0;

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto leafIterRange = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::IteratorRange<FloatTree::LeafCIter> iterRange(tree.cbeginLeaf());

            total += iterRange.test() ? 1 : 0;

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto nodeIterRange = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::IteratorRange<FloatTree::NodeCIter> iterRange(tree.cbeginNode());

            total += iterRange.test() ? 1 : 0;

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueForeachValue = [&](auto& tree, bool threaded, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        auto op = [&](const auto& iter) {
            iter.setValue(iter.getValue() * 2);
        };

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tools::foreach(tree.beginValueOn(), op, threaded);

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueForeachLeaf = [&](auto& tree, bool threaded, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        auto op = [&](const auto& leaf) {
            for (auto iter = leaf->beginValueOn(); iter; ++iter) {
                iter.setValue(iter.getValue() * 2);
            }
        };

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tools::foreach(tree.beginLeaf(), op, threaded);

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueForeachIterRange = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::IteratorRange<FloatTree::ValueOnIter> iterRange(tree.beginValueOn());

            total += iterRange.test() ? 1 : 0;

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueLeafManager = [&](auto& tree, bool threaded, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        float total = 0.0f;

        { // warm up
            for (auto iter = tree.cbeginValueOn(); iter; ++iter) {
                total += iter.getValue();
            }
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::LeafManager<FloatTree> leafManager(tree);
            DoubleOp op;
            leafManager.foreach(op, threaded, /*grainSize=*/1);

            time += timer.milliseconds();

            if (total == 0.0f)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueLeafManagerCreate = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        size_t total = 0;

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::LeafManager<FloatTree> leafManager(tree);

            total += leafManager.leafCount();

            time += timer.milliseconds();

            if (total == 0)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueNodeManagerCreate = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        Index32 total = 0;

        { // warm up
            tree::NodeManager<FloatTree> nodeManager(tree);
            total += nodeManager.root().childCount();
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::NodeManager<FloatTree> nodeManager(tree);
            total += nodeManager.root().childCount();

            time += timer.milliseconds();

            if (total == 0)     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueNodeManager = [&](auto& tree, bool threaded, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        { // warm up
            tree::NodeManager<FloatTree> nodeManager(tree);
            DoubleOp op;
            nodeManager.foreachTopDown(op, threaded);
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::NodeManager<FloatTree> nodeManager(tree);
            DoubleOp op;
            nodeManager.foreachTopDown(op, threaded, /*grainSize=*/1);

            time += timer.milliseconds();
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueDynamicNodeManagerCreate = [&](auto& tree, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        Index32 total = 0;

        { // warm up
            tree::DynamicNodeManager<FloatTree> nodeManager(tree);
            total += nodeManager.root().childCount();
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::DynamicNodeManager<FloatTree> nodeManager(tree);
            total += nodeManager.root().childCount();

            time += timer.milliseconds();

            if (total == Index32(0))     std::cerr << "Zero" << std::endl; // prevent optimization
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    auto setValueDynamicNodeManager = [&](auto& tree, bool threaded, int iterations = 10)
    {
        auto& root = tree.root();

        double time = 0.0f;

        { // warm up
            tree::DynamicNodeManager<FloatTree> nodeManager(tree);
            DoubleOp op;
            nodeManager.foreachTopDown(op, threaded);
        }

        for (int i = 0; i < iterations; i++) {

            timer.start();

            tree::DynamicNodeManager<FloatTree> nodeManager(tree);
            DoubleOp op;
            nodeManager.foreachTopDown(op, threaded, /*grainSize=*/1);

            time += timer.milliseconds();
        }

        util::printTime(std::cerr, time/iterations, " completed in ", "\n", 4, 3, 1);
    };

    // run benchmarks

    {
        std::cerr << "Cloud Set Value Sequential Leaf Iterator ...";
        FloatTree tree = openCloud();
        setValueSequential(tree, 1);
    }

    {
        std::cerr << "Cloud Set Value Foreach Leaf ...";
        FloatTree tree = openCloud();
        setValueForeachLeaf(tree, false, 1);
    }

    {
        std::cerr << "Cloud Leaf Iterator Range ...";
        FloatTree tree = openCloud();
        leafIterRange(tree);
    }

    {
        std::cerr << "Cloud Node Iterator Range ...";
        FloatTree tree = openCloud();
        nodeIterRange(tree);
    }

    {
        std::cerr << "Cloud Value Iterator Range ...";
        FloatTree tree = openCloud();
        valueIterRange(tree);
    }

    {
        std::cerr << "1 Tile Coalesced Root Query Direct ...";
        FloatTree tree;
        addOneTile(tree);
        addCoalescedIJKs();
        rootQueryDirect(tree);
    }

    {
        std::cerr << "1 Tile Coalesced Root Query Accessor ...";
        FloatTree tree;
        addOneTile(tree);
        addCoalescedIJKs();
        rootQueryAccessor(tree);
    }

    {
        std::cerr << "1 Tile Interleaved Root Query Direct ...";
        FloatTree tree;
        addOneTile(tree);
        addInterleavedIJKs();
        rootQueryDirect(tree);
    }

    {
        std::cerr << "1 Tile Interleaved Root Query Accessor ...";
        FloatTree tree;
        addOneTile(tree);
        addInterleavedIJKs();
        rootQueryAccessor(tree);
    }

    {
        std::cerr << "8 Tiles Coalesced Root Query Direct ...";
        FloatTree tree;
        addEightTiles(tree);
        addCoalescedIJKs();
        rootQueryDirect(tree);
    }

    {
        std::cerr << "8 Tiles Coalesced Root Query Accessor ...";
        FloatTree tree;
        addEightTiles(tree);
        addCoalescedIJKs();
        rootQueryAccessor(tree);
    }

    {
        std::cerr << "8 Tiles Interleaved Root Query Direct ...";
        FloatTree tree;
        addEightTiles(tree);
        addInterleavedIJKs();
        rootQueryDirect(tree);
    }

    {
        std::cerr << "8 Tiles Interleaved Root Query Accessor ...";
        FloatTree tree;
        addEightTiles(tree);
        addInterleavedIJKs();
        rootQueryAccessor(tree);
    }

    {
        std::cerr << "64 Tiles Coalesced Root Query Direct ...";
        FloatTree tree;
        addSixtyFourTiles(tree);
        addCoalescedIJKs();
        rootQueryDirect(tree);
    }

    {
        std::cerr << "64 Tiles Coalesced Root Query Accessor ...";
        FloatTree tree;
        addSixtyFourTiles(tree);
        addCoalescedIJKs();
        rootQueryAccessor(tree);
    }

    {
        std::cerr << "64 Tiles Interleaved Root Query Direct ...";
        FloatTree tree;
        addSixtyFourTiles(tree);
        addInterleavedIJKs();
        rootQueryDirect(tree);
    }

    {
        std::cerr << "64 Tiles Interleaved Root Query Accessor ...";
        FloatTree tree;
        addSixtyFourTiles(tree);
        addInterleavedIJKs();
        rootQueryAccessor(tree);
    }

    {
        std::cerr << "Cloud Set Value Sequential Leaf Iterator ...";
        FloatTree tree = openCloud();
        setValueSequential(tree);
    }

    {
        std::cerr << "Cloud Set Value Foreach Leaf ...";
        FloatTree tree = openCloud();
        setValueForeachLeaf(tree, false);
    }

    {
        std::cerr << "Cloud Get Value Sequential Leaf Iterator ...";
        FloatTree tree = openCloud();
        getValueSequential(tree);
    }

    {
        std::cerr << "Cloud Get Value Sequential Hierarchy Iterator ...";
        FloatTree tree = openCloud();
        getValueSequential2(tree);
    }

    {
        std::cerr << "Cloud Get Value Sequential Voxel Iterator ...";
        FloatTree tree = openCloud();
        getValueSequential3(tree);
    }

    {
        std::cerr << "Cloud Get Value Sequential Direct ...";
        FloatTree tree = openCloud();
        addSequentialVoxelIJKs(tree);
        getValueDirect(tree);
    }

    {
        std::cerr << "Cloud Get Value Sequential Accessor ...";
        FloatTree tree = openCloud();
        addSequentialVoxelIJKs(tree);
        getValueAccessor(tree);
    }

    {
        std::cerr << "Cloud Get Value Interleaved Direct ...";
        FloatTree tree = openCloud();
        addInterleavedVoxelIJKs(tree);
        getValueDirect(tree);
    }

    {
        std::cerr << "Cloud Get Value Interleaved Accessor ...";
        FloatTree tree = openCloud();
        addInterleavedVoxelIJKs(tree);
        getValueAccessor(tree);
    }

    {
        std::cerr << "Cloud Set Value Sequential Leaf Iterator ...";
        FloatTree tree = openCloud();
        setValueSequential(tree);
    }

    {
        std::cerr << "Cloud Set Value Sequential Voxel Iterator ...";
        FloatTree tree = openCloud();
        setValueSequential3(tree);
    }

    {
        std::cerr << "Cloud Set Value Foreach Value ...";
        FloatTree tree = openCloud();
        setValueForeachValue(tree, false);
    }

    {
        std::cerr << "Cloud Set Value Foreach Leaf ...";
        FloatTree tree = openCloud();
        setValueForeachLeaf(tree, false);
    }

    for (int n = 1; n <= 32; n *= 2) {
        std::cerr << "Cloud Set Value Foreach Value Thread" << n << " ...";
        tbb::global_control global_control(tbb::global_control::max_allowed_parallelism, n);
        FloatTree tree = openCloud();
        setValueForeachValue(tree, true);
    }

    for (int n = 1; n <= 32; n *= 2) {
        std::cerr << "Cloud Set Value Foreach Leaf Thread" << n << " ...";
        tbb::global_control global_control(tbb::global_control::max_allowed_parallelism, n);
        FloatTree tree = openCloud();
        setValueForeachLeaf(tree, true);
    }

    {
        std::cerr << "Cloud Set Value Sequential Leaf Iterator ...";
        FloatTree tree = openCloud();
        setValueSequential(tree);
    }

    {
        std::cerr << "Cloud Set Value Foreach Leaf ...";
        FloatTree tree = openCloud();
        setValueForeachLeaf(tree, false);
    }

    {
        std::cerr << "Cloud Set Value Foreach Iter Range ...";
        FloatTree tree = openCloud();
        setValueForeachIterRange(tree);
    }

    {
        std::cerr << "Cloud Set Value LeafManager Create ...";
        FloatTree tree = openCloud();
        setValueLeafManagerCreate(tree);
    }

    {
        std::cerr << "Cloud Set Value LeafManager ...";
        FloatTree tree = openCloud();
        setValueLeafManager(tree, false);
    }

    for (int n = 1; n <= 32; n *= 2) {
        std::cerr << "Cloud Set Value LeafManager Thread" << n << " ...";
        tbb::global_control global_control(tbb::global_control::max_allowed_parallelism, n);
        FloatTree tree = openCloud();
        setValueLeafManager(tree, true);
    }

    {
        std::cerr << "Cloud Set Value NodeManager ...";
        FloatTree tree = openCloud();
        setValueNodeManager(tree, false);
    }

    {
        std::cerr << "Cloud Set Value NodeManager Create ...";
        FloatTree tree = openCloud();
        setValueNodeManagerCreate(tree);
    }

    for (int n = 1; n <= 32; n *= 2) {
        std::cerr << "Cloud Set Value NodeManager Thread" << n << " ...";
        tbb::global_control global_control(tbb::global_control::max_allowed_parallelism, n);
        FloatTree tree = openCloud();
        setValueNodeManager(tree, true);
    }

    {
        std::cerr << "Cloud Set Value DynamicNodeManager Create ...";
        FloatTree tree = openCloud();
        setValueDynamicNodeManagerCreate(tree);
    }

    {
        std::cerr << "Cloud Set Value DynamicNodeManager ...";
        FloatTree tree = openCloud();
        setValueDynamicNodeManager(tree, false);
    }

    for (int n = 1; n <= 32; n *= 2) {
        std::cerr << "Cloud Set Value DynamicNodeManager Thread" << n << " ...";
        tbb::global_control global_control(tbb::global_control::max_allowed_parallelism, n);
        FloatTree tree = openCloud();
        setValueDynamicNodeManager(tree, true);
    }

    {
        std::cerr << "Cloud Set Value Sequential Leaf Iterator ...";
        FloatTree tree = openCloud();
        setValueSequential(tree);
    }

    {
        std::cerr << "Cloud Set Value Foreach Leaf ...";
        FloatTree tree = openCloud();
        setValueForeachLeaf(tree, false);
    }

    return 0;
}
