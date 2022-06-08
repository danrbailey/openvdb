// to do:
// add two group parms for input names
// get the grids from the inputs
// check their type, check that the first input is a Vec3fgrid (check iterator, then the grid type)
// check that the second grid is a float grid (for now) (check iterator, then the grid type)

// create a new density grid of the same topology as the position grid
// Use NodeManager to iterate over the new density grid, sampling the positions from the position grid and then the values at the positions from the second input density grid

// // this relies on grid and the grid being run over having the same topology
template<typename SamplerT, bool IndexSpace = false>
struct SampleValueAtPositionOp
{
    SampleValueAtPositionOp(const openvdb::Vec3fTree& positionTree, const openvdb::FloatGrid& valueGrid)
        : mPositionAccessor(positionTree)
        , mValueSampler(openvdb::tree::ValueAccessor<FloatTree>(valueGrid->tree()), valueGrid->transform()) {}

    template <typename LeafNodeType>
    void operator()(LeafT& leaf, size_t) const
    {
         const auto posLeaf = mPositionAccessor.probeConstLeaf(leaf.origin());
         if (posLeaf) {
             for (auto iter = leaf.beginValueOn(); iter; ++iter) {
                if (IndexSpace) iter.setValue(mValueSampler.isSample(posLeaf->getValue(iter.pos())));
                else iter.setValue(mValueSampler.wsSample(posLeaf->getValue(iter.pos())));
             }
         }
    }
    openvdb::tree::ValueAccessor<const openvdb::Vec3fTree> mPositionAccessor;
    SamplerT mValueSampler;
};

openvdb::Vec3fGrid positionGrid;

openvdb::FloatGrid::Ptr newGrid = openvdb::FloatGrid::create();
newGrid->setTransform(positionGrid.transform().copy()); 
newGrid->topologyUnion(positionGrid);


using PointSamplerType = openvdb::GridSampler<openvdb::tree::ValueAccessor<const openvdb::FloatTree>, openvdb::tools::PointSampler>;

SampleValueAtPositionOp<PointSamplerType> sampleOp(positionGrid->tree(), densityGrid);

openvdb::tree::LeafManager<openvdb::FloatTree> manager(newGrid->tree());
manager.foreach(sampleOp);
