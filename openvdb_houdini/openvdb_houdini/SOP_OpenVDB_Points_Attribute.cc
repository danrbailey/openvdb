//
// TM & (c) Lucasfilm Entertainment Company Ltd. and Lucasfilm Ltd.
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//

/// @file SOP_OpenVDB_Points_Attribute.cc
///
/// @author Dan Bailey
///
/// @brief Delete point attributes.

#include <openvdb/openvdb.h>
#include <openvdb/points/PointAttribute.h>

#include <openvdb_houdini/SOP_NodeVDB.h>
#include <openvdb_houdini/Utils.h>
#include <houdini_utils/ParmFactory.h>

#include <UT/UT_String.h> // for Houdini pattern-matching logic

#include <string>
#include <vector>


using namespace openvdb;
using namespace openvdb::points;

namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;


////////////////////////////////////////


class SOP_OpenVDB_Points_Attribute: public hvdb::SOP_NodeVDB
{
public:
    SOP_OpenVDB_Points_Attribute(OP_Network*, const char* name, OP_Operator*);
    ~SOP_OpenVDB_Points_Attribute() override = default;

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);

    class Cache: public SOP_VDBCacheOptions { OP_ERROR cookVDBSop(OP_Context&) override; };
}; // class SOP_OpenVDB_Points_Attribute


////////////////////////////////////////


namespace {

inline bool
testForDeletion(const Name& name, const bool deleteAll,
    const std::vector<Name>& toDelete, const std::vector<Name>& toKeep)
{
    // position attribute is mandatory
    if (name == "P")    return false;

    // TODO: Houdini pattern-matching logic should be available in openvdb::points::AttributeSet
    // for now we convert to UT_String and use the pattern-matching functionality provided there

    UT_String nameStr(name.c_str());

    if (deleteAll) {
        for (const auto& keepName : toKeep) {
            if (nameStr.match(keepName.c_str())) {
                return false;
            }
        }
        return true;
    }

    for (const auto& deleteName : toDelete) {
        if (nameStr.match(deleteName.c_str())) {
            return true;
        }
    }
    return false;
}


inline int
lookupAttrInput(const PRM_SpareData* spare)
{
    const char  *istring;
    if (!spare) return 0;
    istring = spare->getValue("sop_input");
    return istring ? atoi(istring) : 0;
}


inline void
populateAttributeMenu(void* data, PRM_Name* menuEntries, int menuSize,
    const PRM_SpareData* spare, const PRM_Parm*)
{
    menuEntries[0].setToken(0);
    menuEntries[0].setLabel(0);

    SOP_Node* sop = CAST_SOPNODE(static_cast<OP_Node*>(data));
    if (sop == nullptr) return;

    size_t count = 0;

    try {
        const int inputIndex = lookupAttrInput(spare);
        const GU_Detail* gdp = sop->getInputLastGeo(inputIndex, CHgetEvalTime());

        if (gdp) {
            // const cast as iterator requires non-const access, however data is not modified
            hvdb::VdbPrimIterator vdbIt(const_cast<GU_Detail*>(gdp));

            for (; vdbIt; ++vdbIt) {
                GU_PrimVDB* vdbPrim = *vdbIt;

                PointDataGrid::ConstPtr grid =
                        gridConstPtrCast<PointDataGrid>(vdbPrim->getConstGridPtr());

                // ignore all but point data grids
                if (!grid)      continue;
                auto leafIter = grid->tree().cbeginLeaf();
                if (!leafIter)  continue;

                const AttributeSet::Descriptor& descriptor =
                    leafIter->attributeSet().descriptor();

                for (const auto& it : descriptor.map()) {
                    // add each VDB Points attribute (except P) to the menu
                    if (it.first == "P")    continue;
                    // skip if exceeded max menu size
                    if (count > (menuSize - 2))  continue;
                    menuEntries[count].setToken(it.first.c_str());
                    menuEntries[count].setLabel(it.first.c_str());
                    count++;
                }
            }
        }
    } catch (...) {}

    // Terminate the list.
    menuEntries[count].setToken(0);
    menuEntries[count].setLabel(0);
}


} // unnamed namespace


////////////////////////////////////////


// Build UI and register this operator.
void
newSopOperator(OP_OperatorTable* table)
{
    openvdb::initialize();

    if (table == nullptr) return;

    hutil::ParmList parms;

    parms.add(hutil::ParmFactory(PRM_STRING, "group", "Group")
        .setChoiceList(&hutil::PrimGroupMenu)
        .setTooltip("Specify a subset of the input point data grids to convert.")
        .setDocumentation(
            "A subset of the input VDB Points primitives to be processed"
            " (see [specifying volumes|/model/volumes#group])"));

    parms.beginExclusiveSwitcher("mode", "Mode");
    parms.addFolder("Delete");

    parms.add(hutil::ParmFactory(PRM_STRING, "pointattributes", "Point Attributes")
        .setChoiceList(new PRM_ChoiceList(PRM_CHOICELIST_TOGGLE, populateAttributeMenu))
        .setTooltip("Specify VDB points attributes to delete."));

    parms.endSwitcher();

    //////////
    // Register this operator.

    hvdb::OpenVDBOpFactory("VDB Points Attribute",
        SOP_OpenVDB_Points_Attribute::factory, parms, *table)
        .addAliasVerbatim("DW_OpenOpenVDBPointsAttribute")
        .addInput("VDB Points")
        .setVerb(SOP_NodeVerb::COOK_INPLACE, []() { return new SOP_OpenVDB_Points_Attribute::Cache; })
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Delete point attributes.\"\"\"\n\
\n\
@overview\n\
\n\
The OpenVDB Points Attribute SOP provides the ability to delete point attributes from one or more VDB Points grids.\n\
As with the native Attribute Delete SOP, you can specify attributes to delete by name or using Houdini\n\
pattern-matching rules with wildcard characters.\n\
\n\
@related\n\
- [OpenVDB Points Convert|Node:sop/DW_OpenVDBPointsConvert]\n\
- [OpenVDB Points Delete|Node:sop/DW_OpenVDBPointsGroup]\n\
- [Node:sop/attribdelete]\n\
\n\
@examples\n\
\n\
See [openvdb.org|http://www.openvdb.org/download/] for source code\n\
and usage examples.\n");
}


////////////////////////////////////////


OP_Node*
SOP_OpenVDB_Points_Attribute::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_Points_Attribute(net, name, op);
}


SOP_OpenVDB_Points_Attribute::SOP_OpenVDB_Points_Attribute(OP_Network* net,
    const char* name, OP_Operator* op)
    : hvdb::SOP_NodeVDB(net, name, op)
{
}


////////////////////////////////////////


OP_ERROR
SOP_OpenVDB_Points_Attribute::Cache::cookVDBSop(OP_Context& context)
{
    try {
        // extract names of all selected attributes and split into include and exclude (suffixed by ^)

        std::vector<std::string> toDelete, toKeep;
        bool deleteAll;

        AttributeSet::Descriptor::parseNames(toDelete, toKeep, deleteAll,
            evalStdString("pointattributes", context.getTime()), /*checkValidity=*/false);

        // clear list of attributes to keep if wildcard "*" not included
        if (!deleteAll)     toKeep.clear();

        // sort attribute lists
        std::sort(toDelete.begin(), toDelete.end());
        std::sort(toKeep.begin(), toKeep.end());

        // delete any duplicates
        toDelete.erase(std::unique(toDelete.begin(), toDelete.end()), toDelete.end());
        toKeep.erase(std::unique(toKeep.begin(), toKeep.end()), toKeep.end());

        // early exit if no attributes to delete
        if (!deleteAll && toDelete.empty()) {
            return error();
        }

        UT_AutoInterrupt progress("Processing points attribute deletion");

        // iterate over primitives based on group

        const GA_PrimitiveGroup* group = matchGroup(*gdp,
            evalStdString("group", context.getTime()));

        for (hvdb::VdbPrimIterator vdbIt(gdp, group); vdbIt; ++vdbIt) {
            if (progress.wasInterrupted()) {
                throw std::runtime_error("processing was interrupted");
            }
            GU_PrimVDB* vdbPrim = *vdbIt;

            PointDataGrid::ConstPtr inputGrid =
                    openvdb::gridConstPtrCast<PointDataGrid>(vdbPrim->getConstGridPtr());

            // early exit if the grid is of the wrong type
            if (!inputGrid) continue;

            // early exit if the tree is empty
            auto leafIter = inputGrid->tree().cbeginLeaf();
            if (!leafIter) continue;

            const AttributeSet::Descriptor& descriptor =
                leafIter->attributeSet().descriptor();

            // find any attributes to be deleted
            std::vector<std::string> deleteAttributes;
            for (const auto& it : descriptor.map()) {
                const auto& name = it.first;
                if (testForDeletion(name, deleteAll, toDelete, toKeep)) {
                    deleteAttributes.push_back(name);
                }
            }

            // early exit if no attributes to delete
            if (deleteAttributes.empty())  continue;

            // deep copy the VDB tree if it is not already unique
            vdbPrim->makeGridUnique();

            PointDataGrid& outputGrid = UTvdbGridCast<PointDataGrid>(vdbPrim->getGrid());
            dropAttributes(outputGrid.tree(), deleteAttributes);
        }

    } catch (const std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }
    return error();
}
