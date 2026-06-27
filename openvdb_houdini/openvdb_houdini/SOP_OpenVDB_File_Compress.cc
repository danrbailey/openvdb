// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0
//
/// @file SOP_OpenVDB_File_Compress.cc
///
/// @author Dan Bailey

#include <houdini_utils/ParmFactory.h>
#include <openvdb_houdini/Utils.h>
#include <openvdb_houdini/SOP_NodeVDB.h>
#include <openvdb/Metadata.h>
#include <UT/UT_Interrupt.h>
#include <stdexcept>
#include <string>


namespace hvdb = openvdb_houdini;
namespace hutil = houdini_utils;


class SOP_OpenVDB_File_Compress: public hvdb::SOP_NodeVDB
{
public:
    SOP_OpenVDB_File_Compress(OP_Network*, const char* name, OP_Operator*);
    ~SOP_OpenVDB_File_Compress() override {}

    static OP_Node* factory(OP_Network*, const char* name, OP_Operator*);

    class Cache: public SOP_VDBCacheOptions { OP_ERROR cookVDBSop(OP_Context&) override; };
};


void
newSopOperator(OP_OperatorTable* table)
{
    if (table == nullptr) return;

    hutil::ParmList parms;

    parms.add(hutil::ParmFactory(PRM_STRING, "group", "Group")
        .setTooltip("Specify a subset of the input VDBs to be modified.")
        .setChoiceList(&hutil::PrimGroupMenuInput1)
        .setDocumentation(
            "A subset of the input VDBs to be modified"
            " (see [specifying volumes|/model/volumes#group])"));

    parms.add(hutil::ParmFactory(PRM_STRING, "codec", "Codec")
        .setDefault("legacy")
        .setChoiceListItems(PRM_CHOICELIST_SINGLE, {
            "legacy",  "Legacy (lossless)",
            "compact", "Compact (lossless)"
        })
        .setTooltip(
            "Legacy (lossless):\n"
            "    Use the legacy codec (lossless).\n"
            "Compact (lossless):\n"
            "    Use the compact codec (lossless)."));

    hvdb::OpenVDBOpFactory("VDB File Compress", SOP_OpenVDB_File_Compress::factory, parms, *table)
        .setNativeName("")
        .addInput("Input with VDB grids to operate on")
        .setVerb(SOP_NodeVerb::COOK_INPLACE, []() { return new SOP_OpenVDB_File_Compress::Cache; })
        .setDocumentation("\
#icon: COMMON/openvdb\n\
#tags: vdb\n\
\n\
\"\"\"Apply lossless compression to VDB volumes for file I/O.\"\"\"\n\
\n\
@overview\n\
\n\
This node applies lossless compression to VDB volumes to reduce file size on disk.\n\
\n\
@examples\n\
\n\
See [openvdb.org|http://www.openvdb.org/download/] for source code\n\
and usage examples.\n");
}


OP_Node*
SOP_OpenVDB_File_Compress::factory(OP_Network* net,
    const char* name, OP_Operator* op)
{
    return new SOP_OpenVDB_File_Compress(net, name, op);
}


SOP_OpenVDB_File_Compress::SOP_OpenVDB_File_Compress(OP_Network* net,
    const char* name, OP_Operator* op):
    hvdb::SOP_NodeVDB(net, name, op)
{
}


OP_ERROR
SOP_OpenVDB_File_Compress::Cache::cookVDBSop(OP_Context& context)
{
    try {
        const fpreal time = context.getTime();

        const std::string codec = evalStdString("codec", time);

        const GA_PrimitiveGroup* group = matchGroup(*gdp, evalStdString("group", time));

        UT_AutoInterrupt progress("Set VDB file compression");

        for (hvdb::VdbPrimIterator it(gdp, group); it; ++it) {
            if (progress.wasInterrupted()) throw std::runtime_error("was interrupted");

            hvdb::Grid& grid = (*it)->getGrid();

            if (codec == "compact") {
                grid.insertMeta("codec", openvdb::StringMetadata("compact"));
            } else {
                grid.removeMeta("codec");
            }
        }
    } catch (std::exception& e) {
        addError(SOP_MESSAGE, e.what());
    }
    return error();
}
