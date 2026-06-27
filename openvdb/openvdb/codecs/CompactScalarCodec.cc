// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#include "CompactScalarCodec.h"

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace codecs {
namespace internal {

void registerCompactCodecs()
{
    io::CodecRegistry::registerCodec<codecs::CompactScalarCodec<FloatGrid>>();
}

} // namespace internal
} // namespace codecs
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
