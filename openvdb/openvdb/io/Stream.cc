// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

#include "Stream.h"

#include "File.h" ///< @todo refactor
#include "GridDescriptor.h"
#include <openvdb/Exceptions.h>
#include <cstdint>

#include <zstd.h>

#include <cstdio> // for remove()
#include <cstring> // for std::memcpy()
#include <functional> // for std::bind()
#include <iostream>
#include <sstream>
#include <vector>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace io {


Stream::Stream(std::istream& is)
{
    if (!is) return;

    // Delayed loading has been removed - always read directly from the stream
    readHeader(is);

    // Tag the input stream with the library and file format version numbers
    // and the compression options specified in the header.
    StreamMetadata::Ptr streamMetadata(new StreamMetadata);
    io::setStreamMetadataPtr(is, streamMetadata, /*transfer=*/false);
    io::setVersion(is, libraryVersion(), fileVersion());
    io::setDataCompression(is, compression());

    const bool interleavedLayout = fileVersion() >= OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT;

    mMeta.reset(new MetaMap);
    int32_t gridCount = 0;

    mGrids.reset(new GridPtrVec);
    Archive::NamedGridMap namedGrids;

    if (interleavedLayout) {
        // Decompress the zstd-compressed metadata + grid count + all grid headers block.
        uint64_t uncompressedSize = 0, compressedSize = 0;
        is.read(reinterpret_cast<char*>(&uncompressedSize), sizeof(uint64_t));
        is.read(reinterpret_cast<char*>(&compressedSize), sizeof(uint64_t));

        // The stored buffer has the 4-byte zstd magic number stripped;
        // prepend it before decompression.
        static constexpr size_t kZstdMagicSize = 4;
        static constexpr uint32_t kZstdMagicNumber = 0xFD2FB528;

        std::vector<char> compressedBuf(kZstdMagicSize + compressedSize);
        std::memcpy(compressedBuf.data(), &kZstdMagicNumber, kZstdMagicSize);
        is.read(compressedBuf.data() + kZstdMagicSize, compressedSize);

        std::vector<char> uncompressedBuf(uncompressedSize);
        const size_t decompSize = ZSTD_decompress(
            uncompressedBuf.data(), uncompressedSize,
            compressedBuf.data(), kZstdMagicSize + compressedSize);
        if (ZSTD_isError(decompSize)) {
            OPENVDB_THROW(IoError, "zstd decompression of metadata failed: "
                << ZSTD_getErrorName(decompSize));
        }

        std::istringstream localStream(
            std::string(uncompressedBuf.data(), uncompressedSize),
            std::ios_base::binary);
        // Propagate version/compression metadata onto localStream so that
        // readGridHeader() works correctly when called below.
        Archive::setFormatVersion(localStream);
        Archive::setLibraryVersion(localStream);
        Archive::setDataCompression(localStream);
        mMeta->readMeta(localStream);
        localStream.read(reinterpret_cast<char*>(&gridCount), sizeof(int32_t));

        // All grid headers (including per-grid metadata and transforms) are in
        // the compressed block, so read them from localStream, not from is.
        struct GridEntry {
            GridDescriptor gd;
            io::CodecData::Ptr codecData;
        };
        std::vector<GridEntry> entries;
        entries.reserve(gridCount);
        for (int32_t i = 0; i < gridCount; ++i) {
            GridDescriptor gd;
            gd.readHeader(localStream, /*codec=*/true);
            io::CodecData::Ptr codecData = Archive::readGridHeader(
                gd, localStream, io::ReadOptions{}, mReadDiagnostics);
            entries.push_back({std::move(gd), std::move(codecData)});
        }

        // If the data was written seekably (e.g. by File), the offset table
        // sits between the compressed block and the topology data.  Consume
        // it so the read position is aligned to the first topology chunk.
        // If written non-seekably (by Stream), the offset table is at the end
        // and can be ignored; topology follows the compressed block directly.
        if (inputHasGridOffsetsAtStart()) {
            for (const auto& [gd, codecData] : entries) {
                if (gd.isInstance()) continue;
                int64_t topologyPos = 0, dataPos = 0;
                is.read(reinterpret_cast<char*>(&topologyPos), sizeof(int64_t));
                is.read(reinterpret_cast<char*>(&dataPos), sizeof(int64_t));
            }
        }

        // Topologies and buffers are written sequentially in descriptor order —
        // no seeking needed; just read them forward in the same order.

        // Read topology for all non-instance grids.
        for (auto& [gd, codecData] : entries) {
            if (!gd.isInstance()) {
                Archive::readGridTopology(codecData, gd, is, io::ReadOptions{}, mReadDiagnostics);
            }
        }

        // Read buffers for all non-instance grids.
        for (auto& [gd, codecData] : entries) {
            if (!gd.isInstance()) {
                Archive::readGridBuffers(codecData, gd, is, io::ReadOptions{}, mReadDiagnostics);
            }
            mGrids->push_back(codecData->grid);
            namedGrids[gd.uniqueName()] = codecData->grid;
        }

        // Connect instances (grids that share trees with other grids).
        for (auto& [gd, codecData] : entries) {
            Archive::connectInstance(gd, namedGrids);
        }
    } else {
        mMeta->readMeta(is);
        gridCount = readGridCount(is);

        // If written seekably (e.g. by File), stream positions are embedded in
        // each grid descriptor and must be consumed before reading grid data.
        std::vector<GridDescriptor> descriptors;
        descriptors.reserve(gridCount);
        for (int32_t i = 0; i < gridCount; ++i) {
            GridDescriptor gd;
            gd.readHeader(is, /*codec=*/false);
            if (inputHasGridOffsetsAtStart()) gd.readStreamPos(is);
            descriptors.push_back(gd);
            io::CodecData::Ptr codecData = Archive::readGridHeader(
                gd, is, io::ReadOptions{}, mReadDiagnostics);
            Archive::readGridTopology(codecData, gd, is, io::ReadOptions{}, mReadDiagnostics);
            Archive::readGridBuffers(codecData, gd, is, io::ReadOptions{}, mReadDiagnostics);
            GridBase::Ptr grid = codecData->grid;
            mGrids->push_back(grid);
            namedGrids[gd.uniqueName()] = grid;
        }

        // Connect instances (grids that share trees with other grids).
        for (size_t i = 0, N = descriptors.size(); i < N; ++i) {
            Archive::connectInstance(descriptors[i], namedGrids);
        }
    }
}


Stream::Stream(std::ostream& os)
    : Archive()
    , mOutputStream(&os)
{
}


Stream::Stream(const Stream& other)
    : Archive(other)
    , mMeta(other.mMeta)
    , mGrids(other.mGrids)
    , mOutputStream(other.mOutputStream)
{
}


Stream&
Stream::operator=(const Stream& other)
{
    if (&other != this) {
        mMeta = other.mMeta;
        mGrids = other.mGrids;
        mOutputStream = other.mOutputStream;
    }
    return *this;
}


SharedPtr<Archive>
Stream::copy() const
{
    return SharedPtr<Archive>(new Stream(*this));
}


////////////////////////////////////////


void
Stream::write(const GridCPtrVec& grids, const MetaMap& metadata,
    const io::WriteOptions& writeOptions) const
{
    if (mOutputStream == nullptr) {
        OPENVDB_THROW(ValueError, "no output stream was specified");
    }
    this->writeGrids(*mOutputStream, grids, metadata, writeOptions);
}


void
Stream::writeGrids(std::ostream& os, const GridCPtrVec& grids, const MetaMap& metadata,
    const io::WriteOptions& writeOptions) const
{
    Archive::write(os, grids, /*seekable=*/false, metadata, writeOptions);
}


////////////////////////////////////////


MetaMap::Ptr
Stream::getMetadata() const
{
    MetaMap::Ptr result;
    if (mMeta) {
        // Return a deep copy of the file-level metadata
        // that was read when this object was constructed.
        result.reset(new MetaMap(*mMeta));
    }
    return result;
}


GridPtrVecPtr
Stream::getGrids()
{
    return mGrids;
}

} // namespace io
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
