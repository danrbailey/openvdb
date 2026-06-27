// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

/// @file io/File.cc

#include "File.h"
#include "ThrottledStreamBuf.h"

#include <openvdb/Exceptions.h>
#include <openvdb/util/logging.h>
#include <openvdb/util/Assert.h>
#include <openvdb/util/CpuTimer.h>
#include <cstdint>

#include <zstd.h>

#include <sys/stat.h> // stat()

#include <cstdlib> // for getenv(), strtoul()
#include <cstring> // for strerror_r()
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace io {


File::File(const std::string& filename, int maximumBandwidth)
    : Archive()
    , mFilename(filename)
    , mMaxBandwidth(maximumBandwidth)
{
    setInputHasGridOffsetsAtStart(true);
}


File::~File() = default;


File::File(const File& other)
    : Archive(other)
    , mFilename(other.mFilename)
    , mMeta(other.mMeta)
    , mMaxBandwidth(other.mMaxBandwidth)
    , mIsOpen(false)
    , mGridDescriptors(other.mGridDescriptors)
    , mNamedGrids(other.mNamedGrids)
    , mGrids(other.mGrids)
{
}


File&
File::operator=(const File& other)
{
    if (&other != this) {
        Archive::operator=(other);
        mFilename = other.mFilename;
        mMeta = other.mMeta;
        mMaxBandwidth = other.mMaxBandwidth;
        mIsOpen = false; // don't want two file objects reading from the same stream
        mGridDescriptors = other.mGridDescriptors;
        mNamedGrids = other.mNamedGrids;
        mGrids = other.mGrids;
    }
    return *this;
}


SharedPtr<Archive>
File::copy() const
{
    return SharedPtr<Archive>{new File{*this}};
}


////////////////////////////////////////


const std::string&
File::filename() const
{
    return mFilename;
}


MetaMap::Ptr
File::fileMetadata()
{
    return mMeta;
}

MetaMap::ConstPtr
File::fileMetadata() const
{
    return mMeta;
}


const File::NameMap&
File::gridDescriptors() const
{
    return mGridDescriptors;
}

File::NameMap&
File::gridDescriptors()
{
    return mGridDescriptors;
}


std::istream&
File::inputStream() const
{
    if (!mInStream) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading");
    }
    return *mInStream;
}


////////////////////////////////////////


Index64
File::getSize() const
{
    /// @internal boost::filesystem::file_size() would be a more portable alternative,
    /// but as of 9/2014, Houdini ships without the Boost.Filesystem library,
    /// which makes it much less convenient to use that library.

    Index64 result = std::numeric_limits<Index64>::max();

    std::string mesg = "could not get size of file " + mFilename;

#ifdef _WIN32
    // Get the file size by seeking to the end of the file.
    std::ifstream fstrm(mFilename);
    if (fstrm) {
        fstrm.seekg(0, fstrm.end);
        result = static_cast<Index64>(fstrm.tellg());
    } else {
        OPENVDB_THROW(IoError, mesg);
    }
#else
    // Get the file size using the stat() system call.
    struct stat info;
    if (0 != ::stat(mFilename.c_str(), &info)) {
        std::string s = getErrorString();
        if (!s.empty()) mesg += " (" + s + ")";
        OPENVDB_THROW(IoError, mesg);
    }
    if (!S_ISREG(info.st_mode)) {
        mesg += " (not a regular file)";
        OPENVDB_THROW(IoError, mesg);
    }
    result = static_cast<Index64>(info.st_size);
#endif

    return result;
}


////////////////////////////////////////


bool
File::isOpen() const
{
    return mIsOpen;
}


bool
File::open()
{
    if (mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is already open");
    }
    mInStream.reset();
    mThrottledBuf.reset();
    mUnderlyingStream.reset();
    mIOTimeMilliseconds = 0.0;

    // Open the file using standard I/O (delayed loading has been removed)
    auto fileStream = std::make_unique<std::ifstream>(
        mFilename.c_str(), std::ios_base::in | std::ios_base::binary);

    if (fileStream->fail()) {
        OPENVDB_THROW(IoError, "could not open file " << mFilename);
    }

    // If a maximum read bandwidth was requested, wrap the file's stream buffer
    // in a throttling buffer that also records the time spent blocked on I/O.
    std::unique_ptr<std::istream> newStream;
    if (mMaxBandwidth > 0) {
        mThrottledBuf = std::make_unique<ThrottledStreamBuf>(
            fileStream->rdbuf(), static_cast<double>(mMaxBandwidth) * 1.0e6);
        mUnderlyingStream = std::move(fileStream); // keep the file alive
        newStream = std::make_unique<std::istream>(mThrottledBuf.get());
    } else {
        newStream = std::move(fileStream);
    }

    // Read in the file header.
    bool newFile = false;
    try {
        newFile = Archive::readHeader(*newStream);
    } catch (IoError& e) {
        if (e.what() && std::string("not a VDB file") == e.what()) {
            // Rethrow, adding the filename.
            OPENVDB_THROW(IoError, mFilename << " is not a VDB file");
        }
        throw;
    }

    mInStream.swap(newStream);

    // Tag the input stream with the file format and library version numbers
    // and other metadata.
    mStreamMetadata.reset(new StreamMetadata);
    mStreamMetadata->setSeekable(true);
    io::setStreamMetadataPtr(inputStream(), mStreamMetadata, /*transfer=*/false);
    Archive::setFormatVersion(inputStream());
    Archive::setLibraryVersion(inputStream());
    Archive::setDataCompression(inputStream());

    const bool interleavedLayout = fileVersion() >= OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT;

    // Read in the VDB metadata and grid count.
    mMeta = MetaMap::Ptr(new MetaMap);
    int32_t gridCount = 0;

    if (interleavedLayout) {
        // Decompress the zstd-compressed metadata + grid count block.
        uint64_t uncompressedSize = 0, compressedSize = 0;
        inputStream().read(reinterpret_cast<char*>(&uncompressedSize), sizeof(uint64_t));
        inputStream().read(reinterpret_cast<char*>(&compressedSize), sizeof(uint64_t));

        // The stored buffer has the 4-byte zstd magic number stripped;
        // prepend it before decompression.
        static constexpr size_t kZstdMagicSize = 4;
        static constexpr uint32_t kZstdMagicNumber = 0xFD2FB528;

        std::vector<char> compressedBuf(kZstdMagicSize + compressedSize);
        std::memcpy(compressedBuf.data(), &kZstdMagicNumber, kZstdMagicSize);
        inputStream().read(compressedBuf.data() + kZstdMagicSize, compressedSize);

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
        // checkFormatVersion() and setDataCompression() work correctly when
        // called by gd.readHeader() and Archive::readGridHeader() below.
        Archive::setFormatVersion(localStream);
        Archive::setLibraryVersion(localStream);
        Archive::setDataCompression(localStream);
        mMeta->readMeta(localStream);
        localStream.read(reinterpret_cast<char*>(&gridCount), sizeof(int32_t));

        mInterleavedGrids.clear();
        mGridDescriptors.clear();

        for (int32_t i = 0; i < gridCount; ++i) {
            GridDescriptor gd;
            gd.readHeader(localStream, /*codec=*/true);

            io::CodecData::Ptr codecData = Archive::readGridHeader(
                gd, localStream, io::ReadOptions{}, mReadDiagnostics);

            auto it = mGridDescriptors.insert({gd.gridName(), gd});
            mInterleavedGrids.emplace_back(it, std::move(codecData));
        }

        int64_t currentPos = inputStream().tellg();

        // If the file does not have grid offsets at the start, read the last integer in the file
        // as the offset of the offset table then seek to the offset table and read the offsets.
        inputStream().seekg(-static_cast<std::streamoff>(sizeof(int64_t)), std::ios_base::end);
        int64_t trailingOffset = 0;
        inputStream().read(reinterpret_cast<char*>(&trailingOffset), sizeof(int64_t));
        inputStream().seekg(trailingOffset);

        // Read in the grid offset table.
        for (auto& [gridName, gd] : mGridDescriptors) {
            if (gd.isInstance()) continue;
            int64_t topologyPos = 0;
            int64_t dataPos = 0;
            int64_t endPos = 0;
            inputStream().read(reinterpret_cast<char*>(&topologyPos), sizeof(int64_t));
            inputStream().read(reinterpret_cast<char*>(&dataPos), sizeof(int64_t));
            inputStream().read(reinterpret_cast<char*>(&endPos), sizeof(int64_t));
            gd.setTopologyPos(topologyPos);
            gd.setDataPos(dataPos);
            gd.setEndPos(endPos);
        }

        // Return to the original position.
        inputStream().seekg(currentPos);
    } else {
        mMeta->readMeta(inputStream());
        gridCount = readGridCount(inputStream());

        if (!inputHasGridOffsetsAtStart()) {
            OPENVDB_LOG_DEBUG_RUNTIME("file " << mFilename << " does not support partial reading");

            mGrids.reset(new GridPtrVec);
            mNamedGrids.clear();

            // Stream in the entire contents of the file and append all grids to mGrids.
            // Note: no stream positions were written (seekable=false), so do not call readStreamPos().
            for (int32_t i = 0; i < gridCount; ++i) {
                GridDescriptor gd;
                gd.readHeader(inputStream(), /*codec=*/false);

                io::CodecData::Ptr codecData = Archive::readGridHeader(gd, inputStream(), io::ReadOptions{}, mReadDiagnostics);
                Archive::readGridTopology(codecData, gd, inputStream(), io::ReadOptions{}, mReadDiagnostics);
                Archive::readGridBuffers(codecData, gd, inputStream(), io::ReadOptions{}, mReadDiagnostics);
                GridBase::Ptr grid = codecData->grid;

                mGridDescriptors.insert(std::make_pair(gd.gridName(), gd));
                mGrids->push_back(grid);
                mNamedGrids[gd.uniqueName()] = grid;
            }
            // Connect instances (grids that share trees with other grids).
            for (NameMapCIter it = mGridDescriptors.begin(); it != mGridDescriptors.end(); ++it) {
                Archive::connectInstance(it->second, mNamedGrids);
            }
        } else {
            mGridDescriptors.clear();

            for (int32_t i = 0, N = gridCount; i < N; ++i) {
                // Read the grid descriptor.
                GridDescriptor gd;
                gd.readHeader(inputStream(), /*codec=*/false);
                gd.readStreamPos(inputStream());

                // Add the descriptor to the dictionary.
                mGridDescriptors.insert(std::make_pair(gd.gridName(), gd));

                // Skip forward to the next descriptor.
                gd.seekToEnd(inputStream());
            }
        }
    }

    mIsOpen = true;
    return newFile; // true if file is not identical to opened file
}


void
File::close()
{
    // Cache the accumulated I/O time before destroying the throttling buffer,
    // so that readIOTimeMilliseconds() remains valid after close().
    if (mThrottledBuf) {
        mIOTimeMilliseconds = mThrottledBuf->readTimeMilliseconds();
    }

    // Reset all data.
    mMeta.reset();
    mInterleavedGrids.clear(); // clear before mGridDescriptors (iterators point into it)
    mGridDescriptors.clear();
    mGrids.reset();
    mNamedGrids.clear();
    mInStream.reset();
    mThrottledBuf.reset();
    mUnderlyingStream.reset();
    mStreamMetadata.reset();

    mIsOpen = false;
    setInputHasGridOffsetsAtStart(true);
}


double
File::readIOTimeMilliseconds() const
{
    return mThrottledBuf ? mThrottledBuf->readTimeMilliseconds() : mIOTimeMilliseconds;
}


////////////////////////////////////////


bool
File::hasGrid(const Name& name) const
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading");
    }
    return (findDescriptor(name) != mGridDescriptors.end());
}


MetaMap::Ptr
File::getMetadata() const
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading");
    }
    // Return a deep copy of the file-level metadata, which was read
    // when the file was opened.
    return MetaMap::Ptr(new MetaMap(*mMeta));
}


GridPtrVecPtr
File::getGrids(const io::ReadOptions& readOptions) const
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading");
    }

    const bool interleavedLayout = fileVersion() >= OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT;

    GridPtrVecPtr ret;
    if (!inputHasGridOffsetsAtStart() && !interleavedLayout) {
        // Legacy non-seekable: all grids already streamed in and stored in mGrids.
        ret = mGrids;
    } else {
        ret.reset(new GridPtrVec);

        Archive::NamedGridMap namedGrids;

        if (interleavedLayout) {
            // Mint fresh CodecData for each grid from the stored templates,
            // so multiple getGrids() calls are independent.
            std::vector<std::pair<NameMapCIter, io::CodecData::Ptr>> work;
            work.reserve(mInterleavedGrids.size());
            for (const auto& [it, tmpl] : mInterleavedGrids) {
                const GridDescriptor& gd = it->second;
                io::CodecData::Ptr fresh;
                if (tmpl && tmpl->codec) {
                    fresh = tmpl->codec->createData();
                    fresh->codec = tmpl->codec;
                    // copyGridWithNewTree preserves metadata + transform, empty tree.
                    fresh->grid = tmpl->grid->copyGridWithNewTree();
                    fresh->grid->setSaveFloatAsHalf(gd.saveFloatAsHalf());
                }
                work.emplace_back(it, std::move(fresh));
            }

            for (auto& [it, codecData] : work) {
                const GridDescriptor& gd = it->second;
                gd.seekToTopology(inputStream());
                Archive::readGridTopology(codecData, gd, inputStream(),
                    readOptions, mReadDiagnostics);
            }
            for (auto& [it, codecData] : work) {
                const GridDescriptor& gd = it->second;
                gd.seekToBuffers(inputStream());
                Archive::readGridBuffers(codecData, gd, inputStream(),
                    readOptions, mReadDiagnostics);
                ret->push_back(codecData->grid);
                namedGrids[gd.uniqueName()] = codecData->grid;
            }

            // Connect instances (grids that share trees with other grids).
            for (const auto& [it, tmpl] : mInterleavedGrids) {
                Archive::connectInstance(it->second, namedGrids);
            }
        } else {
            // Read all grids represented by the GridDescriptors.
            for (NameMapCIter i = mGridDescriptors.begin(), e = mGridDescriptors.end(); i != e; ++i) {
                const GridDescriptor& gd = i->second;
                // Seek to the grid in the file.
                gd.seekToGrid(inputStream());
                io::CodecData::Ptr codecData = Archive::readGridHeader(gd, inputStream(), readOptions, mReadDiagnostics);
                Archive::readGridTopology(codecData, gd, inputStream(), readOptions, mReadDiagnostics);
                gd.seekToBlocks(inputStream());
                Archive::readGridBuffers(codecData, gd, inputStream(), readOptions, mReadDiagnostics);
                ret->push_back(codecData->grid);
                namedGrids[gd.uniqueName()] = codecData->grid;
            }

            // Connect instances (grids that share trees with other grids).
            for (NameMapCIter i = mGridDescriptors.begin(), e = mGridDescriptors.end(); i != e; ++i) {
                Archive::connectInstance(i->second, namedGrids);
            }
        }
    }

    return ret;
}


GridBase::Ptr
File::retrieveCachedGrid(const Name& name) const
{
    // If the file has grid offsets, grids are read on demand
    // and not cached in mNamedGrids.
    if (inputHasGridOffsetsAtStart()) return GridBase::Ptr();

    // If the file does not have grid offsets, mNamedGrids should already
    // contain the entire contents of the file.

    // Search by unique name.
    Archive::NamedGridMap::const_iterator it =
        mNamedGrids.find(GridDescriptor::stringAsUniqueName(name));
    // If not found, search by grid name.
    if (it == mNamedGrids.end()) it = mNamedGrids.find(name);
    if (it == mNamedGrids.end()) {
        OPENVDB_THROW(KeyError, mFilename << " has no grid named \"" << name << "\"");
    }
    return it->second;
}


////////////////////////////////////////


GridPtrVecPtr
File::readAllGridMetadata()
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading");
    }

    if (fileVersion() < OPENVDB_FILE_VERSION_FLOAT_FRUSTUM_BBOX) {
        OPENVDB_THROW(IoError,
            "VDB file version < 221 (FLOAT_FRUSTUM_BBOX) is no longer supported.");
    }

    GridPtrVecPtr ret(new GridPtrVec);

    const bool interleavedLayout = fileVersion() >= OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT;
    if (!inputHasGridOffsetsAtStart() && !interleavedLayout) {
        // Legacy non-seekable: all grids already streamed in and stored in mGrids.
        for (size_t i = 0, N = mGrids->size(); i < N; ++i) {
            ret->push_back((*mGrids)[i]->copyGridWithNewTree());
        }
    } else if (interleavedLayout) {
        // Interleaved: metadata + transform are already in the mInterleavedGrids templates.
        for (const auto& [tmplIt, tmpl] : mInterleavedGrids) {
            ret->push_back(tmpl->grid->copyGridWithNewTree());
        }
    } else {
        // Legacy seekable: seek to each grid and read just the header.
        for (NameMapCIter i = mGridDescriptors.begin(), e = mGridDescriptors.end(); i != e; ++i) {
            const GridDescriptor& gd = i->second;
            gd.seekToGrid(inputStream());
            io::CodecData::Ptr codecData = Archive::readGridHeader(gd, inputStream(), io::ReadOptions{}, mReadDiagnostics);
            ret->push_back(codecData->grid->copyGridWithNewTree());
        }
    }
    return ret;
}


GridBase::Ptr
File::readGridMetadata(const Name& name)
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading.");
    }

    if (fileVersion() < OPENVDB_FILE_VERSION_FLOAT_FRUSTUM_BBOX) {
        OPENVDB_THROW(IoError,
            "VDB file version < 221 (FLOAT_FRUSTUM_BBOX) is no longer supported.");
    }

    const bool interleavedLayout = fileVersion() >= OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT;
    if (!inputHasGridOffsetsAtStart() && !interleavedLayout) {
        // Legacy non-seekable: use the cached grid from mNamedGrids.
        return readGrid(name)->copyGridWithNewTree();
    }

    NameMapCIter it = findDescriptor(name);
    if (it == mGridDescriptors.end()) {
        OPENVDB_THROW(KeyError, mFilename << " has no grid named \"" << name << "\"");
    }

    if (interleavedLayout) {
        // Interleaved: metadata + transform are already in the mInterleavedGrids template.
        for (const auto& [tmplIt, tmpl] : mInterleavedGrids) {
            if (tmplIt == it) return tmpl->grid->copyGridWithNewTree();
        }
        OPENVDB_THROW(KeyError, mFilename << " has no grid named \"" << name << "\"");
    }

    // Legacy seekable: seek to grid and read just the header.
    const GridDescriptor& gd = it->second;
    gd.seekToGrid(inputStream());
    io::CodecData::Ptr codecData = Archive::readGridHeader(gd, inputStream(), io::ReadOptions{}, mReadDiagnostics);
    return codecData->grid->copyGridWithNewTree();
}


////////////////////////////////////////


GridBase::Ptr
File::readGrid(const Name& name, const BBoxd& bbox)
{
    io::ReadOptions readOptions;
    readOptions.clipBBox = bbox;
    return readGrid(name, readOptions);
}


GridBase::Ptr
File::readGrid(const Name& name, const io::ReadOptions& readOptions)
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading.");
    }

    const bool interleavedLayout = fileVersion() >= OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT;

    GridBase::Ptr grid;
    if (!interleavedLayout) {
        // For Legacy, use the cache for non-seekable files (mNamedGrids was populated during open()).
        grid = retrieveCachedGrid(name);
        if (grid) {
            const auto& bbox = readOptions.clipBBox;
            if (bbox.isSorted()) {
                grid = grid->deepCopyGrid();
                grid->clipGrid(bbox);
            }
            return grid;
        }
    }

    NameMapCIter it = findDescriptor(name);
    if (it == mGridDescriptors.end()) {
        OPENVDB_THROW(KeyError, mFilename << " has no grid named \"" << name << "\"");
    }

    const GridDescriptor& gd = it->second;

    if (interleavedLayout) {
        // Interleaved: the grid header (metadata + transform) was decoded from the compressed
        // block during open() and lives in mInterleavedGrids. Works for both seekable and
        // non-seekable files (offset table was read from either the inline or trailing position).
        for (const auto& [tmplIt, tmpl] : mInterleavedGrids) {
            if (tmplIt != it) continue;
            io::CodecData::Ptr codecData = tmpl->codec->createData();
            codecData->codec = tmpl->codec;
            codecData->grid = tmpl->grid->copyGridWithNewTree();
            codecData->grid->setSaveFloatAsHalf(gd.saveFloatAsHalf());
            if (!gd.isInstance()) {
                gd.seekToTopology(inputStream());
                Archive::readGridTopology(codecData, gd, inputStream(), readOptions, mReadDiagnostics);
                gd.seekToBuffers(inputStream());
                Archive::readGridBuffers(codecData, gd, inputStream(), readOptions, mReadDiagnostics);
            }
            GridBase::Ptr grid = codecData->grid;
            const auto& bbox = readOptions.clipBBox;
            if (bbox.isSorted()) {
                grid = grid->deepCopyGrid();
                grid->clipGrid(bbox);
            }
            return grid;
        }
        OPENVDB_THROW(KeyError, mFilename << " has no grid named \"" << name << "\"");
    }

    // Legacy seekable: grid header is in the main stream at gd.gridPos.
    OPENVDB_ASSERT(inputHasGridOffsetsAtStart());
    gd.seekToGrid(inputStream());
    io::CodecData::Ptr codecData = Archive::readGridHeader(gd, inputStream(), readOptions, mReadDiagnostics);
    Archive::readGridTopology(codecData, gd, inputStream(), readOptions, mReadDiagnostics);
    Archive::readGridBuffers(codecData, gd, inputStream(), readOptions, mReadDiagnostics);
    grid = codecData->grid;

    if (gd.isInstance()) {
        /// @todo Refactor to share code with Archive::connectInstance()?
        NameMapCIter parentIt =
            findDescriptor(GridDescriptor::nameAsString(gd.instanceParentName()));
        if (parentIt == mGridDescriptors.end()) {
            OPENVDB_THROW(KeyError, "missing instance parent \""
                << GridDescriptor::nameAsString(gd.instanceParentName())
                << "\" for grid " << GridDescriptor::nameAsString(gd.uniqueName())
                << " in file " << mFilename);
        }

        GridBase::Ptr parent;
        parentIt->second.seekToGrid(inputStream());
        io::CodecData::Ptr parentCodecData = Archive::readGridHeader(parentIt->second, inputStream(), readOptions, mReadDiagnostics);
        Archive::readGridTopology(parentCodecData, parentIt->second, inputStream(), readOptions, mReadDiagnostics);
        Archive::readGridBuffers(parentCodecData, parentIt->second, inputStream(), readOptions, mReadDiagnostics);
        parent = parentCodecData->grid;
        if (parent) grid->setTree(parent->baseTreePtr());
    }
    return grid;
}


////////////////////////////////////////


void
File::writeGrids(const GridCPtrVec& grids, const MetaMap& meta, const io::WriteOptions& writeOptions) const
{
    if (mIsOpen) {
        OPENVDB_THROW(IoError,
            mFilename << " cannot be written because it is open for reading");
    }

    // Create a file stream and write it out.
    std::ofstream file;
    file.open(mFilename.c_str(),
        std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);

    if (file.fail()) {
        OPENVDB_THROW(IoError, "could not open " << mFilename << " for writing");
    }

    // Write out the vdb.
    Archive::write(file, grids, /*seekable=*/true, meta, writeOptions);

    file.close();
}



////////////////////////////////////////


File::NameMapCIter
File::findDescriptor(const Name& name) const
{
    const Name uniqueName = GridDescriptor::stringAsUniqueName(name);

    // Find all descriptors with the given grid name.
    std::pair<NameMapCIter, NameMapCIter> range = mGridDescriptors.equal_range(name);

    if (range.first == range.second) {
        // If no descriptors were found with the given grid name, the name might have
        // a suffix ("name[N]").  In that case, remove the "[N]" suffix and search again.
        range = mGridDescriptors.equal_range(GridDescriptor::stripSuffix(uniqueName));
    }

    const size_t count = size_t(std::distance(range.first, range.second));
    if (count > 1 && name == uniqueName) {
        OPENVDB_LOG_WARN(mFilename << " has more than one grid named \"" << name << "\"");
    }

    NameMapCIter ret = mGridDescriptors.end();

    if (count > 0) {
        if (name == uniqueName) {
            // If the given grid name is unique or if no "[N]" index was given,
            // use the first matching descriptor.
            ret = range.first;
        } else {
            // If the given grid name has a "[N]" index, find the descriptor
            // with a matching unique name.
            for (NameMapCIter it = range.first; it != range.second; ++it) {
                const Name candidateName = it->second.uniqueName();
                if (candidateName == uniqueName || candidateName == name) {
                    ret = it;
                    break;
                }
            }
        }
    }
    return ret;
}


////////////////////////////////////////


File::NameIterator
File::beginName() const
{
    if (!mIsOpen) {
        OPENVDB_THROW(IoError, mFilename << " is not open for reading");
    }
    return File::NameIterator(mGridDescriptors.begin());
}


File::NameIterator
File::endName() const
{
    return File::NameIterator(mGridDescriptors.end());
}


} // namespace io
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb
