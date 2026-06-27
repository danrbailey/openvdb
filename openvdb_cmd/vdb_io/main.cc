// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0

/// @file vdb_io/main.cc
///
/// @brief Command-line tool for assessing OpenVDB read performance.
///
/// Reads one or more VDB files and reports how long the read took. By default
/// the operating system page cache for each file is evicted before reading, so
/// that the measurement reflects a cold read from disk. A maximum read
/// bandwidth can be imposed to simulate a slower device, in which case the
/// total time is broken down into time blocked on I/O and other (overlapped
/// compute) time.

#include <openvdb/openvdb.h>
#include <openvdb/io/File.h>
#include <openvdb/util/logging.h>

#include <openvdb/io/GridDescriptor.h>
#include <openvdb/codecs/CompactScalarCodec.h> // for CompactScalarCodecReadOptions

#include <algorithm> // for std::max()
#include <chrono>
#include <cstdint> // for std::uintmax_t
#include <cstdlib> // for std::atoi(), std::atoll()
#include <cstring> // for std::strerror()
#include <filesystem> // for std::filesystem::file_size()
#include <iomanip> // for std::setprecision()
#include <iostream>
#include <map> // for std::map
#include <set> // for std::set
#include <sstream> // for std::ostringstream
#include <string>
#include <system_error> // for std::error_code
#include <vector>

#if defined(__linux__)
#include <fcntl.h>     // for open(), posix_fadvise()
#include <sys/mman.h>  // for mmap(), munmap()
#include <sys/stat.h>  // for fstat()
#include <unistd.h>    // for close(), sysconf()
#include <cerrno>
#endif


namespace {

const char* gProgName = "";


inline void
usage [[noreturn]] (int exitStatus = EXIT_FAILURE)
{
    std::cerr <<
"Usage: " << gProgName << " file.vdb [file.vdb ...] [options]\n" <<
"Which: reads OpenVDB files and reports how long each read took\n" <<
"Options:\n" <<
"    -bandwidth N    limit read throughput to N MB/s (default: unlimited).\n" <<
"                    When set, the read time is split into total, I/O and\n" <<
"                    other (overlapped compute) time.\n" <<
"    -iterations N   read each file N times and report each iteration on its\n" <<
"                    own line (default: 1)\n" <<
"    -page-cache     prefetch the whole file into the OS page cache before\n" <<
"                    reading (the equivalent of \"vmtouch -t <file>\"), instead\n" <<
"                    of the default eviction; use this to measure warm reads\n" <<
"    -output FILE    after reading, write the grids back out to FILE\n" <<
"    -codec NAME     codec to use for -output: \"legacy\" (no codec metadata)\n" <<
"                    or \"compact\" (default: legacy)\n" <<
"    -max-voxels-per-chunk N\n" <<
"                    for the compact codec, override the target number of\n" <<
"                    active voxels per leaf chunk (default: 100000)\n" <<
"    -decompress-threads N\n" <<
"                    for the compact codec, the maximum number of threads used\n" <<
"                    to decompress value buffers when reading. Clamped to the\n" <<
"                    number of logical cores (minimum 1; default: codec choice)\n" <<
"    -read-float-bytes N\n" <<
"                    for the compact codec, read only the N most-significant\n" <<
"                    byte planes of each 4-byte float value (1..4) and zero the\n" <<
"                    rest, skipping the dropped planes on disk for a faster but\n" <<
"                    lossy read. N=1 keeps just the sign/exponent byte; N=4 (the\n" <<
"                    default) is lossless. Ignored for non-float value types.\n" <<
"    -zstd-PARAM V   for the compact codec, override an advanced zstd\n" <<
"                    compression parameter for all non-byte-planed (and\n" <<
"                    integer) value compression. PARAM is one of:\n" <<
"                    wlog, hlog, clog, slog, mml, tlen, strat (the same\n" <<
"                    aliases as zstd --zstd=...). Unset = compression preset 3.\n" <<
"    -zstd-floatN-PARAM V\n" <<
"                    same, but for byte plane N of float value types (plane 0\n" <<
"                    is the exponent, higher planes the mantissa), letting each\n" <<
"                    plane be tuned independently\n" <<
"    -version        print version information\n" <<
"    -h, -help       print this usage message\n" <<
"\n" <<
"By default the OS page cache for each file is evicted before reading, so the\n" <<
"measurement reflects a cold read from disk.\n" <<
"\n" <<
"Examples:\n" <<
"    Cold read of a file, full speed:\n" <<
"        " << gProgName << " file.vdb\n" <<
"    Simulate a 50 MB/s device (e.g. cold NFS):\n" <<
"        " << gProgName << " -bandwidth 50 file.vdb\n" <<
"    Time three separate cold reads:\n" <<
"        " << gProgName << " -iterations 3 file.vdb\n" <<
"    Read a file and re-write it with the compact codec:\n" <<
"        " << gProgName << " input.vdb -output output.vdb -codec compact\n" <<
"    Re-write with tuned zstd parameters (different for exponent vs mantissa):\n" <<
"        " << gProgName << " input.vdb -output output.vdb -codec compact"
        " -zstd-strat 6 -zstd-float0-wlog 20 -zstd-float1-wlog 23\n" <<
"    Lossy read keeping only the two leading byte planes of each float:\n" <<
"        " << gProgName << " -read-float-bytes 2 file.vdb\n";
    exit(exitStatus);
}


/// @brief Evict the operating system page cache for the given file.
/// @details This is the minimal equivalent of "vmtouch -e <file>": it opens the
/// file, determines its size and advises the kernel to drop the cached pages
/// via posix_fadvise(POSIX_FADV_DONTNEED). Failures are reported as warnings
/// rather than treated as fatal, since an inaccurate cold measurement is still
/// useful.
void
evictPageCache(const std::string& filename)
{
#if defined(__linux__)
    const int fd = ::open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
        OPENVDB_LOG_WARN("could not open " << filename << " to evict page cache ("
            << std::strerror(errno) << ")");
        return;
    }

    struct stat info;
    if (::fstat(fd, &info) != 0) {
        OPENVDB_LOG_WARN("could not stat " << filename << " to evict page cache ("
            << std::strerror(errno) << ")");
        ::close(fd);
        return;
    }

    if (::posix_fadvise(fd, 0, info.st_size, POSIX_FADV_DONTNEED) != 0) {
        OPENVDB_LOG_WARN("could not evict page cache for " << filename
            << " (" << std::strerror(errno) << ")");
    }

    ::close(fd);
#else
    OPENVDB_LOG_WARN("page cache eviction is not supported on this platform; "
        "reading " << filename << " with whatever is already cached");
#endif
}


/// @brief Prefetch the whole file into the operating system page cache.
/// @details This is the equivalent of "vmtouch -t <file>": it memory-maps the
/// file read-only and touches one byte of every page so that the kernel faults
/// each page into the page cache. The subsequent read therefore measures a warm
/// (fully cached) read rather than a cold read from disk. Failures are reported
/// as warnings rather than treated as fatal, since a partially warmed cache is
/// still a useful measurement.
void
prefetchPageCache(const std::string& filename)
{
#if defined(__linux__)
    const int fd = ::open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
        OPENVDB_LOG_WARN("could not open " << filename << " to prefetch page cache ("
            << std::strerror(errno) << ")");
        return;
    }

    struct stat info;
    if (::fstat(fd, &info) != 0) {
        OPENVDB_LOG_WARN("could not stat " << filename << " to prefetch page cache ("
            << std::strerror(errno) << ")");
        ::close(fd);
        return;
    }

    // An empty file has no pages to fault in.
    if (info.st_size == 0) {
        ::close(fd);
        return;
    }

    void* mapping = ::mmap(nullptr, info.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (mapping == MAP_FAILED) {
        OPENVDB_LOG_WARN("could not mmap " << filename << " to prefetch page cache ("
            << std::strerror(errno) << ")");
        ::close(fd);
        return;
    }

    // Touch one byte of every page so that the kernel faults each page into the
    // page cache. The accumulator is volatile so that the reads are not elided.
    const long pageSize = ::sysconf(_SC_PAGESIZE);
    const char* bytes = static_cast<const char*>(mapping);
    volatile char accumulator = 0;
    for (std::uintmax_t offset = 0; offset < std::uintmax_t(info.st_size);
        offset += std::uintmax_t(pageSize))
    {
        accumulator += bytes[offset];
    }

    if (::munmap(mapping, info.st_size) != 0) {
        OPENVDB_LOG_WARN("could not munmap " << filename << " after prefetch ("
            << std::strerror(errno) << ")");
    }

    ::close(fd);
#else
    OPENVDB_LOG_WARN("page cache prefetch is not supported on this platform; "
        "reading " << filename << " with whatever is already cached");
#endif
}


/// @brief Codec to use when writing an output file.
/// @details Legacy is the absence of any codec metadata; Compact tags each grid
/// with the "compact" codec and uses the interleaved file layout.
enum class OutputCodec { Legacy, Compact };


/// @brief The advanced zstd compression parameters that can be overridden, using
/// the same short aliases as the zstd CLI (e.g. "wlog" for the window log). The
/// codec reads each as an Int32 grid metadata value named "<prefix><alias>".
const std::vector<std::string>&
zstdParameterNames()
{
    static const std::vector<std::string> names =
        {"wlog", "hlog", "clog", "slog", "mml", "tlen", "strat"};
    return names;
}


/// @brief If @a arg is a "-zstd-..." compression parameter flag, parse it and
/// record the corresponding metadata key/value in @a zstdParameters, advancing
/// @a index past the consumed value. Returns true if @a arg was a (recognized)
/// zstd flag, whether or not parsing succeeded; false if it is not a zstd flag.
///
/// Recognized forms:
///   -zstd-<alias> V             -> key "zstd_<alias>"          (e.g. zstd_wlog)
///   -zstd-float<plane>-<alias> V -> key "zstd_float<plane>_<alias>"
/// where <alias> is one of zstdParameterNames(). The plain form applies to all
/// non-byte-planed (and non-float value) compression; the per-plane float form
/// applies to byte plane <plane> of float value types (plane 0 is the exponent).
bool
parseZstdParameterArg(const std::string& arg, int argc, char* argv[], int& index,
    std::map<std::string, int32_t>& zstdParameters)
{
    static const std::string prefix = "-zstd-";
    if (arg.compare(0, prefix.size(), prefix) != 0) return false;

    // The token after the "-zstd-" prefix, e.g. "wlog" or "float0-wlog".
    const std::string remainder = arg.substr(prefix.size());

    // Resolve the metadata-key prefix and the alias. A "floatN-" segment selects
    // the per-plane set; otherwise the plain "zstd_" set is used.
    std::string keyPrefix = "zstd_";
    std::string alias = remainder;
    if (remainder.compare(0, 5, "float") == 0) {
        const size_t dash = remainder.find('-');
        if (dash == std::string::npos) {
            OPENVDB_LOG_FATAL("malformed zstd option \"" << arg << "\"");
            usage();
        }
        const std::string plane = remainder.substr(5, dash - 5);
        keyPrefix = "zstd_float" + plane + "_";
        alias = remainder.substr(dash + 1);
    }

    const std::vector<std::string>& aliases = zstdParameterNames();
    if (std::find(aliases.begin(), aliases.end(), alias) == aliases.end()) {
        OPENVDB_LOG_FATAL("unknown zstd parameter in \"" << arg << "\"");
        usage();
    }

    if (index + 1 >= argc || !argv[index + 1]) {
        OPENVDB_LOG_FATAL("missing value after " << arg);
        usage();
    }
    zstdParameters[keyPrefix + alias] = int32_t(std::atoi(argv[index + 1]));
    ++index;
    return true;
}


/// @brief A single grid as it was stored on disk: its human-readable name
/// (e.g. "density[0]") and the name of the codec used to read it.
struct GridStorage
{
    std::string name;
    std::string codecName;
};


/// @brief Read a single VDB file once and return the elapsed times.
struct ReadResult
{
    double totalMilliseconds = 0.0;
    double ioMilliseconds = 0.0;
    // The file format version that the file was written with; 300 and above
    // is the interleaved layout, anything below is the legacy layout.
    uint32_t fileVersion = 0;
    // One entry per grid stored in the file, in the order it appears on disk.
    std::vector<GridStorage> gridStorage;
    // The grids that were read, retained so that they can be written out.
    openvdb::GridPtrVecPtr grids;
    // File-level metadata, retained so that it can be written out.
    openvdb::MetaMap::Ptr metadata;
};


ReadResult
readOnce(const std::string& filename, int bandwidth, int decompressThreads,
    int readFloatBytes)
{
    using Clock = std::chrono::steady_clock;

    openvdb::io::ReadOptions readOptions;
    // A non-positive value leaves the codec to pick its own concurrency.
    readOptions.decompressThreads = decompressThreads;

    const auto start = Clock::now();
    openvdb::io::File file(filename, bandwidth);
    file.open();

    // Lossy float read: attach a CompactScalarCodecReadOptions to each
    // compact-codec grid so the codec keeps only the leading readFloatBytes byte
    // planes and skips the rest. typeData is keyed by the codec's registered
    // name (e.g. "compact_floatGrid"), which is exactly the per-grid codec name
    // the file descriptors carry. Only meaningful in 1..3; 4 (or the default -1)
    // leaves the read lossless, so nothing is attached. The codec itself ignores
    // the option for value types that are not 4-byte floats.
    if (readFloatBytes >= 1 && readFloatBytes <= 3) {
        static const std::string compactPrefix = "compact_";
        for (auto it = file.beginName(); it != file.endName(); ++it) {
            const std::string codecName = it.codecName();
            if (codecName.compare(0, compactPrefix.size(), compactPrefix) != 0) continue;
            if (readOptions.typeData.find(codecName) != readOptions.typeData.end()) continue;
            auto typeData = std::make_shared<openvdb::codecs::CompactScalarCodecReadOptions>();
            typeData->readFloatBytes = readFloatBytes;
            readOptions.typeData[codecName] = typeData;
        }
    }

    openvdb::GridPtrVecPtr grids = file.getGrids(readOptions);
    const auto end = Clock::now();

    ReadResult result;
    result.totalMilliseconds = std::chrono::duration<double, std::milli>(end - start).count();
    result.ioMilliseconds = file.readIOTimeMilliseconds();
    result.fileVersion = file.fileVersion();

    // Capture the on-disk name and codec for each grid from its descriptor.
    for (auto it = file.beginName(); it != file.endName(); ++it) {
        GridStorage storage;
        storage.name = it.gridName();
        storage.codecName = it.codecName();
        result.gridStorage.push_back(storage);
    }

    result.grids = grids;
    result.metadata = file.getMetadata();

    file.close();
    return result;
}


/// @brief The result of writing an output file.
struct WriteResult
{
    double totalSeconds = 0.0;
    // The file format version that the grids were written with.
    uint32_t fileVersion = openvdb::OPENVDB_FILE_VERSION;
    // One entry per grid, mirroring how the grids were stored on disk.
    std::vector<GridStorage> gridStorage;
};


/// @brief Determine the name of the codec that the archive would use to write
/// the given grid under the chosen layout.
/// @details This mirrors the codec selection performed by Archive::writeGrids():
/// under the interleaved layout a grid's "codec" string metadata (if present)
/// is combined with its grid type to form the lookup key, and the registry
/// resolves that to the final codec name.
std::string
resolveCodecName(const openvdb::GridBase::ConstPtr& grid, bool interleavedLayout)
{
    std::string codecLookupName = grid->type();
    if (interleavedLayout) {
        if (auto codecMetadata = grid->getMetadata<openvdb::StringMetadata>("codec")) {
            codecLookupName = codecMetadata->str() + "_" + grid->type();
        }
    }
    if (openvdb::io::CodecRegistry::get(codecLookupName)) return codecLookupName;
    // No registered codec resolves to the bare grid type (legacy tree I/O).
    return grid->type();
}


/// @brief Reproduce the unique on-disk name that Archive::writeGrids() assigns
/// to each grid, so that the reported names match what is stored in the file.
std::vector<GridStorage>
deriveGridStorage(const openvdb::GridPtrVecPtr& grids, bool interleavedLayout)
{
    // Count how many grids share each name; duplicates (and empty names) get a
    // numeric suffix appended.
    std::map<std::string, int> nameCount;
    for (const openvdb::GridBase::Ptr& grid : *grids) {
        if (grid) ++nameCount[grid->getName()];
    }

    std::set<std::string> uniqueNames;
    std::vector<GridStorage> gridStorage;
    for (const openvdb::GridBase::Ptr& grid : *grids) {
        if (!grid) continue;

        std::string name = grid->getName();
        if (name.empty() || nameCount[name] > 1) {
            name = openvdb::io::GridDescriptor::addSuffix(name, 0);
        }
        for (int n = 1; uniqueNames.find(name) != uniqueNames.end(); ++n) {
            name = openvdb::io::GridDescriptor::addSuffix(grid->getName(), n);
        }
        uniqueNames.insert(name);

        GridStorage storage;
        storage.name = openvdb::io::GridDescriptor::nameAsString(name);
        storage.codecName = resolveCodecName(grid, interleavedLayout);
        gridStorage.push_back(storage);
    }
    return gridStorage;
}


/// @brief Write the given grids to an output file using the chosen codec.
WriteResult
writeGrids(const std::string& outputFilename, const openvdb::GridPtrVecPtr& grids,
    const openvdb::MetaMap::Ptr& metadata, OutputCodec codec,
    int64_t maxVoxelsPerChunk,
    const std::map<std::string, int32_t>& zstdParameters)
{
    using Clock = std::chrono::steady_clock;

    openvdb::io::WriteOptions writeOptions;
    for (const openvdb::GridBase::Ptr& grid : *grids) {
        if (codec == OutputCodec::Compact) {
            grid->insertMeta("codec", openvdb::StringMetadata("compact"));
            // A positive value overrides the codec's built-in chunk default;
            // a non-positive value leaves the codec to use its default.
            if (maxVoxelsPerChunk > 0) {
                grid->insertMeta("max_voxels_per_chunk",
                    openvdb::Int64Metadata(maxVoxelsPerChunk));
            }
            // Advanced zstd parameter overrides, attached as transient metadata
            // for the codec to read. Archive strips these keys before the grid
            // metadata is serialized, so they are not persisted in the file.
            for (const auto& [key, value] : zstdParameters) {
                grid->insertMeta(key, openvdb::Int32Metadata(value));
            }
        } else {
            // Legacy is the absence of any codec metadata.
            grid->removeMeta("codec");
        }
    }
    const bool interleavedLayout = (codec == OutputCodec::Compact);
    writeOptions.layout = interleavedLayout
        ? openvdb::io::WriteLayout::Interleaved
        : openvdb::io::WriteLayout::Legacy;

    const auto start = Clock::now();
    openvdb::io::File file(outputFilename);
    if (metadata) file.write(*grids, *metadata, writeOptions);
    else          file.write(*grids, openvdb::MetaMap(), writeOptions);
    const auto end = Clock::now();

    WriteResult result;
    result.totalSeconds =
        std::chrono::duration<double, std::milli>(end - start).count() / 1000.0;
    result.fileVersion = interleavedLayout
        ? openvdb::OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT
        : openvdb::OPENVDB_FILE_VERSION;
    result.gridStorage = deriveGridStorage(grids, interleavedLayout);
    return result;
}


/// @brief One reported operation: a single read or write of a file, with the
/// timing, on-disk layout, file size and per-grid storage information needed to
/// render an aligned summary line (plus optional indented per-grid lines).
struct OperationLine
{
    std::string action;     // "Read:" or "Write:"
    double seconds = 0.0;
    uint32_t fileVersion = 0;
    std::uintmax_t fileBytes = 0;
    std::string filename;
    std::string note;       // e.g. "(no page cache, unthrottled)"
    std::vector<GridStorage> gridStorage;
};


/// @brief Format a duration in seconds with three decimal places (millisecond
/// resolution), e.g. 34.518.
std::string
formatSeconds(double seconds)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << seconds;
    return stream.str();
}


/// @brief Human-readable name for an on-disk layout, derived from the file
/// format version. The interleaved layout was introduced at version 300;
/// anything earlier is the legacy layout.
std::string
layoutName(uint32_t fileVersion)
{
    return (fileVersion >= openvdb::OPENVDB_FILE_VERSION_INTERLEAVED_LAYOUT)
        ? "Interleaved" : "Legacy";
}


/// @brief Return the size of a file on disk in bytes, or 0 if it cannot be
/// determined.
std::uintmax_t
fileSizeOnDisk(const std::string& filename)
{
    std::error_code error;
    const std::uintmax_t bytes = std::filesystem::file_size(filename, error);
    return error ? 0 : bytes;
}


/// @brief Describe the read conditions for an operation's trailing note, e.g.
/// "(no page cache, unthrottled)" or "(page cache, 50 MB/s)".
std::string
conditionsNote(bool keepPageCache, int bandwidth)
{
    std::ostringstream stream;
    stream << "(" << (keepPageCache ? "page cache" : "no page cache") << ", ";
    if (bandwidth > 0) stream << bandwidth << " MB/s";
    else               stream << "unthrottled";
    stream << ")";
    return stream.str();
}


/// @brief Print the accumulated operations as aligned summary lines.
/// @details Each operation prints one line of the form
/// "Read:  7.523s  Legacy  2790543678 bytes  /path/file.vdb  (note)", with the
/// columns aligned across all operations. When a file holds more than one grid,
/// the grid names and their codecs are listed on indented lines beneath it.
void
printOperations(const std::vector<OperationLine>& operations)
{
    auto leftCell = [](const std::string& text, size_t width) {
        return text + std::string(width - text.size(), ' ');
    };
    auto rightCell = [](const std::string& text, size_t width) {
        return std::string(width - text.size(), ' ') + text;
    };

    // Pre-format every cell so that column widths can be computed up front.
    struct Cells { std::string action, time, layout, bytes; };
    std::vector<Cells> cells;
    cells.reserve(operations.size());

    size_t actionWidth = 0, timeWidth = 0, layoutWidth = 0, bytesWidth = 0;
    size_t filenameWidth = 0, gridNameWidth = 0;
    for (const OperationLine& operation : operations) {
        Cells cell;
        cell.action = operation.action;
        cell.time = formatSeconds(operation.seconds) + "s";
        cell.layout = layoutName(operation.fileVersion);
        cell.bytes = std::to_string(operation.fileBytes) + " bytes";
        cells.push_back(cell);

        actionWidth = std::max(actionWidth, cell.action.size());
        timeWidth = std::max(timeWidth, cell.time.size());
        layoutWidth = std::max(layoutWidth, cell.layout.size());
        bytesWidth = std::max(bytesWidth, cell.bytes.size());
        filenameWidth = std::max(filenameWidth, operation.filename.size());

        // Only files with multiple grids list their grids on indented lines.
        if (operation.gridStorage.size() > 1) {
            for (const GridStorage& grid : operation.gridStorage) {
                gridNameWidth = std::max(gridNameWidth, grid.name.size());
            }
        }
    }

    for (size_t i = 0; i < operations.size(); ++i) {
        const OperationLine& operation = operations[i];
        std::cout << leftCell(cells[i].action, actionWidth) << "  "
                  << rightCell(cells[i].time, timeWidth) << "  "
                  << leftCell(cells[i].layout, layoutWidth) << "  "
                  << rightCell(cells[i].bytes, bytesWidth) << "  "
                  << leftCell(operation.filename, filenameWidth) << "  "
                  << operation.note << "\n";

        if (operation.gridStorage.size() > 1) {
            for (const GridStorage& grid : operation.gridStorage) {
                std::cout << "    " << leftCell(grid.name, gridNameWidth) << "  "
                          << grid.codecName << "\n";
            }
        }
    }
}

} // unnamed namespace


int
main(int argc, char *argv[])
{
    OPENVDB_START_THREADSAFE_STATIC_WRITE
    gProgName = argv[0];
    if (const char* ptr = ::strrchr(gProgName, '/')) gProgName = ptr + 1;
    OPENVDB_FINISH_THREADSAFE_STATIC_WRITE

    int exitStatus = EXIT_SUCCESS;

    openvdb::logging::initialize(argc, argv);
    openvdb::initialize();

    // Parse command-line arguments.
    int bandwidth = -1;
    int iterations = 1;
    bool keepPageCache = false;
    bool version = false;
    std::string outputFilename;
    OutputCodec codec = OutputCodec::Legacy;
    int64_t maxVoxelsPerChunk = -1; // -1 = use the codec's built-in default
    int decompressThreads = -1; // -1 = let the codec pick its own concurrency
    // Lossy float read: number of leading byte planes to keep when reading a
    // 4-byte float compact-codec grid (1..4). -1 (the default) and 4 both read
    // every plane (lossless).
    int readFloatBytes = -1;
    // Advanced zstd compression parameter overrides for the compact codec,
    // collected as metadata-key -> value (e.g. "zstd_wlog" or "zstd_float0_wlog").
    // The codec reads these from the grid metadata at write time.
    std::map<std::string, int32_t> zstdParameters;
    std::vector<std::string> filenames;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (!arg.empty() && arg[0] == '-') {
            if (arg == "-bandwidth") {
                if (i + 1 < argc && argv[i + 1]) {
                    bandwidth = std::atoi(argv[i + 1]);
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing value after -bandwidth");
                    usage();
                }
            } else if (arg == "-iterations") {
                if (i + 1 < argc && argv[i + 1]) {
                    iterations = std::atoi(argv[i + 1]);
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing value after -iterations");
                    usage();
                }
            } else if (arg == "-page-cache") {
                keepPageCache = true;
            } else if (arg == "-output") {
                if (i + 1 < argc && argv[i + 1]) {
                    outputFilename = argv[i + 1];
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing filename after " << arg);
                    usage();
                }
            } else if (arg == "-codec") {
                if (i + 1 < argc && argv[i + 1]) {
                    const std::string value = argv[i + 1];
                    if (value == "legacy") {
                        codec = OutputCodec::Legacy;
                    } else if (value == "compact") {
                        codec = OutputCodec::Compact;
                    } else {
                        OPENVDB_LOG_FATAL("unknown codec \"" << value
                            << "\"; expected \"legacy\" or \"compact\"");
                        usage();
                    }
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing value after " << arg);
                    usage();
                }
            } else if (arg == "-max-voxels-per-chunk" || arg == "--max-voxels-per-chunk") {
                if (i + 1 < argc && argv[i + 1]) {
                    maxVoxelsPerChunk = std::atoll(argv[i + 1]);
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing value after " << arg);
                    usage();
                }
            } else if (arg == "-decompress-threads" || arg == "--decompress-threads") {
                if (i + 1 < argc && argv[i + 1]) {
                    decompressThreads = std::atoi(argv[i + 1]);
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing value after " << arg);
                    usage();
                }
            } else if (arg == "-read-float-bytes" || arg == "--read-float-bytes") {
                if (i + 1 < argc && argv[i + 1]) {
                    readFloatBytes = std::atoi(argv[i + 1]);
                    ++i;
                } else {
                    OPENVDB_LOG_FATAL("missing value after " << arg);
                    usage();
                }
            } else if (arg == "-h" || arg == "-help" || arg == "--help") {
                usage(EXIT_SUCCESS);
            } else if (arg == "-version" || arg == "--version") {
                version = true;
            } else if (parseZstdParameterArg(arg, argc, argv, i, zstdParameters)) {
                // handled (value consumed inside the helper)
            } else {
                OPENVDB_LOG_FATAL("\"" << arg << "\" is not a valid option");
                usage();
            }
        } else if (!arg.empty()) {
            filenames.push_back(arg);
        }
    }

    if (version) {
        std::cout << "OpenVDB library version: "
            << openvdb::getLibraryAbiVersionString() << "\n";
        std::cout << "OpenVDB file format version: "
            << openvdb::OPENVDB_FILE_VERSION << std::endl;
        if (filenames.empty()) return EXIT_SUCCESS;
    }

    if (filenames.empty()) {
        OPENVDB_LOG_FATAL("missing input OpenVDB filename");
        usage();
    }
    if (iterations < 1) {
        OPENVDB_LOG_FATAL("number of iterations must be at least 1");
        usage();
    }
    if (readFloatBytes != -1 && (readFloatBytes < 1 || readFloatBytes > 4)) {
        OPENVDB_LOG_FATAL("-read-float-bytes must be between 1 and 4");
        usage();
    }
    if (!outputFilename.empty() && filenames.size() > 1) {
        OPENVDB_LOG_FATAL("-output requires a single input file");
        usage();
    }

    const std::string note = conditionsNote(keepPageCache, bandwidth);

    // Accumulate one line per operation (each read iteration and the optional
    // write) so that they can be printed as aligned summary lines.
    std::vector<OperationLine> operations;

    // The most recent read result, retained for an optional -output write.
    ReadResult lastResult;

    for (const std::string& filename : filenames) {
        try {
            for (int iteration = 0; iteration < iterations; ++iteration) {
                // Evict the page cache before each iteration for a cold read,
                // or prefetch the whole file into it for a warm read if the
                // user asked to keep the page cache.
                if (keepPageCache) prefetchPageCache(filename);
                else               evictPageCache(filename);

                ReadResult result = readOnce(filename, bandwidth, decompressThreads, readFloatBytes);

                // Each iteration prints its own line with its own time; there
                // is no averaging across iterations.
                OperationLine operation;
                operation.action = "Read:";
                operation.seconds = result.totalMilliseconds / 1000.0;
                operation.fileVersion = result.fileVersion;
                operation.fileBytes = fileSizeOnDisk(filename);
                operation.filename = filename;
                operation.note = note;
                operation.gridStorage = result.gridStorage;
                operations.push_back(operation);

                lastResult = std::move(result);
            }
        }
        catch (const std::exception& e) {
            OPENVDB_LOG_ERROR(e.what() << " (" << filename << ")");
            exitStatus = EXIT_FAILURE;
        }
    }

    // Optionally write the grids back out to a new file.
    if (!outputFilename.empty() && lastResult.grids) {
        try {
            const WriteResult writeResult = writeGrids(
                outputFilename, lastResult.grids, lastResult.metadata, codec,
                maxVoxelsPerChunk, zstdParameters);

            OperationLine operation;
            operation.action = "Write:";
            operation.seconds = writeResult.totalSeconds;
            operation.fileVersion = writeResult.fileVersion;
            operation.fileBytes = fileSizeOnDisk(outputFilename);
            operation.filename = outputFilename;
            operation.note = note;
            operation.gridStorage = writeResult.gridStorage;
            operations.push_back(operation);
        }
        catch (const std::exception& e) {
            OPENVDB_LOG_ERROR(e.what() << " (" << outputFilename << ")");
            exitStatus = EXIT_FAILURE;
        }
    }

    if (!operations.empty()) printOperations(operations);

    return exitStatus;
}
