// Copyright Contributors to the OpenVDB Project
// SPDX-License-Identifier: Apache-2.0
//
/// @file ThrottledStreamBuf.h
///
/// @brief A std::streambuf adaptor that limits read throughput and records the
/// wall-clock time spent reading, for assessing I/O performance.

#ifndef OPENVDB_IO_THROTTLEDSTREAMBUF_HAS_BEEN_INCLUDED
#define OPENVDB_IO_THROTTLEDSTREAMBUF_HAS_BEEN_INCLUDED

#include <openvdb/version.h>
#include <chrono>
#include <cstdint>
#include <ios>
#include <streambuf>
#include <thread>


namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {
namespace io {

/// @brief A read-only stream buffer that wraps another stream buffer, throttles
/// the rate at which bytes can be read from it, and accumulates the time spent
/// reading (including the time spent sleeping to enforce the throttle).
///
/// @details This is intended for performance assessment: wrapping a file's
/// stream buffer at a fixed bytes-per-second rate simulates reading from a
/// slower device (for example, a cold network filesystem). The accumulated read
/// time approximates the wall-clock time blocked on I/O, which can then be
/// compared against the total read time to estimate overlapped compute.
///
/// The buffer is seek-safe: it delegates @c seekoff and @c seekpos to the
/// source buffer, which is required because the OpenVDB readers seek the input
/// stream. To keep seeking simple, @c underflow reads only a single byte at a
/// time (so the get area never holds more than one buffered byte).
class ThrottledStreamBuf : public std::streambuf
{
public:
    /// @param source         The underlying stream buffer to read from.
    /// @param bytesPerSecond  The maximum read throughput in bytes per second.
    ///                        A value @c <= 0 disables throttling (reads still
    ///                        timed, but no sleeping is performed).
    ThrottledStreamBuf(std::streambuf* source, double bytesPerSecond)
        : mSource(source)
        , mBytesPerSecond(bytesPerSecond)
    {
    }

    /// @brief Return the total wall-clock time spent reading, in milliseconds.
    /// @details Includes the time spent sleeping to enforce the throttle, since
    /// that sleep represents the simulated slow transfer.
    double readTimeMilliseconds() const
    {
        return std::chrono::duration<double, std::milli>(mReadTime).count();
    }

protected:
    // Bulk read path used by std::istream::read() via sgetn().
    std::streamsize xsgetn(char* destination, std::streamsize length) override
    {
        std::streamsize total = 0;
        // Drain any single byte buffered by a prior underflow()/peek().
        if (gptr() < egptr()) {
            *destination++ = *gptr();
            gbump(1);
            ++total;
            --length;
        }
        // Read the remainder straight from the source, timed and throttled.
        if (length > 0) {
            total += this->readFromSource(destination, length);
        }
        return total;
    }

    // Single-byte read path used by get()/peek(). No read-ahead is performed so
    // that seeking remains straightforward (the get area holds at most one byte).
    int_type underflow() override
    {
        if (gptr() < egptr()) {
            return traits_type::to_int_type(*gptr());
        }
        char byte;
        if (this->readFromSource(&byte, 1) != 1) {
            return traits_type::eof();
        }
        mBuffer = byte;
        setg(&mBuffer, &mBuffer, &mBuffer + 1);
        return traits_type::to_int_type(mBuffer);
    }

    // Delegate relative/absolute seeks to the source, accounting for any byte
    // buffered in the get area, and discard that buffered byte.
    pos_type seekoff(off_type offset, std::ios_base::seekdir direction,
        std::ios_base::openmode which) override
    {
        // The source is ahead of the logical read position by any buffered byte.
        const std::streamsize buffered = egptr() - gptr();
        if (direction == std::ios_base::cur) {
            offset -= buffered;
        }
        setg(nullptr, nullptr, nullptr);
        return mSource->pubseekoff(offset, direction, which);
    }

    pos_type seekpos(pos_type position, std::ios_base::openmode which) override
    {
        setg(nullptr, nullptr, nullptr);
        return mSource->pubseekpos(position, which);
    }

private:
    std::streamsize readFromSource(char* destination, std::streamsize length)
    {
        const auto start = std::chrono::steady_clock::now();
        if (!mStarted) {
            mStartTime = start;
            mStarted = true;
        }
        const std::streamsize actual = mSource->sgetn(destination, length);
        if (mBytesPerSecond > 0 && actual > 0) {
            // The throttle enforces a ceiling on the average throughput, not added
            // latency. Rather than sleeping a fresh interval on every read (whose
            // OS-scheduler overshoots would accumulate across the many reads of a
            // file), sleep against a cumulative deadline: the wall-clock time by
            // which the total bytes read so far should have arrived at the target
            // rate. If a previous sleep overshot, this read's deadline has already
            // passed and it sleeps less (or not at all), so the error is corrected
            // rather than compounded. A source already slower than the target rate
            // (for example a cold filesystem) never reaches its deadline and so is
            // never throttled.
            mTotalBytes += static_cast<uint64_t>(actual);
            const std::chrono::duration<double> targetElapsed(
                static_cast<double>(mTotalBytes) / mBytesPerSecond);
            const auto deadline = mStartTime
                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    targetElapsed);
            if (deadline > std::chrono::steady_clock::now()) {
                std::this_thread::sleep_until(deadline);
            }
        }
        mReadTime += std::chrono::steady_clock::now() - start;
        return actual;
    }

    std::streambuf* mSource;
    double mBytesPerSecond;
    char mBuffer{};
    std::chrono::nanoseconds mReadTime{0};
    std::chrono::steady_clock::time_point mStartTime{};
    uint64_t mTotalBytes{0};
    bool mStarted{false};
};

} // namespace io
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb

#endif // OPENVDB_IO_THROTTLEDSTREAMBUF_HAS_BEEN_INCLUDED
