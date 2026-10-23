#pragma once
/*
 * MP6 raw byte links.
 *
 * MP6 is not the ESP32/COBS backend. It is an 18-byte host frame / variable
 * length telemetry byte stream at 115200 8N1 with no COBS, no ASCII lines and
 * no ITransport contract. This header therefore defines a deliberately narrow
 * byte pump rather than extending transport.h.
 *
 * Evidence:
 *   motum_resources/motum-replacement-handoff/PROTOCOL.md
 *   recovered-source/MotSim/frm_Main.cs:32848-32864 (serial settings)
 *   recovered-source/MotSim/frm_Main.cs:31954-32424 (byte-oriented RX)
 *   recovered-source/MotSim/frm_Main.cs:32503-32630 (TX gating)
 *
 * Ownership:
 *   - A Link owns no protocol state and never increments the MP6 sequence.
 *   - mp6::Session is the single writer. Nothing else may call write() while a
 *     session is attached.
 *   - SerialLink opens a COM port only on explicit user action and refuses to
 *     open in the process-immutable documentation mode.
 */

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace mp6 {

// Monotonic byte-level counters. SerialLink reads these from atomics; the
// in-memory link keeps plain members. stats() returns a snapshot by value so
// callers never observe a torn multi-field record.
struct LinkStats {
    std::uint64_t openCount = 0;
    std::uint64_t closeCount = 0;
    std::uint64_t txWrites = 0;
    std::uint64_t txBytes = 0;
    std::uint64_t rxBytes = 0;
    std::uint64_t writeFailures = 0;
    std::uint64_t readErrors = 0;
    std::uint64_t rxQueueDroppedBytes = 0;  // bounded RX buffer overflow
    std::uint64_t txLogDroppedFrames = 0;   // in-memory link only
};

class Link {
public:
    virtual ~Link() = default;

    virtual bool isOpen() const = 0;

    // Writes one complete host frame (or any contiguous byte run). Returns
    // false on any I/O failure; the session treats that as a motion stop.
    virtual bool write(const std::uint8_t* data, std::size_t length) = 0;

    // Copies up to capacity bytes that arrived since the last call. Returns 0
    // when nothing is available. Must not block longer than the configured
    // device timeout.
    virtual std::size_t read(std::uint8_t* destination,
                             std::size_t capacity) = 0;

    virtual LinkStats stats() const = 0;

    // True when the transport has hit an unrecoverable condition (for example
    // the serial reader thread ended on a device error). The session stops
    // promptly and leaves close()/join() to the owner. Default false keeps
    // replay and test links source-compatible.
    virtual bool terminalFailure() const { return false; }

    // Optional human-readable detail for the last open/write failure.
    virtual std::string lastError() const { return {}; }
};

// In-memory / replay link. It is the default for tests and for app startup
// before any user-initiated connect. Bytes injected with inject() are handed
// back by read(); every write() is appended to a bounded frame log for
// command-order assertions.
class MemoryLink final : public Link {
public:
    explicit MemoryLink(std::size_t rxCapacity = 8192,
                        std::size_t txFrameCapacity = 4096);

    // Simulates an explicit user connect. open("...") text is informational.
    bool open(const std::string& description = "memory");
    void close();
    bool isOpen() const override;

    bool write(const std::uint8_t* data, std::size_t length) override;
    std::size_t read(std::uint8_t* destination,
                     std::size_t capacity) override;
    LinkStats stats() const override;
    bool terminalFailure() const override;
    std::string lastError() const override;

    // Test/replay input. Bytes are appended to the bounded RX queue.
    void inject(const std::uint8_t* data, std::size_t length);
    void inject(const std::vector<std::uint8_t>& bytes);

    // Captured host writes, oldest first. A write that crosses frames is
    // still stored as one entry; tests that assert frame order inject whole
    // 18-byte frames (which is what Session does).
    std::vector<std::vector<std::uint8_t>> takeWrites();
    std::size_t pendingWrites() const;
    void clearWrites();

    // Forces write() to fail without closing the link (write-failure tests).
    void setWriteFailing(bool failing);

    // Simulates a terminal reader failure. read() returns no bytes and
    // terminalFailure() stays true until the flag is cleared or the link is
    // reopened. Tests use this to prove the session stops without closing the
    // link from inside a read path.
    void setReadFailing(bool failing);

private:
    std::size_t rxCapacity_;
    std::size_t txFrameCapacity_;
    mutable std::mutex mutex_;
    bool open_ = false;
    bool writeFailing_ = false;
    bool readFailing_ = false;
    std::string description_;
    std::string lastError_;
    std::deque<std::uint8_t> rx_;
    std::deque<std::vector<std::uint8_t>> tx_;
    LinkStats stats_{};
};

/*
 * Raw Win32 serial link for the unmodified MOTUM Mega.
 *
 * Settings are fixed to the recovered host values: 115200 baud, 8 data bits,
 * no parity, one stop bit, no flow control, DTR and RTS held inactive (low),
 * 250 ms read/write timeouts, binary byte stream. There is no COBS, no line
 * parser and no protocol logic here.
 *
 * Opening the port may reset a Mega because DTR is asserted by some USB
 * bridges during CreateFile. The guard in SerialLink::open() refuses to touch
 * the device when STEWART_DOCUMENTATION_MODE is set, and this class is never
 * constructed by the app until the user asks for MP6. It is not opened during
 * tests unless a test explicitly does so; the test suite uses MemoryLink.
 */
class SerialLink final : public Link {
public:
    static constexpr int kDefaultBaud = 115200;
    static constexpr int kDefaultTimeoutMs = 250;
    static constexpr std::size_t kDefaultRxCapacity = 8192;

    explicit SerialLink(int timeoutMs = kDefaultTimeoutMs,
                        std::size_t rxCapacity = kDefaultRxCapacity);
    ~SerialLink() override;

    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    // port may be "COM7" or "\\\\.\\COM7". Returns false and fills lastError()
    // on guard rejection or Win32 failure. A failed open leaves the link
    // closed and never throws.
    bool open(const char* port, int baud = kDefaultBaud);
    void close();
    bool isOpen() const override;

    bool write(const std::uint8_t* data, std::size_t length) override;
    std::size_t read(std::uint8_t* destination,
                     std::size_t capacity) override;
    LinkStats stats() const override;
    bool terminalFailure() const override;
    std::string lastError() const override;

    const char* portName() const;

private:
    struct Impl;
    Impl* impl_;
};

}  // namespace mp6
