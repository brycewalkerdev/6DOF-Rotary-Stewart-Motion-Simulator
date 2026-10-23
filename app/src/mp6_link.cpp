/*
 * MP6 raw byte links. See mp6_link.h for evidence and ownership notes.
 */

#include "mp6_link.h"

#include "automation.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
// Keep Win32's min/max macros out of std::min/std::max and numeric_limits.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace mp6 {

// ---------------------------------------------------------------------------
// MemoryLink
// ---------------------------------------------------------------------------

MemoryLink::MemoryLink(std::size_t rxCapacity, std::size_t txFrameCapacity)
    : rxCapacity_(std::max<std::size_t>(rxCapacity, 64)),
      txFrameCapacity_(std::max<std::size_t>(txFrameCapacity, 8)) {}

bool MemoryLink::open(const std::string& description) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (open_) {
        return false;
    }
    open_ = true;
    readFailing_ = false;
    description_ = description;
    lastError_.clear();
    ++stats_.openCount;
    return true;
}

void MemoryLink::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (open_) {
        open_ = false;
        ++stats_.closeCount;
    }
}

bool MemoryLink::isOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return open_;
}

bool MemoryLink::write(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr && length != 0) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_ || writeFailing_) {
        ++stats_.writeFailures;
        lastError_ = writeFailing_ ? "injected write failure" : "memory link closed";
        return false;
    }
    // Count only successful writes. A failed write is a writeFailures event,
    // not a transmitted frame (the old ordering overstated txWrites).
    ++stats_.txWrites;
    if (length == 0) {
        return true;
    }
    if (tx_.size() >= txFrameCapacity_) {
        tx_.pop_front();
        ++stats_.txLogDroppedFrames;
    }
    tx_.emplace_back(data, data + length);
    stats_.txBytes += length;
    return true;
}

std::size_t MemoryLink::read(std::uint8_t* destination,
                             std::size_t capacity) {
    if (destination == nullptr || capacity == 0) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (readFailing_) {
        ++stats_.readErrors;
        lastError_ = "injected terminal read failure";
        return 0;
    }
    const std::size_t count = std::min(capacity, rx_.size());
    for (std::size_t i = 0; i < count; ++i) {
        destination[i] = rx_.front();
        rx_.pop_front();
    }
    return count;
}

LinkStats MemoryLink::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

bool MemoryLink::terminalFailure() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return readFailing_;
}

std::string MemoryLink::lastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
}

void MemoryLink::inject(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < length; ++i) {
        if (rx_.size() >= rxCapacity_) {
            rx_.pop_front();
            ++stats_.rxQueueDroppedBytes;
        }
        rx_.push_back(data[i]);
    }
}

void MemoryLink::inject(const std::vector<std::uint8_t>& bytes) {
    inject(bytes.data(), bytes.size());
}

std::vector<std::vector<std::uint8_t>> MemoryLink::takeWrites() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::vector<std::uint8_t>> out;
    out.reserve(tx_.size());
    while (!tx_.empty()) {
        out.push_back(std::move(tx_.front()));
        tx_.pop_front();
    }
    return out;
}

std::size_t MemoryLink::pendingWrites() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tx_.size();
}

void MemoryLink::clearWrites() {
    std::lock_guard<std::mutex> lock(mutex_);
    tx_.clear();
}

void MemoryLink::setWriteFailing(bool failing) {
    std::lock_guard<std::mutex> lock(mutex_);
    writeFailing_ = failing;
}

void MemoryLink::setReadFailing(bool failing) {
    std::lock_guard<std::mutex> lock(mutex_);
    readFailing_ = failing;
    if (failing) {
        lastError_ = "injected terminal read failure";
    }
}

// ---------------------------------------------------------------------------
// SerialLink
// ---------------------------------------------------------------------------

#ifdef _WIN32

struct SerialLink::Impl {
    explicit Impl(int timeout, std::size_t capacity)
        : timeoutMs(timeout), rxCapacity(std::max<std::size_t>(capacity, 64)) {}

    ~Impl() { shutdown(); }

    void shutdown() {
        if (!open.exchange(false)) {
            stopping = false;
            return;  // idempotent: a never-opened (or already closed) link
        }
        stopping = true;
        if (handle != INVALID_HANDLE_VALUE) {
            // Cancel a pending synchronous ReadFile in the reader thread. The
            // handle is closed only after the reader has returned.
            CancelIoEx(handle, nullptr);
        }
        if (reader.joinable()) {
            reader.join();
        }
        if (handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
            handle = INVALID_HANDLE_VALUE;
        }
        {
            std::lock_guard<std::mutex> lock(rxMutex);
            rx.clear();
        }
        stopping = false;
        ++closeCount;
    }

    void setError(const std::string& text) {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastError = text;
    }

    std::string getError() const {
        std::lock_guard<std::mutex> lock(errorMutex);
        return lastError;
    }

    void readerLoop() {
        std::uint8_t chunk[512];
        while (!stopping.load(std::memory_order_relaxed)) {
            DWORD got = 0;
            const BOOL ok = ReadFile(handle, chunk, sizeof(chunk), &got, nullptr);
            if (!ok) {
                const DWORD error = GetLastError();
                if (error == ERROR_OPERATION_ABORTED ||
                    error == ERROR_INVALID_HANDLE) {
                    break;
                }
                if (error == ERROR_IO_PENDING) {
                    continue;
                }
                // Read timeouts surface as a successful zero-byte read on
                // Windows; a genuine failure ends the reader and marks the
                // transport terminal. The session observes this and stops;
                // close()/join() stay with the owning thread, never here.
                ++readErrors;
                readFailed = true;
                setError("ReadFile failed (" + std::to_string(error) + ")");
                break;
            }
            if (got == 0) {
                continue;  // read timeout
            }
            rxBytes += got;
            std::lock_guard<std::mutex> lock(rxMutex);
            for (DWORD i = 0; i < got; ++i) {
                if (rx.size() >= rxCapacity) {
                    rx.pop_front();
                    ++rxQueueDropped;
                }
                rx.push_back(chunk[i]);
            }
        }
    }

    HANDLE handle = INVALID_HANDLE_VALUE;
    std::atomic<bool> open{false};
    std::atomic<bool> stopping{false};
    std::atomic<bool> readFailed{false};
    std::thread reader;
    std::mutex ioMutex;
    std::mutex rxMutex;
    mutable std::mutex errorMutex;
    std::deque<std::uint8_t> rx;
    std::size_t rxCapacity;
    int timeoutMs;
    std::string port;
    std::string lastError;

    std::atomic<std::uint64_t> openCount{0};
    std::atomic<std::uint64_t> closeCount{0};
    std::atomic<std::uint64_t> txWrites{0};
    std::atomic<std::uint64_t> txBytes{0};
    std::atomic<std::uint64_t> rxBytes{0};
    std::atomic<std::uint64_t> writeFailures{0};
    std::atomic<std::uint64_t> readErrors{0};
    std::atomic<std::uint64_t> rxQueueDropped{0};
};

#else

struct SerialLink::Impl {
    explicit Impl(int timeout, std::size_t capacity)
        : timeoutMs(timeout), rxCapacity(capacity) {}
    void* handle = nullptr;
    std::atomic<bool> open{false};
    std::atomic<bool> stopping{false};
    std::atomic<bool> readFailed{false};
    std::thread reader;
    std::mutex ioMutex;
    std::mutex rxMutex;
    mutable std::mutex errorMutex;
    std::deque<std::uint8_t> rx;
    std::size_t rxCapacity;
    int timeoutMs;
    std::string port;
    std::string lastError;
    std::atomic<std::uint64_t> openCount{0};
    std::atomic<std::uint64_t> closeCount{0};
    std::atomic<std::uint64_t> txWrites{0};
    std::atomic<std::uint64_t> txBytes{0};
    std::atomic<std::uint64_t> rxBytes{0};
    std::atomic<std::uint64_t> writeFailures{0};
    std::atomic<std::uint64_t> readErrors{0};
    std::atomic<std::uint64_t> rxQueueDropped{0};

    void shutdown() {
        if (open.exchange(false)) {
            ++closeCount;
        }
    }
    void setError(const std::string& text) {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastError = text;
    }
    std::string getError() const {
        std::lock_guard<std::mutex> lock(errorMutex);
        return lastError;
    }
    void readerLoop() {}
};

#endif

SerialLink::SerialLink(int timeoutMs, std::size_t rxCapacity)
    : impl_(new Impl(timeoutMs, rxCapacity)) {}

SerialLink::~SerialLink() {
    close();
    delete impl_;
}

bool SerialLink::open(const char* port, int baud) {
    if (port == nullptr || port[0] == '\0') {
        impl_->setError("empty port name");
        return false;
    }
    if (IsDocumentationMode()) {
        // Rejected before any device access, matching the app-wide policy in
        // automation.h. The refusal is intentionally not overridable.
        impl_->setError("documentation mode forbids serial device access");
        return false;
    }
#ifdef _WIN32
    std::lock_guard<std::mutex> io(impl_->ioMutex);
    if (impl_->open.load()) {
        impl_->setError("port already open");
        return false;
    }
    std::string device(port);
    if (device.rfind("\\\\.\\", 0) != 0) {
        device = "\\\\.\\" + device;
    }
    HANDLE handle = CreateFileA(
        device.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        impl_->setError("CreateFile failed (" +
                        std::to_string(static_cast<unsigned long>(GetLastError())) +
                        ")");
        return false;
    }

    SetupComm(handle, 4096, 4096);

    DCB dcb;
    std::memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle, &dcb)) {
        impl_->setError("GetCommState failed");
        CloseHandle(handle);
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baud > 0 ? baud : kDefaultBaud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;   // DTR low
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;   // RTS low
    dcb.fAbortOnError = FALSE;
    if (!SetCommState(handle, &dcb)) {
        impl_->setError("SetCommState failed");
        CloseHandle(handle);
        return false;
    }

    COMMTIMEOUTS timeouts;
    std::memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(impl_->timeoutMs);
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = static_cast<DWORD>(impl_->timeoutMs);
    if (!SetCommTimeouts(handle, &timeouts)) {
        impl_->setError("SetCommTimeouts failed");
        CloseHandle(handle);
        return false;
    }

    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    impl_->handle = handle;
    impl_->port = port;
    impl_->lastError.clear();
    impl_->stopping = false;
    impl_->readFailed = false;
    impl_->open = true;
    ++impl_->openCount;
    impl_->reader = std::thread([this] { impl_->readerLoop(); });
    return true;
#else
    (void)baud;
    impl_->setError("serial link requires Windows");
    return false;
#endif
}

void SerialLink::close() {
#ifdef _WIN32
    std::lock_guard<std::mutex> io(impl_->ioMutex);
    impl_->shutdown();
#else
    impl_->shutdown();
#endif
}

bool SerialLink::isOpen() const {
    return impl_->open.load();
}

bool SerialLink::write(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr && length != 0) {
        return false;
    }
    if (!impl_->open.load()) {
        ++impl_->writeFailures;
        impl_->setError("write on closed link");
        return false;
    }
#ifdef _WIN32
    std::lock_guard<std::mutex> io(impl_->ioMutex);
    if (!impl_->open.load()) {
        ++impl_->writeFailures;
        impl_->setError("write on closed link");
        return false;
    }
    std::size_t offset = 0;
    while (offset < length) {
        DWORD written = 0;
        const DWORD remaining =
            static_cast<DWORD>(std::min<std::size_t>(length - offset, 0x7FFFFFFF));
        if (!WriteFile(impl_->handle, data + offset, remaining, &written,
                       nullptr)) {
            ++impl_->writeFailures;
            impl_->setError("WriteFile failed (" +
                            std::to_string(static_cast<unsigned long>(
                                GetLastError())) +
                            ")");
            return false;
        }
        if (written == 0) {
            ++impl_->writeFailures;
            impl_->setError("WriteFile wrote zero bytes");
            return false;
        }
        offset += written;
    }
    ++impl_->txWrites;
    impl_->txBytes += length;
    return true;
#else
    (void)length;
    ++impl_->writeFailures;
    impl_->setError("serial link requires Windows");
    return false;
#endif
}

bool SerialLink::terminalFailure() const {
    return impl_->open.load() && impl_->readFailed.load();
}

std::size_t SerialLink::read(std::uint8_t* destination,
                             std::size_t capacity) {
    if (destination == nullptr || capacity == 0) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(impl_->rxMutex);
    const std::size_t count = std::min(capacity, impl_->rx.size());
    for (std::size_t i = 0; i < count; ++i) {
        destination[i] = impl_->rx.front();
        impl_->rx.pop_front();
    }
    return count;
}

LinkStats SerialLink::stats() const {
    LinkStats out;
    out.openCount = impl_->openCount.load();
    out.closeCount = impl_->closeCount.load();
    out.txWrites = impl_->txWrites.load();
    out.txBytes = impl_->txBytes.load();
    out.rxBytes = impl_->rxBytes.load();
    out.writeFailures = impl_->writeFailures.load();
    out.readErrors = impl_->readErrors.load();
    out.rxQueueDroppedBytes = impl_->rxQueueDropped.load();
    out.txLogDroppedFrames = 0;
    return out;
}

std::string SerialLink::lastError() const {
    return impl_->getError();
}

const char* SerialLink::portName() const {
    return impl_->port.c_str();
}

}  // namespace mp6
