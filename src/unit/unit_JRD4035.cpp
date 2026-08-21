/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_JRD4035.cpp
  @brief JRD-4035 (magicRF M100) UHF-RFID Unit for M5UnitUnified
*/
#include "unit_JRD4035.hpp"

#include <string>

#include <M5Utility.hpp>

using namespace m5::unit::jrd4035;
using namespace m5::unit::types;
using namespace m5::utility::mmh3;

namespace {
// Command codes
constexpr uint8_t CMD_MODULE_INFORMATION{0x03};
constexpr uint8_t CMD_MULTIPLE_POLLING{0x27};
constexpr uint8_t CMD_STOP_POLLING{0x28};
constexpr uint8_t CMD_GET_QUERY{0x0D};
constexpr uint8_t CMD_SET_QUERY{0x0E};
constexpr uint8_t CMD_SET_REGION{0x07};
constexpr uint8_t CMD_GET_REGION{0x08};
constexpr uint8_t CMD_GET_CHANNEL{0xAA};
constexpr uint8_t CMD_SET_CHANNEL{0xAB};
constexpr uint8_t CMD_SET_HOPPING{0xAD};
constexpr uint8_t CMD_SET_TX_POWER{0xB6};
constexpr uint8_t CMD_GET_TX_POWER{0xB7};
constexpr uint8_t CMD_SLEEP{0x17};
constexpr uint8_t CMD_SET_AUTO_SLEEP_TIME{0x1D};

// Longest inactivity period the module accepts before sleeping
constexpr uint8_t AUTO_SLEEP_MAX_MINUTES{30};


// Reserved byte of the multiple polling parameter
constexpr uint8_t MULTIPLE_POLLING_RESERVED{0x22};

// Render a byte buffer as hex for the diagnostic logs
std::string to_hex(const uint8_t* data, const size_t len)
{
    std::string s{};
    for (size_t i = 0; i < len; ++i) {
        s += m5::utility::formatString("%02X ", data[i]);
    }
    return s;
}

// Probe settings for begin. The module answers within about 10ms once it is ready, so a short
// per-attempt timeout fits many more probes into the startup window than the normal timeout
// would. The M5Stack reference implementation retries getVersion() indefinitely at 500ms
// intervals, which suggests it met the same slow start; a library cannot block forever, so the
// probe is bounded by a deadline instead.
constexpr uint32_t BEGIN_PROBE_TIMEOUT_MS{200};
constexpr uint32_t BEGIN_RETRY_INTERVAL_MS{50};

// Delay before the first command is sent. The unit is powered from the Grove 5V rail, which
// the board brings up while M5Unified initialises, so the module may still be booting
constexpr uint32_t STARTUP_DELAY_MS{500};

// Upper bound for the begin probe. A healthy module answers the first probe within tens of
// milliseconds, and the slowest legitimate start observed on hardware was about two seconds.
// Beyond that the module is in the state described in ARCHITECTURE below, which no amount of
// waiting resolves, so the probe gives up rather than stalling the caller.
constexpr uint32_t BEGIN_PROBE_WINDOW_MS{3000};

constexpr int MODULE_INFORMATION_RETRY{3};
constexpr uint32_t MODULE_INFORMATION_RETRY_INTERVAL_MS{50};

uint8_t to_region_code(const m5::uhf::Region region)
{
    switch (region) {
        case m5::uhf::Region::China900MHz:
            return 0x01;
        case m5::uhf::Region::America:
            return 0x02;
        case m5::uhf::Region::Europe:
            return 0x03;
        case m5::uhf::Region::China800MHz:
            return 0x04;
        case m5::uhf::Region::SouthKorea:
            return 0x06;
        default:
            return 0x00;
    }
}

m5::uhf::Region from_region_code(const uint8_t code)
{
    switch (code) {
        case 0x01:
            return m5::uhf::Region::China900MHz;
        case 0x02:
            return m5::uhf::Region::America;
        case 0x03:
            return m5::uhf::Region::Europe;
        case 0x04:
            return m5::uhf::Region::China800MHz;
        case 0x06:
            return m5::uhf::Region::SouthKorea;
        default:
            return m5::uhf::Region::Unspecified;
    }
}
}  // namespace

namespace m5 {
namespace unit {

const char UnitJRD4035::name[] = "UnitJRD4035";
const types::uid_t UnitJRD4035::uid{"UnitJRD4035"_mmh3};
const types::attr_t UnitJRD4035::attr{attribute::AccessUART};

bool UnitJRD4035::begin()
{
    auto ad = asAdapter<AdapterUART>(Adapter::Type::UART);
    if (!ad) {
        M5_LIB_LOGE("Illegal adapter");
        return false;
    }
    if (!UHFRFIDComponent::begin()) {
        return false;
    }
    ad->setTimeout(_cfg.timeout_ms);
    ad->flushRX();

    // Hold back the first transmission until the module has finished booting (STARTUP_DELAY_MS).
    m5::utility::delay(STARTUP_DELAY_MS);
    ad->flushRX();

    // The UART transport has no address probe, so the module information is used to
    // confirm that the module responds
    bool detected{};
    const uint8_t kind[]{0x00};
    auto probe_once = [this, &kind]() {
        Frame res{};
        return send_and_wait(res, CMD_MODULE_INFORMATION, kind, sizeof(kind), BEGIN_PROBE_TIMEOUT_MS) &&
               res.parameter.size() >= 2;
    };

    // Probe until the module answers or the window closes
    const unsigned long probe_started_at = m5::utility::millis();
    const unsigned long probe_expire_at  = probe_started_at + BEGIN_PROBE_WINDOW_MS;
    int attempts{};
    while (m5::utility::millis() < probe_expire_at) {
        ++attempts;
        if (probe_once()) {
            detected = true;
            break;
        }
        M5_LIB_LOGD("Probe %d did not answer", attempts);
        m5::utility::delay(BEGIN_RETRY_INTERVAL_MS);
        ad->flushRX();
    }
    const unsigned long probe_elapsed = m5::utility::millis() - probe_started_at;
    if (!detected) {
        // The module can stop answering altogether until its power is removed, and nothing that
        // can be sent over the UART brings it back, so say what actually helps
        M5_LIB_LOGE(
            "UnitJRD4035 did not answer in %lums (%d probes). The module stops responding until "
            "its power is removed: disconnect the unit for a few seconds and reconnect it.",
            probe_elapsed, attempts);
        return false;
    }
    M5_LIB_LOGD("Module answered after %lums (%d probes)", probe_elapsed, attempts);

    m5::uhf::ModuleInformation info{};
    if (!readModuleInformation(info)) {
        M5_LIB_LOGE("Failed to readModuleInformation");
        return false;
    }
    M5_LIB_LOGI("HW:%s SW:%s MFR:%s", info.hardware_version.c_str(), info.software_version.c_str(),
                info.manufacturer.c_str());

    // The region is only written when the caller specified one, so the module's
    // factory setting is preserved by default
    if (_cfg.region != m5::uhf::Region::Unspecified && !writeRegion(_cfg.region)) {
        M5_LIB_LOGE("Failed to writeRegion");
        return false;
    }

    if (_cfg.start_polling && !startPolling(_cfg.polling_count)) {
        M5_LIB_LOGE("Failed to startPolling");
        return false;
    }
    return true;
}

bool UnitJRD4035::read_frame(Frame& out, const uint32_t timeout_ms)
{
    auto ad = asAdapter<AdapterUART>(Adapter::Type::UART);
    if (!ad) {
        return false;
    }

    // 1. Look for the frame header one byte at a time. A timeout here only means
    // that nothing is pending, and no byte is lost
    uint8_t b{};
    ad->setTimeout(timeout_ms);
    if (readWithTransaction(&b, 1) != m5::hal::error::error_t::OK) {
        return false;
    }
    M5_LIB_LOGV("RX: %02X", b);
    while (b != _frame_header) {
        M5_LIB_LOGD("Skipping %02X while looking for header %02X", b, _frame_header);
        if (readWithTransaction(&b, 1) != m5::hal::error::error_t::OK) {
            return false;
        }
        M5_LIB_LOGV("RX: %02X", b);
    }

    // 2. The remaining header bytes belong to the same frame, so a normal timeout is used
    std::vector<uint8_t> raw{};
    raw.push_back(b);
    uint8_t head[4]{};
    ad->setTimeout(_cfg.timeout_ms);
    if (readWithTransaction(head, sizeof(head)) != m5::hal::error::error_t::OK) {
        M5_LIB_LOGW("Header found but the rest did not arrive");
        return false;
    }
    raw.insert(raw.end(), head, head + sizeof(head));

    // 3. Parameter, checksum and end
    const uint16_t param_len = static_cast<uint16_t>((head[2] << 8) | head[3]);
    if (param_len > MAX_PARAMETER_LENGTH) {
        M5_LIB_LOGW("Too large parameter length %u", param_len);
        return false;
    }
    const size_t rest = static_cast<size_t>(param_len) + 2;
    std::vector<uint8_t> tail(rest);
    if (readWithTransaction(tail.data(), rest) != m5::hal::error::error_t::OK) {
        return false;
    }
    raw.insert(raw.end(), tail.begin(), tail.end());

    // 4. Validate
    if (!parse_frame(out, raw.data(), raw.size(), _frame_header, _frame_end)) {
        M5_LIB_LOGW("Malformed frame: %s", to_hex(raw.data(), raw.size()).c_str());
        return false;
    }
    return true;
}

void UnitJRD4035::route_frame(const Frame& f)
{
    note_frame_arrival();

    // Tag notification. The protocol document is inconsistent about the Type byte of a tag
    // notification, so the command code is used to route it
    if (f.command == COMMAND_SINGLE_POLLING || f.command == COMMAND_MULTIPLE_POLLING) {
        // The document is inconsistent about the Type byte of a tag notification; log the
        // observed value so it can be settled against real hardware
        M5_LIB_LOGD("Tag notification: type=%02X cmd=%02X", f.type, f.command);
        m5::uhf::Tag tag{};
        if (parse_tag_notification(tag, f.parameter.data(), f.parameter.size())) {
            // The module already rejects tags failing the CRC check (Inventory Fail 0x15), so a
            // mismatch here points at our own parsing rather than at RF corruption. Warn and keep
            // the tag so the problem is visible instead of silently losing detections
            if (!verify_tag_crc(tag)) {
                M5_LIB_LOGW("Gen2 CRC-16 mismatch (reported %04X)", tag.crc);
            }
            push_tag(tag);
        }
        return;
    }

    // Failure notification
    if (is_error_frame(f.command)) {
        const uint8_t code = f.parameter.empty() ? 0x00 : f.parameter[0];
        // During polling this only means that no tag responded this round
        if (is_no_tag(code)) {
            return;
        }
        M5_LIB_LOGW("Error frame %02X", code);
        if (_response_pending) {
            _response         = f;
            _response_pending = false;
        }
        return;
    }

    // Response to a pending command
    if (_response_pending && f.command == _awaiting_command) {
        _response         = f;
        _response_pending = false;
    }
}

bool UnitJRD4035::pump(const uint32_t timeout_ms)
{
    Frame f{};
    bool handled{};
    // Drain everything that is already pending
    while (read_frame(f, timeout_ms)) {
        route_frame(f);
        handled = true;
    }
    return handled;
}

bool UnitJRD4035::send_command(const uint8_t command, const uint8_t* param, const uint16_t param_len)
{
    std::vector<uint8_t> frame{};
    if (!build_frame(frame, TYPE_COMMAND, command, param, param_len, _frame_header, _frame_end)) {
        M5_LIB_LOGE("Failed to build frame");
        return false;
    }
    M5_LIB_LOGD("TX: %s", to_hex(frame.data(), frame.size()).c_str());
    return writeWithTransaction(frame.data(), frame.size()) == m5::hal::error::error_t::OK;
}

bool UnitJRD4035::send_and_wait(Frame& response, const uint8_t command, const uint8_t* param, const uint16_t param_len,
                                const uint32_t timeout_ms)
{
    // The response to a command that timed out can still arrive afterwards, and would then be
    // taken for the answer to this one, shifting every later exchange by one frame. Whatever is
    // already in flight is therefore consumed first: route_frame queues tag notifications as
    // usual and drops the stale responses, because nothing is pending yet.
    pump(1);

    _awaiting_command = command;
    _response         = Frame{};
    _response_pending = true;

    if (!send_command(command, param, param_len)) {
        _response_pending = false;
        return false;
    }

    // Keep pumping so that tag notifications arriving while we wait are queued, not dropped
    const unsigned long expire_at = m5::utility::millis() + timeout_ms;
    while (m5::utility::millis() < expire_at) {
        Frame f{};
        if (read_frame(f, 1)) {
            route_frame(f);
        }
        if (!_response_pending) {
            response = _response;
            return true;
        }
    }
    _response_pending = false;
    M5_LIB_LOGE("Timeout waiting for response of %02X", command);
    return false;
}

bool UnitJRD4035::start_polling_command(const uint16_t count)
{
    const uint8_t param[] = {MULTIPLE_POLLING_RESERVED, static_cast<uint8_t>(count >> 8),
                             static_cast<uint8_t>(count & 0xFF)};
    // The multiple polling command answers with notifications, not with a response frame
    return send_command(CMD_MULTIPLE_POLLING, param, sizeof(param));
}

bool UnitJRD4035::stop_polling_command()
{
    Frame res{};
    return send_and_wait(res, CMD_STOP_POLLING, nullptr, 0);
}

bool UnitJRD4035::read_module_information_kind(std::string& out, const uint8_t kind)
{
    Frame res{};
    const uint8_t param[] = {kind};
    if (!send_and_wait(res, CMD_MODULE_INFORMATION, param, sizeof(param))) {
        return false;
    }
    if (res.parameter.size() < 2) {
        return false;
    }
    // The first byte repeats the requested kind; the rest is an ASCII string
    out.assign(res.parameter.begin() + 1, res.parameter.end());
    return true;
}

bool UnitJRD4035::readModuleInformation(m5::uhf::ModuleInformation& info)
{
    if (reject_while_polling("readModuleInformation")) {
        return false;
    }
    for (int retry = 0; retry < MODULE_INFORMATION_RETRY; ++retry) {
        if (read_module_information_kind(info.hardware_version, 0x00) &&
            read_module_information_kind(info.software_version, 0x01) &&
            read_module_information_kind(info.manufacturer, 0x02)) {
            return true;
        }
        M5_LIB_LOGW("Retry readModuleInformation (%d)", retry);
        m5::utility::delay(MODULE_INFORMATION_RETRY_INTERVAL_MS);
    }
    return false;
}

bool UnitJRD4035::readTransmitPower(int16_t& dbm100)
{
    if (reject_while_polling("readTransmitPower")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_TX_POWER, nullptr, 0) || res.parameter.size() < 2) {
        return false;
    }
    dbm100 = static_cast<int16_t>((res.parameter[0] << 8) | res.parameter[1]);
    return true;
}

bool UnitJRD4035::writeTransmitPower(const int16_t dbm100)
{
    if (reject_while_polling("writeTransmitPower")) {
        return false;
    }
    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(dbm100 >> 8), static_cast<uint8_t>(dbm100 & 0xFF)};
    return send_and_wait(res, CMD_SET_TX_POWER, param, sizeof(param));
}

bool UnitJRD4035::readRegion(m5::uhf::Region& region)
{
    if (reject_while_polling("readRegion")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_REGION, nullptr, 0) || res.parameter.empty()) {
        return false;
    }
    region = from_region_code(res.parameter[0]);
    return true;
}

bool UnitJRD4035::writeRegion(const m5::uhf::Region region)
{
    if (reject_while_polling("writeRegion")) {
        return false;
    }
    const uint8_t code = to_region_code(region);
    if (code == 0x00) {
        M5_LIB_LOGE("Unsupported region");
        return false;
    }
    Frame res{};
    const uint8_t param[] = {code};
    return send_and_wait(res, CMD_SET_REGION, param, sizeof(param));
}

bool UnitJRD4035::readChannel(uint8_t& index)
{
    if (reject_while_polling("readChannel")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_CHANNEL, nullptr, 0) || res.parameter.empty()) {
        return false;
    }
    index = res.parameter[0];
    return true;
}

bool UnitJRD4035::writeChannel(const uint8_t index)
{
    if (reject_while_polling("writeChannel")) {
        return false;
    }
    Frame res{};
    const uint8_t param[] = {index};
    return send_and_wait(res, CMD_SET_CHANNEL, param, sizeof(param));
}

bool UnitJRD4035::writeAutomaticFrequencyHopping(const bool enable)
{
    if (reject_while_polling("writeAutomaticFrequencyHopping")) {
        return false;
    }
    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(enable ? 0xFF : 0x00)};
    return send_and_wait(res, CMD_SET_HOPPING, param, sizeof(param));
}

bool UnitJRD4035::readQueryParameters(m5::uhf::QueryParameters& qp)
{
    if (reject_while_polling("readQueryParameters")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_QUERY, nullptr, 0) || res.parameter.size() < 2) {
        return false;
    }
    // Query: DR(1) M(2) TRext(1) Sel(2) Session(2) Target(1) Q(4) as a 13-bit field
    const uint16_t raw = static_cast<uint16_t>((res.parameter[0] << 8) | res.parameter[1]);
    qp.q               = static_cast<uint8_t>((raw >> 0) & 0x0F);
    qp.target          = static_cast<m5::uhf::Target>((raw >> 4) & 0x01);
    qp.session         = static_cast<m5::uhf::Session>((raw >> 5) & 0x03);
    return true;
}

bool UnitJRD4035::writeQueryParameters(const m5::uhf::QueryParameters& qp)
{
    if (reject_while_polling("writeQueryParameters")) {
        return false;
    }
    // Preserve the fields we do not expose by reading the current value first
    Frame cur{};
    if (!send_and_wait(cur, CMD_GET_QUERY, nullptr, 0) || cur.parameter.size() < 2) {
        return false;
    }
    uint16_t raw = static_cast<uint16_t>((cur.parameter[0] << 8) | cur.parameter[1]);
    raw &= static_cast<uint16_t>(~0x00FF);
    raw |= static_cast<uint16_t>(qp.q & 0x0F);
    raw |= static_cast<uint16_t>((static_cast<uint8_t>(qp.target) & 0x01) << 4);
    raw |= static_cast<uint16_t>((static_cast<uint8_t>(qp.session) & 0x03) << 5);

    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(raw >> 8), static_cast<uint8_t>(raw & 0xFF)};
    return send_and_wait(res, CMD_SET_QUERY, param, sizeof(param));
}

bool UnitJRD4035::writeAutoSleepTime(const uint8_t minutes)
{
    if (reject_while_polling("writeAutoSleepTime")) {
        return false;
    }
    if (minutes > AUTO_SLEEP_MAX_MINUTES) {
        M5_LIB_LOGE("Auto sleep time out of range %u", minutes);
        return false;
    }
    Frame res{};
    const uint8_t param[]{minutes};
    return send_and_wait(res, CMD_SET_AUTO_SLEEP_TIME, param, sizeof(param));
}

bool UnitJRD4035::sleep()
{
    if (reject_while_polling("sleep")) {
        return false;
    }
    // The module answers before it powers down, so the response is awaited as usual
    Frame res{};
    return send_and_wait(res, CMD_SLEEP, nullptr, 0);
}

}  // namespace unit
}  // namespace m5
