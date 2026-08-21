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

#include <M5Utility.hpp>

using namespace m5::unit::jrd4035;
using namespace m5::unit::types;
using namespace m5::utility::mmh3;

namespace {
// Command codes
constexpr uint8_t CMD_MODULE_INFORMATION = 0x03;
constexpr uint8_t CMD_MULTIPLE_POLLING   = 0x27;
constexpr uint8_t CMD_STOP_POLLING       = 0x28;
constexpr uint8_t CMD_GET_QUERY          = 0x0D;
constexpr uint8_t CMD_SET_QUERY          = 0x0E;
constexpr uint8_t CMD_SET_REGION         = 0x07;
constexpr uint8_t CMD_GET_REGION         = 0x08;
constexpr uint8_t CMD_GET_CHANNEL        = 0xAA;
constexpr uint8_t CMD_SET_CHANNEL        = 0xAB;
constexpr uint8_t CMD_SET_HOPPING        = 0xAD;
constexpr uint8_t CMD_SET_TX_POWER       = 0xB6;
constexpr uint8_t CMD_GET_TX_POWER       = 0xB7;

// Reserved byte of the multiple polling parameter
constexpr uint8_t MULTIPLE_POLLING_RESERVED = 0x22;

// Number of begin retries (NessoN1 first-transaction failure)
constexpr int BEGIN_RETRY = 3;

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

    // The UART transport has no address probe, so the module information is used to
    // confirm that the module responds
    m5::uhf::ModuleInformation info{};
    bool detected{};
    for (int retry = 0; retry < BEGIN_RETRY; ++retry) {
        if (readModuleInformation(info) && !info.hardware_version.empty()) {
            detected = true;
            break;
        }
        M5_LIB_LOGW("Retry readModuleInformation (%d)", retry);
        m5::utility::delay(100);
        ad->flushRX();
    }
    if (!detected) {
        M5_LIB_LOGE("UnitJRD4035 was not detected");
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
    while (b != _frame_header) {
        if (readWithTransaction(&b, 1) != m5::hal::error::error_t::OK) {
            return false;
        }
    }

    // 2. The remaining header bytes belong to the same frame, so a normal timeout is used
    std::vector<uint8_t> raw{};
    raw.push_back(b);
    uint8_t head[4]{};
    ad->setTimeout(_cfg.timeout_ms);
    if (readWithTransaction(head, sizeof(head)) != m5::hal::error::error_t::OK) {
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
        M5_LIB_LOGW("Malformed frame");
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
        m5::uhf::Tag tag{};
        if (parse_tag_notification(tag, f.parameter.data(), f.parameter.size())) {
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
    return writeWithTransaction(frame.data(), frame.size()) == m5::hal::error::error_t::OK;
}

bool UnitJRD4035::send_and_wait(Frame& response, const uint8_t command, const uint8_t* param, const uint16_t param_len,
                                const uint32_t timeout_ms)
{
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
    return read_module_information_kind(info.hardware_version, 0x00) &&
           read_module_information_kind(info.software_version, 0x01) &&
           read_module_information_kind(info.manufacturer, 0x02);
}

bool UnitJRD4035::readTransmitPower(int16_t& dbm100)
{
    Frame res{};
    if (!send_and_wait(res, CMD_GET_TX_POWER, nullptr, 0) || res.parameter.size() < 2) {
        return false;
    }
    dbm100 = static_cast<int16_t>((res.parameter[0] << 8) | res.parameter[1]);
    return true;
}

bool UnitJRD4035::writeTransmitPower(const int16_t dbm100)
{
    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(dbm100 >> 8), static_cast<uint8_t>(dbm100 & 0xFF)};
    return send_and_wait(res, CMD_SET_TX_POWER, param, sizeof(param));
}

bool UnitJRD4035::readRegion(m5::uhf::Region& region)
{
    Frame res{};
    if (!send_and_wait(res, CMD_GET_REGION, nullptr, 0) || res.parameter.empty()) {
        return false;
    }
    region = from_region_code(res.parameter[0]);
    return true;
}

bool UnitJRD4035::writeRegion(const m5::uhf::Region region)
{
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
    Frame res{};
    if (!send_and_wait(res, CMD_GET_CHANNEL, nullptr, 0) || res.parameter.empty()) {
        return false;
    }
    index = res.parameter[0];
    return true;
}

bool UnitJRD4035::writeChannel(const uint8_t index)
{
    Frame res{};
    const uint8_t param[] = {index};
    return send_and_wait(res, CMD_SET_CHANNEL, param, sizeof(param));
}

bool UnitJRD4035::writeAutomaticFrequencyHopping(const bool enable)
{
    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(enable ? 0xFF : 0x00)};
    return send_and_wait(res, CMD_SET_HOPPING, param, sizeof(param));
}

bool UnitJRD4035::readQueryParameters(m5::uhf::QueryParameters& qp)
{
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

}  // namespace unit
}  // namespace m5
