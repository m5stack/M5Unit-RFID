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

using namespace m5::unit::m100;
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
constexpr uint8_t CMD_IDLE_MODE{0x04};
constexpr uint8_t CMD_SLEEP{0x17};
constexpr uint8_t CMD_SET_AUTO_SLEEP_TIME{0x1D};
constexpr uint8_t CMD_INSERT_CHANNEL{0xA9};
constexpr uint8_t CMD_SCAN_JAMMER{0xF2};
constexpr uint8_t CMD_SCAN_RSSI{0xF3};
constexpr uint8_t CMD_GET_DEMODULATOR{0xF1};
constexpr uint8_t CMD_SET_DEMODULATOR{0xF0};
constexpr uint8_t CMD_GET_SELECT_PARAMETER{0x0B};
constexpr uint8_t CMD_SET_SELECT_PARAMETER{0x0C};
constexpr uint8_t CMD_SET_SELECT_MODE{0x12};
constexpr uint8_t CMD_READ_TAG_MEMORY{COMMAND_READ_TAG_MEMORY};
constexpr uint8_t CMD_WRITE_TAG_MEMORY{COMMAND_WRITE_TAG_MEMORY};
constexpr uint8_t CMD_LOCK_TAG_MEMORY{COMMAND_LOCK_TAG_MEMORY};
constexpr uint8_t CMD_KILL_TAG{COMMAND_KILL_TAG};

// The channel count of the insert command is a single byte
constexpr size_t CHANNEL_LIST_MAX{255};

// A scan sweeps every channel of the region and answers only once it is done, so it needs far
// longer than an ordinary setting command
constexpr uint32_t CHANNEL_SCAN_TIMEOUT_MS{3000};

// Longest inactivity period the module accepts before sleeping
constexpr uint8_t AUTO_SLEEP_MAX_MINUTES{30};
//! @brief Longest the module can be left to enter IDLE on its own
constexpr uint8_t AUTO_IDLE_MAX_MINUTES{30};
//! @brief Second parameter byte of the IDLE command. The protocol fixes it at one
constexpr uint8_t IDLE_RESERVED{0x01};

// Reserved byte of the multiple polling parameter
constexpr uint8_t MULTIPLE_POLLING_RESERVED{0x22};

/*!
  @name Select actions used to address a single tag
  @details A Select sets the inventoried flag of one session, and the round that follows invites
  one end of that flag. Setting the matching tag to the end the round invites, and everything
  else to the other, is what narrows the field down to one tag
  @note The Target field says which session, and it has to be the session the round names: a
  Select acting on any other changes a flag the round does not look at (EPC Gen2 v2.1 6.3.2.2).
  Both are taken from the Query the module currently holds rather than assumed
 */
///@{
//! Action 000: matching tags to A, the rest to B (EPC Gen2 v2.1 Table 6-31)
constexpr uint8_t SELECT_ACTION_MATCH_TO_A{0x00};
//! Action 100: matching tags to B, the rest to A (EPC Gen2 v2.1 Table 6-31)
constexpr uint8_t SELECT_ACTION_MATCH_TO_B{0x04};
///@}

/*!
  @brief Settling time between storing the select mask and using it
  @details The response to Set Select Parameter arriving does not mean the mask is in effect
  yet. The vendor's own host tool waits this long before it reads
 */
constexpr uint32_t SELECT_SETTLE_MS{20};

/*!
  @brief How long the module is given to answer a tag operation
  @details A write costs about 37ms of overhead plus 10ms for every word, measured on an Alien
  Higgs 9 and an Impinj Monza 4QT: a two word write comes back in 51 to 72ms and the longest a
  single command can be, 32 words, in 307 to 394ms. Reads are an order of magnitude cheaper per
  word, about 1.2ms, because Gen2 returns the whole of a read in one backscatter while a write
  programs the words one at a time and pays the tag's write time for each. Five times the worst
  measured leaves room for a chip slower to program than either of those. A module that has
  stopped answering altogether is the only thing this ever actually waits out
 */
constexpr uint32_t TAG_OPERATION_TIMEOUT_MS{2000};

/*!
  @brief How many times a tag operation is attempted before it is reported as failed
  @details A tag fails to complete about one exchange in fifty with the antenna and the tag
  well placed, and about one in five with them not, whichever chip it is. Polling hides that by
  running rounds continuously; a single addressed operation cannot. Three attempts put the
  residual under a percent even in the poor case
  @note No delay between attempts. Learning that an operation failed already takes 33 to 160ms,
  which is longer than the 25ms an inventory round takes, so the retry lands in a fresh round
  without being made to wait for one
 */
constexpr int TAG_OPERATION_ATTEMPTS{3};

/*!
  @brief Longest gap between two bytes of the same frame
  @details The module sends a frame in one go, so a pause longer than a couple of dozen byte
  times means it has finished sending rather than that more is coming
 */
constexpr uint32_t BYTE_GAP_TIMEOUT_MS{2};

/*!
  @brief How long the link has to stay silent before the next command is sent
  @details The answer to a command that timed out can still arrive afterwards, and a response
  frame carries nothing but the command code to say what it answers. A gap this long says the
  answer is not on its way, which is what makes the next command's answer its own
 */
constexpr uint32_t RESYNC_QUIET_MS{50};

/*!
  @brief Longest a resynchronisation may take
  @details Tag notifications keep arriving while a round is running, so the link does not fall
  silent on its own during one. This bounds the wait for that case
 */
constexpr uint32_t RESYNC_LIMIT_MS{250};

//! @brief Byte sent to wake the module. Any value does; the protocol does not name one
constexpr uint8_t WAKE_BYTE{0x55};
//! @brief How long to wait after the wake byte. Known to be short: a module can take over a
//! second to answer, and no figure for this appears in the vendor's documents
constexpr uint32_t WAKE_DELAY_MS{100};

//! @brief A bank code that is not one of the four, used to reject a bank cast in from outside
constexpr uint8_t MEMBANK_NONE{0xFF};

/*!
  @brief Translate a bank to the code the module uses
  @param bank Bank
  @return Bank code, or MEMBANK_NONE when the bank is not one of the four
  @details Naming a bank the enum does not hold can only be a mistake, and every bank is a
  valid target for the operations that take one. Guessing which was meant would silently read
  or write the wrong part of the tag, so this says so instead
 */
inline uint8_t membank_of(const m5::uhf::Bank bank)
{
    switch (bank) {
        case m5::uhf::Bank::Reserved:
            return MEMBANK_RESERVED;
        case m5::uhf::Bank::Epc:
            return MEMBANK_EPC;
        case m5::uhf::Bank::Tid:
            return MEMBANK_TID;
        case m5::uhf::Bank::User:
            return MEMBANK_USER;
    }
    M5_LIB_LOGE("Illegal bank %u", (unsigned)static_cast<uint8_t>(bank));
    return MEMBANK_NONE;
}

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
// would.
constexpr uint32_t BEGIN_PROBE_TIMEOUT_MS{200};
constexpr uint32_t BEGIN_RETRY_INTERVAL_MS{50};

// Delay before the first command is sent. The unit is powered from the Grove 5V rail, which
// the board brings up while M5Unified initialises, so the module may still be booting
constexpr uint32_t STARTUP_DELAY_MS{500};

// Upper bound for the begin probe. Measured on hardware, the module always answered the first
// probe within 6 to 64 milliseconds, so this window is generous. A module that never answers is
// almost always one that is not wired up, and stalling the caller does not help with that.
constexpr uint32_t BEGIN_PROBE_WINDOW_MS{3000};

constexpr int MODULE_INFORMATION_RETRY{3};
constexpr uint32_t MODULE_INFORMATION_RETRY_INTERVAL_MS{50};

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

//! @brief The code the module uses for a region, or 0x00 when it is not one of them
uint8_t to_region_code(const m5::uhf::Region region)
{
    // The enumerators carry the module's own codes, so the round trip through the table above
    // is what says whether the caller named a region the module knows
    const uint8_t code = static_cast<uint8_t>(region);
    return from_region_code(code) == region ? code : 0x00;
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
    flush_rx();

    // Hold back the first transmission until the module has finished booting
    m5::utility::delay(STARTUP_DELAY_MS);
    flush_rx();

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
        flush_rx();
    }
    const unsigned long probe_elapsed = m5::utility::millis() - probe_started_at;
    if (!detected) {
        M5_LIB_LOGE(
            "UnitJRD4035 did not answer in %lums (%d probes). Check the cable and the connectors, "
            "and that the unit is powered.",
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

    return true;
}

void UnitJRD4035::flush_rx()
{
    auto ad = asAdapter<AdapterUART>(Adapter::Type::UART);
    if (ad) {
        ad->flushRX();
    }
    // Half a frame left over here would be taken for the start of the next one
    _rx.clear();
}

bool UnitJRD4035::read_frame(Frame& out, const uint32_t timeout_ms)
{
    auto ad = asAdapter<AdapterUART>(Adapter::Type::UART);
    if (!ad) {
        return false;
    }

    bool first = true;
    for (;;) {
        // What is already in hand is tried first: the bytes of a frame that arrived in pieces
        // are still there from the last call, and finishing it needs no waiting at all
        size_t discarded{};
        const FrameExtract taken = extract_frame(out, _rx, discarded, _frame_header, _frame_end);
        if (discarded) {
            M5_LIB_LOGW("Dropped %zu bytes to find the start of a frame", discarded);
        }
        if (taken == FrameExtract::Ok) {
            M5_LIB_LOGD("RX: type=%02X cmd=%02X len=%u", out.type, out.command,
                        static_cast<unsigned>(out.parameter.size()));
            return true;
        }

        // Waiting on the first byte is waiting for a frame to arrive at all, which is what the
        // caller's timeout is about. Every byte after it belongs to a frame already on its way
        uint8_t b{};
        ad->setTimeout(first ? timeout_ms : BYTE_GAP_TIMEOUT_MS);
        first = false;
        if (readWithTransaction(&b, 1) != m5::hal::error::error_t::OK) {
            // Whatever is in the buffer stays there. Nothing is lost by giving up here
            return false;
        }
        _rx.push_back(b);
    }
}

void UnitJRD4035::route_frame(const Frame& f)
{
    note_frame_arrival();

    switch (route_for(f, _response_pending, _awaiting_command)) {
        case FrameRoute::TagNotification: {
            M5_LIB_LOGD("Tag notification: type=%02X cmd=%02X", f.type, f.command);
            m5::uhf::Tag tag{};
            if (parse_tag_notification(tag, f.parameter.data(), f.parameter.size())) {
                // The module already rejects tags failing the CRC check (Inventory Fail 0x15), so a
                // mismatch here points at our own parsing rather than at RF corruption. Warn and keep
                // the tag so the problem is visible instead of silently losing detections
                if (!m5::uhf::verify_tag_crc(tag)) {
                    M5_LIB_LOGW("Gen2 CRC-16 mismatch (reported %04X)", tag.crc);
                }
                push_tag(tag);
            }
            break;
        }
        case FrameRoute::Response:
            // Whether a failure here matters is not known yet: a retry may still succeed. Saying
            // so is left to succeeded(), which is reached only once the operation has given up
            _response         = f;
            _response_pending = false;
            break;
        case FrameRoute::Drop:
            // An empty round reports itself once per round, which is ordinary and not worth a line
            if (f.parameter.empty() || !is_no_tag(f.parameter[0])) {
                M5_LIB_LOGD("Left over from an earlier exchange: cmd=%02X code=%02X", f.command,
                            f.parameter.empty() ? 0x00 : f.parameter[0]);
            }
            break;
        case FrameRoute::Unexpected:
            M5_LIB_LOGW("Unhandled frame: type=%02X cmd=%02X len=%u%s", f.type, f.command,
                        static_cast<unsigned>(f.parameter.size()),
                        _response_pending ? ", while waiting for a response" : "");
            break;
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

void UnitJRD4035::resynchronize()
{
    // Nothing is pending, so route_frame queues tag notifications as usual and drops answers.
    // Reading on until the link falls silent is what keeps a late answer out of the next exchange
    const unsigned long give_up_at = m5::utility::millis() + RESYNC_LIMIT_MS;
    Frame f{};
    while (m5::utility::millis() < give_up_at) {
        if (!read_frame(f, RESYNC_QUIET_MS)) {
            return;
        }
        route_frame(f);
    }
    M5_LIB_LOGW("The link did not fall silent; the next answer may belong to an earlier command");
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
    // Anything that arrived since the last exchange is taken first: route_frame queues tag
    // notifications as usual and drops answers, because nothing is pending yet. That clears what
    // has already come in; an answer still on its way is waited out where the timeout happens
    pump(1);

    _awaiting_command = command;
    _response         = Frame{};
    _response_pending = true;

    if (!send_command(command, param, param_len)) {
        _response_pending = false;
        return false;
    }

    // Keep pumping so that tag notifications arriving while we wait are queued, not dropped
    const uint32_t wait_ms        = timeout_ms != 0 ? timeout_ms : _cfg.command_timeout_ms;
    const unsigned long expire_at = m5::utility::millis() + wait_ms;
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
    M5_LIB_LOGE("Timeout waiting for the response to %02X", command);
    // The answer may still be on its way, and nothing but the command code says what a response
    // answers. Leaving it there would make it the answer to whatever is sent next
    resynchronize();
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
    return send_and_wait(res, CMD_STOP_POLLING, nullptr, 0) && succeeded(res, "stopPolling");
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
    if (!send_and_wait(res, CMD_GET_TX_POWER, nullptr, 0) || !succeeded(res, "readTransmitPower") ||
        res.parameter.size() < 2) {
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
    return send_and_wait(res, CMD_SET_TX_POWER, param, sizeof(param)) && succeeded(res, "writeTransmitPower");
}

bool UnitJRD4035::readRegion(m5::uhf::Region& region)
{
    if (reject_while_polling("readRegion")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_REGION, nullptr, 0) || !succeeded(res, "readRegion") || res.parameter.empty()) {
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
    return send_and_wait(res, CMD_SET_REGION, param, sizeof(param)) && succeeded(res, "writeRegion");
}

bool UnitJRD4035::readChannel(uint8_t& index)
{
    if (reject_while_polling("readChannel")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_CHANNEL, nullptr, 0) || !succeeded(res, "readChannel") || res.parameter.empty()) {
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
    return send_and_wait(res, CMD_SET_CHANNEL, param, sizeof(param)) && succeeded(res, "writeChannel");
}

bool UnitJRD4035::writeAutomaticFrequencyHopping(const bool enable)
{
    if (reject_while_polling("writeAutomaticFrequencyHopping")) {
        return false;
    }
    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(enable ? 0xFF : 0x00)};
    return send_and_wait(res, CMD_SET_HOPPING, param, sizeof(param)) &&
           succeeded(res, "writeAutomaticFrequencyHopping");
}

bool UnitJRD4035::readQueryParameters(m5::uhf::QueryParameters& qp)
{
    if (reject_while_polling("readQueryParameters")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_QUERY, nullptr, 0) || !succeeded(res, "readQueryParameters") ||
        res.parameter.size() < 2) {
        return false;
    }
    parse_query_parameters(qp, static_cast<uint16_t>((res.parameter[0] << 8) | res.parameter[1]));
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
    const uint16_t raw = build_query_parameters(qp, static_cast<uint16_t>((cur.parameter[0] << 8) | cur.parameter[1]));

    Frame res{};
    const uint8_t param[] = {static_cast<uint8_t>(raw >> 8), static_cast<uint8_t>(raw & 0xFF)};
    if (!send_and_wait(res, CMD_SET_QUERY, param, sizeof(param)) || !succeeded(res, "writeQueryParameters")) {
        return false;
    }
    // A mask already stored was built to match the round these settings describe. Changing them
    // leaves it acting on a flag the round no longer looks at, which nothing reports
    M5_LIB_LOGW("A select mask stored before this was built for the old round; store it again");
    return true;
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
    return send_and_wait(res, CMD_SET_AUTO_SLEEP_TIME, param, sizeof(param)) && succeeded(res, "writeAutoSleepTime");
}

bool UnitJRD4035::writeIdle(const bool enter, const uint8_t minutes)
{
    if (reject_while_polling("writeIdle")) {
        return false;
    }
    if (minutes > AUTO_IDLE_MAX_MINUTES) {
        M5_LIB_LOGE("Auto idle time out of range %u", minutes);
        return false;
    }
    Frame res{};
    const uint8_t param[]{static_cast<uint8_t>(enter ? 0x01 : 0x00), IDLE_RESERVED, minutes};
    return send_and_wait(res, CMD_IDLE_MODE, param, sizeof(param)) && succeeded(res, "writeIdle");
}

bool UnitJRD4035::sleep()
{
    if (reject_while_polling("sleep")) {
        return false;
    }
    // The module answers before it powers down, so the response is awaited as usual
    Frame res{};
    return send_and_wait(res, CMD_SLEEP, nullptr, 0) && succeeded(res, "sleep");
}

bool UnitJRD4035::wake()
{
    auto ad = asAdapter<AdapterUART>(Adapter::Type::UART);
    if (!ad) {
        M5_LIB_LOGE("Illegal adapter");
        return false;
    }
    // The module discards whatever byte wakes it, so this one is spent on that alone
    const uint8_t byte{WAKE_BYTE};
    if (writeWithTransaction(&byte, 1) != m5::hal::error::error_t::OK) {
        M5_LIB_LOGE("Failed to send the wake byte");
        return false;
    }
    // Waking powers the chip down and reloads its firmware, so nothing answers until that is
    // done. How long that takes is not documented, and this is not a measured figure
    m5::utility::delay(WAKE_DELAY_MS);
    return true;
}

bool UnitJRD4035::writeOperatingChannels(const std::vector<uint8_t>& channels)
{
    if (reject_while_polling("writeOperatingChannels")) {
        return false;
    }
    if (channels.empty() || channels.size() > CHANNEL_LIST_MAX) {
        M5_LIB_LOGE("Illegal channel count %zu", channels.size());
        return false;
    }
    std::vector<uint8_t> param{};
    param.reserve(channels.size() + 1);
    param.push_back(static_cast<uint8_t>(channels.size()));
    param.insert(param.end(), channels.begin(), channels.end());

    Frame res{};
    return send_and_wait(res, CMD_INSERT_CHANNEL, param.data(), static_cast<uint16_t>(param.size())) &&
           succeeded(res, "writeOperatingChannels");
}

bool UnitJRD4035::read_channel_levels(m5::uhf::ChannelLevels& levels, const uint8_t command)
{
    Frame res{};
    // CH_L and CH_H bracket the scanned range, and one signed level follows for each channel
    if (!send_and_wait(res, command, nullptr, 0, CHANNEL_SCAN_TIMEOUT_MS) || !succeeded(res, "read_channel_levels") ||
        res.parameter.size() < 3) {
        return false;
    }
    const uint8_t first = res.parameter[0];
    const uint8_t last  = res.parameter[1];
    const size_t count  = res.parameter.size() - 2;
    if (last < first || static_cast<size_t>(last - first) + 1 != count) {
        M5_LIB_LOGE("Channel range %u-%u does not match %zu levels", first, last, count);
        return false;
    }

    levels.first_channel = first;
    levels.dbm.clear();
    levels.dbm.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        levels.dbm.push_back(static_cast<int8_t>(res.parameter[i + 2]));
    }
    return true;
}

bool UnitJRD4035::readBlockingSignal(m5::uhf::ChannelLevels& levels)
{
    if (reject_while_polling("readBlockingSignal")) {
        return false;
    }
    return read_channel_levels(levels, CMD_SCAN_JAMMER);
}

bool UnitJRD4035::readChannelRSSI(m5::uhf::ChannelLevels& levels)
{
    if (reject_while_polling("readChannelRSSI")) {
        return false;
    }
    return read_channel_levels(levels, CMD_SCAN_RSSI);
}

bool UnitJRD4035::succeeded(const Frame& response, const char* what) const
{
    if (!is_error_frame(response.command)) {
        return true;
    }
    const uint8_t code = response.parameter.empty() ? 0x00 : response.parameter[0];
    // The module appends the PC and EPC of the tag it was addressing when it had already got
    // that far, which is what says whether the tag was seen at all before the failure
    const std::string tail =
        response.parameter.size() > 1 ? to_hex(response.parameter.data() + 1, response.parameter.size() - 1) : "";
    M5_LIB_LOGE("%s failed: %02X %s %s", what, code, error_description(code), tail.c_str());
    return false;
}

bool UnitJRD4035::readSelectParameter(m5::uhf::SelectParameter& sp)
{
    if (reject_while_polling("readSelectParameter")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_SELECT_PARAMETER, nullptr, 0) || !succeeded(res, "readSelectParameter")) {
        return false;
    }
    if (!parse_select_parameter(sp, res.parameter.data(), res.parameter.size())) {
        M5_LIB_LOGE("Malformed select parameter");
        return false;
    }
    return true;
}

bool UnitJRD4035::readDemodulatorParameters(m100::DemodulatorParameters& dp)
{
    if (reject_while_polling("readDemodulatorParameters")) {
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_GET_DEMODULATOR, nullptr, 0) || !succeeded(res, "readDemodulatorParameters")) {
        return false;
    }
    if (!parse_demodulator_parameters(dp, res.parameter.data(), res.parameter.size())) {
        M5_LIB_LOGE("Malformed demodulator parameters");
        return false;
    }
    return true;
}

bool UnitJRD4035::writeDemodulatorParameters(const m100::DemodulatorParameters& dp)
{
    if (reject_while_polling("writeDemodulatorParameters")) {
        return false;
    }
    std::vector<uint8_t> param{};
    if (!build_demodulator_parameters(param, dp)) {
        M5_LIB_LOGE("Illegal demodulator parameters");
        return false;
    }
    Frame res{};
    return send_and_wait(res, CMD_SET_DEMODULATOR, param.data(), static_cast<uint16_t>(param.size())) &&
           succeeded(res, "writeDemodulatorParameters");
}

bool UnitJRD4035::send_tag_operation(Frame& response, const uint8_t command, const uint8_t* param,
                                     const uint16_t param_len, const char* what)
{
    for (int attempt = 1; attempt <= TAG_OPERATION_ATTEMPTS; ++attempt) {
        // A module saying nothing at all is not failing for a reason a repeat would fix, and
        // each attempt would cost the whole timeout, so that case is left alone
        if (!send_and_wait(response, command, param, param_len, TAG_OPERATION_TIMEOUT_MS)) {
            return false;
        }
        if (!is_error_frame(response.command)) {
            if (attempt > 1) {
                M5_LIB_LOGD("%s answered on attempt %d", what, attempt);
            }
            return true;
        }
        const uint8_t code = response.parameter.empty() ? 0x00 : response.parameter[0];
        if (!is_worth_retrying(code)) {
            break;
        }
        M5_LIB_LOGD("%s: %02X %s (attempt %d of %d)", what, code, error_description(code), attempt,
                    TAG_OPERATION_ATTEMPTS);
    }
    return succeeded(response, what);
}

bool UnitJRD4035::write_select_parameter(const m5::uhf::Bank bank, const uint32_t pointer_bits, const uint8_t* mask,
                                         const size_t mask_len)
{
    if (reject_while_polling("write_select_parameter")) {
        return false;
    }
    if (mask_len * 8 > SELECT_MASK_MAX_BITS) {
        M5_LIB_LOGE("Mask of %zu bytes is longer than a Select can carry", mask_len);
        return false;
    }
    const uint8_t code = membank_of(bank);
    if (code == MEMBANK_NONE) {
        return false;
    }
    // A Select acts on one session's inventoried flag, and a round is qualified by the flag of
    // the session it names. Act on a different one and the mask changes nothing a round looks
    // at (Gen2 v2.1 6.3.2.2); assert the wrong end of it and the mask picks out every tag
    // except the one meant. Both are silent, so the round's own settings decide these two
    m5::uhf::QueryParameters qp{};
    if (!readQueryParameters(qp)) {
        M5_LIB_LOGE("Cannot store a mask without knowing which round it has to match");
        return false;
    }
    const uint8_t target    = static_cast<uint8_t>(qp.session);
    const uint8_t action    = (qp.target == m5::uhf::Target::A) ? SELECT_ACTION_MATCH_TO_A : SELECT_ACTION_MATCH_TO_B;
    const uint8_t sel_param = select_parameter_byte(target, action, code);

    std::vector<uint8_t> param{};
    if (!build_select_parameter(param, sel_param, pointer_bits, static_cast<uint8_t>(mask_len * 8), SELECT_TRUNCATE_OFF,
                                mask, mask_len)) {
        M5_LIB_LOGE("Failed to build the select parameter");
        return false;
    }
    Frame res{};
    if (!send_and_wait(res, CMD_SET_SELECT_PARAMETER, param.data(), static_cast<uint16_t>(param.size()))) {
        return false;
    }
    if (!succeeded(res, "write_select_parameter")) {
        return false;
    }
    m5::utility::delay(SELECT_SETTLE_MS);
    return true;
}

bool UnitJRD4035::write_select_enabled(const bool enable)
{
    if (reject_while_polling("write_select_enabled")) {
        return false;
    }
    // Setting the parameter already switches the module to SELECT_MODE_NON_INVENTORY on its own,
    // so this is only needed to turn the mask back off, and to put it back on afterwards
    const uint8_t param[] = {enable ? SELECT_MODE_NON_INVENTORY : SELECT_MODE_NEVER};
    Frame res{};
    // The protocol document has Set Select Mode answering under the command code of Set Select
    // Parameter, but an M100 running V2.3.5 answers under 0x12, its own. Measured, not read
    if (!send_and_wait(res, CMD_SET_SELECT_MODE, param, sizeof(param))) {
        return false;
    }
    return succeeded(res, "write_select_enabled");
}

bool UnitJRD4035::read_tag_memory(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t word_address,
                                  const uint16_t word_count, const uint32_t access_password)
{
    out.clear();
    if (reject_while_polling("read_tag_memory")) {
        return false;
    }
    std::vector<uint8_t> param{};
    const uint8_t code = membank_of(bank);
    if (code == MEMBANK_NONE) {
        return false;
    }
    if (!build_read_tag_memory(param, access_password, code, word_address, word_count)) {
        M5_LIB_LOGE("Failed to build the read parameter");
        return false;
    }
    Frame res{};
    if (!send_tag_operation(res, CMD_READ_TAG_MEMORY, param.data(), static_cast<uint16_t>(param.size()),
                            "read_tag_memory")) {
        return false;
    }

    TagOperationResult result{};
    if (!parse_tag_operation(result, res.parameter.data(), res.parameter.size())) {
        M5_LIB_LOGE("Malformed read response");
        return false;
    }
    // A short answer would leave the caller reading fewer words than it asked for without ever
    // being told, so it is refused rather than truncated
    if (result.data_len != static_cast<size_t>(word_count) * 2) {
        M5_LIB_LOGE("Asked for %u words but got %zu bytes", word_count, result.data_len);
        return false;
    }
    out.assign(result.data, result.data + result.data_len);
    return true;
}

bool UnitJRD4035::write_tag_memory(const m5::uhf::Bank bank, const uint16_t word_address, const uint8_t* data,
                                   const size_t len, const uint32_t access_password)
{
    if (reject_while_polling("write_tag_memory")) {
        return false;
    }
    std::vector<uint8_t> param{};
    const uint8_t code = membank_of(bank);
    if (code == MEMBANK_NONE) {
        return false;
    }
    if (!build_write_tag_memory(param, access_password, code, word_address, data, len)) {
        M5_LIB_LOGE("Failed to build the write parameter");
        return false;
    }
    Frame res{};
    // The protocol document says in prose that Write answers under the command code of Read,
    // but the byte table on the same page reads BB 01 49 ... A9 7E, and that checksum only adds
    // up with 0x49 in the command byte; 0x39 would make it 0x99. Same kind of slip as the one
    // Set Select Mode carries, and the same resolution: follow the bytes
    return send_tag_operation(res, CMD_WRITE_TAG_MEMORY, param.data(), static_cast<uint16_t>(param.size()),
                              "write_tag_memory") &&
           tag_carried_it_out(res, "write_tag_memory");
}

bool UnitJRD4035::lock_tag_memory(const uint32_t payload, const uint32_t access_password)
{
    if (reject_while_polling("lock_tag_memory")) {
        return false;
    }
    std::vector<uint8_t> param{};
    if (!build_lock_tag(param, access_password, payload)) {
        M5_LIB_LOGE("Failed to build the lock parameter");
        return false;
    }
    Frame res{};
    return send_tag_operation(res, CMD_LOCK_TAG_MEMORY, param.data(), static_cast<uint16_t>(param.size()),
                              "lock_tag_memory") &&
           tag_carried_it_out(res, "lock_tag_memory");
}

bool UnitJRD4035::kill_tag(const uint32_t kill_password)
{
    if (reject_while_polling("kill_tag")) {
        return false;
    }
    std::vector<uint8_t> param{};
    if (!build_kill_tag(param, kill_password)) {
        M5_LIB_LOGE("Failed to build the kill parameter");
        return false;
    }
    Frame res{};
    return send_tag_operation(res, CMD_KILL_TAG, param.data(), static_cast<uint16_t>(param.size()), "kill_tag") &&
           tag_carried_it_out(res, "kill_tag");
}

bool UnitJRD4035::tag_carried_it_out(const Frame& response, const char* what) const
{
    TagOperationResult result{};
    if (!parse_tag_operation(result, response.parameter.data(), response.parameter.size())) {
        M5_LIB_LOGE("Malformed %s response", what);
        return false;
    }
    if (result.data_len != 1 || result.data[0] != TAG_OPERATION_SUCCESS) {
        M5_LIB_LOGE("%s reported status %02X", what, result.data_len ? result.data[0] : 0xFF);
        return false;
    }
    return true;
}

}  // namespace unit
}  // namespace m5
