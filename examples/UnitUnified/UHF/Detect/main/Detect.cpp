/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Detect UHF-RFID tags with Unit UHF-RFID (U107), and identify a single tag by its TID
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include <string>
#include <vector>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitUHFRFID unit{};
m5::uhf::UHFLayer uhf{unit};

//! @brief Words to read when the tag holds user memory but never says how much
constexpr uint16_t USER_PROBE_WORDS{2};
//! @brief Enough of the User bank to see what is in it without filling the log with it
constexpr uint16_t USER_DUMP_MAX_WORDS{16};

const char* region_to_string(const m5::uhf::Region r)
{
    switch (r) {
        case m5::uhf::Region::China900MHz:
            return "China900MHz";
        case m5::uhf::Region::America:
            return "America";
        case m5::uhf::Region::Europe:
            return "Europe";
        case m5::uhf::Region::China800MHz:
            return "China800MHz";
        case m5::uhf::Region::SouthKorea:
            return "SouthKorea";
        default:
            return "Unspecified";
    }
}

void print_channel_levels(const char* what, const m5::uhf::ChannelLevels& levels)
{
    std::string s{};
    for (size_t i = 0; i < levels.dbm.size(); ++i) {
        s += m5::utility::formatString("%u:%d ", (unsigned)(levels.first_channel + i), levels.dbm[i]);
    }
    M5_LOGI("%s per channel (dBm): %s", what, s.c_str());
}

void print_tag(const m5::uhf::Tag& tag)
{
    const std::string epc = tag.epc.toString();

    // The PC carries the EPC length, the user memory indicator and the numbering system,
    // which tells a lot about an unknown tag without reading any memory bank
    const uint8_t words  = m5::uhf::pcEPCLengthWords(tag.pc);
    const bool length_ok = (words * 2 == static_cast<int>(tag.epc.size));
    const bool crc_ok    = m5::unit::jrd4035::verify_tag_crc(tag);

    M5_LOGI("EPC : %s (%u bytes / %u bits)", epc.c_str(), (unsigned)tag.epc.size, (unsigned)(tag.epc.size * 8));
    M5_LOGI("PC  : %04X len=%uword UMI=%u XI=%u NSI=0x%03X -> length %s", tag.pc, words,
            m5::uhf::pcUserMemoryIndicator(tag.pc) ? 1 : 0, m5::uhf::pcXPCIndicator(tag.pc) ? 1 : 0,
            m5::uhf::pcNumberingSystemIdentifier(tag.pc), length_ok ? "matches" : "MISMATCH");
    M5_LOGI("CRC : %04X (%s)", tag.crc, crc_ok ? "valid" : "INVALID");
    M5_LOGI("RSSI: %d dBm", tag.rssi);

    lcd.printf("%s\n", epc.c_str());
    lcd.printf("  %ddBm UMI=%u CRC=%s\n", tag.rssi, m5::uhf::pcUserMemoryIndicator(tag.pc) ? 1 : 0,
               crc_ok ? "OK" : "NG");
}
std::string to_hex(const std::vector<uint8_t>& data)
{
    std::string s{};
    for (auto&& b : data) {
        s += m5::utility::formatString("%02X", b);
    }
    return s;
}

void dump_bank(const char* what, const m5::uhf::Bank bank, const uint16_t word_address, const uint16_t words)
{
    std::vector<uint8_t> data{};
    if (!uhf.readBank(data, bank, word_address, words)) {
        // A refusal is worth as much as the data here: a Reserved bank that will not be read is
        // how a tag says its access password has already been set
        M5_LOGW("%s: could not be read", what);
        return;
    }
    M5_LOGI("%s: %s", what, to_hex(data).c_str());
}

void identify_and_dump(const m5::uhf::Tag& detected)
{
    // Reading one word back is what proves the mask actually picks this tag out, so the failure
    // shows up here rather than halfway through the dump below
    if (!uhf.select(detected)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return;
    }

    m5::uhf::Tag tag{};
    if (!uhf.identify(tag)) {
        M5_LOGE("Failed to identify the tag");
        lcd.println("identify: failed");
        uhf.deselect();
        return;
    }

    M5_LOGI("TID   : %s", tag.tid.toString().c_str());
    M5_LOGI("Vendor: %s (MDID 0x%03X)", tag.vendorAsString().c_str(), tag.mdid);
    M5_LOGI("Chip  : %s (TMN 0x%03X)", tag.chipAsString().c_str(), tag.model_number);
    M5_LOGI("XTID  : %s serial=%ubit security=%u file=%u", tag.has_xtid ? "yes" : "no", tag.serial_bits,
            tag.supports_security ? 1 : 0, tag.supports_file ? 1 : 0);
    // Zero means the tag never said, not that the tag has none of it
    M5_LOGI("Sizes : user=%ubit maxEPC=%ubit permalockBlock=%ubit blockPermalock=%s", tag.user_memory_bits,
            tag.epc_max_bits, tag.permalock_block_bits, tag.supports_block_permalock ? "yes" : "no");

    lcd.printf("%s / %s\n", tag.vendorAsString().c_str(), tag.chipAsString().c_str());

    // Kill password then access password. All zeros is the state a tag leaves the factory in,
    // and it is also what keeps the tag from ever being killed by accident
    dump_bank("Reserved", m5::uhf::Bank::Reserved, 0, 4);

    // The XTID says how big the User bank is. When it stays silent the PC still carries an
    // indicator, so a short probe read is worth trying before giving up on the bank
    uint16_t user_words = static_cast<uint16_t>(tag.user_memory_bits / 16);
    if (user_words == 0 && m5::uhf::pcUserMemoryIndicator(tag.pc)) {
        user_words = USER_PROBE_WORDS;
        M5_LOGI("User  : size unknown, probing %u words", user_words);
    }
    if (user_words > USER_DUMP_MAX_WORDS) {
        user_words = USER_DUMP_MAX_WORDS;
    }
    if (user_words) {
        dump_bank("User", m5::uhf::Bank::User, 0, user_words);
    } else {
        M5_LOGI("User  : the tag reports none");
    }

    uhf.deselect();
}
}  // namespace

void setup()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    // Unit UHF-RFID is a UART unit; PortC is preferred and PortA is the fallback
    if (!(m5::unit::wiring::addUART(Units, unit, 115200) && Units.begin())) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // Reader settings are rejected while polling, so stop it first
    unit.stopPolling();

    // Show the module settings
    m5::uhf::ModuleInformation info{};
    if (unit.readModuleInformation(info)) {
        M5_LOGI("HW:%s SW:%s MFR:%s", info.hardware_version.c_str(), info.software_version.c_str(),
                info.manufacturer.c_str());
    }
    int16_t dbm100{};
    if (unit.readTransmitPower(dbm100)) {
        M5_LOGI("TxPower: %d.%02d dBm", dbm100 / 100, dbm100 % 100);
    }
    m5::uhf::Region region{};
    if (unit.readRegion(region)) {
        M5_LOGI("Region: %s", region_to_string(region));
    }
    uint8_t channel{};
    if (unit.readChannel(channel)) {
        M5_LOGI("Channel: %u", channel);
    }
    // Select acts on the inventoried flag of one session, so which session and which flag value
    // the module then queries for decides whether a selected tag is included or excluded
    m5::uhf::QueryParameters qp{};
    if (unit.readQueryParameters(qp)) {
        static const char* filters[] = {"All", "All", "NotSelected", "Selected"};
        M5_LOGI("Query: Q=%u session=S%u target=%c sel=%s", qp.q, (unsigned)qp.session,
                qp.target == m5::uhf::Target::A ? 'A' : 'B', filters[static_cast<uint8_t>(qp.filter) & 0x03]);
    }

    // Survey the band before reading anything. A tag that refuses to answer while the antenna
    // and the transmit power are both fine is usually drowned out by something on the air, and
    // these two scans are what tells that apart from a problem with the tag
    m5::uhf::ChannelLevels levels{};
    if (unit.readBlockingSignal(levels)) {
        print_channel_levels("Blocking", levels);
    }
    if (unit.readChannelRSSI(levels)) {
        print_channel_levels("RSSI", levels);
    }

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: detect + identify");
}

void loop()
{
    M5.update();
    Units.update();

    // Button A runs the blocking, deduplicated detection of UHFLayer
    if (M5.BtnA.wasClicked()) {
        std::vector<m5::uhf::Tag> tags{};
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        if (uhf.detect(tags, 1000)) {
            lcd.printf("detect: %u tag(s)\n", (unsigned)tags.size());
            for (size_t i = 0; i < tags.size(); ++i) {
                print_tag(tags[i]);
            }
            // Which tag to address is only unambiguous while there is exactly one in the field
            if (tags.size() == 1) {
                identify_and_dump(tags[0]);
            } else {
                M5_LOGI("Put a single tag in the field to identify it");
            }
        } else {
            lcd.println("detect: no tag");
        }
        return;
    }

    // The raw queue keeps every notification, so the same tag appears repeatedly
    while (unit.available()) {
        print_tag(unit.oldest());
        unit.discard();
    }
    if (unit.dropped()) {
        M5_LOGW("Dropped %u notifications; raise tag_queue_size", unit.dropped());
        unit.clearDropped();
    }
}

#if !defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#if CONFIG_FREERTOS_UNICORE
static inline void feedIdleTaskPeriodically(void)
{
    constexpr uint32_t FEED_INTERVAL_MS   = 2000;
    constexpr TickType_t FEED_SLEEP_TICKS = pdMS_TO_TICKS(5);
    static uint32_t s_next_feed_ms        = 0;
    const uint32_t now_ms                 = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (now_ms >= s_next_feed_ms) {
        s_next_feed_ms = now_ms + FEED_INTERVAL_MS;
        vTaskDelay(FEED_SLEEP_TICKS);
    }
}
#endif

extern "C" void app_main(void)
{
    setup();
    for (;;) {
#if CONFIG_FREERTOS_UNICORE
        feedIdleTaskPeriodically();
#endif
        loop();
    }
}
#endif
