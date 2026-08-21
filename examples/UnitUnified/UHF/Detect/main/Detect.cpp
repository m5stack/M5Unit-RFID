/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Detect UHF-RFID tags with Unit UHF-RFID (U107)
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

void print_tag(const m5::uhf::Tag& tag)
{
    std::string epc{};
    for (size_t i = 0; i < tag.epc.size(); ++i) {
        epc += m5::utility::formatString("%02X", tag.epc[i]);
    }

    // The PC carries the EPC length, the user memory indicator and the numbering system,
    // which tells a lot about an unknown tag without reading any memory bank
    const uint8_t words  = m5::uhf::pcEPCLengthWords(tag.pc);
    const bool length_ok = (words * 2 == static_cast<int>(tag.epc.size()));
    const bool crc_ok    = m5::unit::jrd4035::verify_tag_crc(tag);

    M5_LOGI("EPC : %s (%u bytes / %u bits)", epc.c_str(), (unsigned)tag.epc.size(), (unsigned)(tag.epc.size() * 8));
    M5_LOGI("PC  : %04X len=%uword UMI=%u XI=%u NSI=0x%03X -> length %s", tag.pc, words,
            m5::uhf::pcUserMemoryIndicator(tag.pc) ? 1 : 0, m5::uhf::pcXPCIndicator(tag.pc) ? 1 : 0,
            m5::uhf::pcNumberingSystemIdentifier(tag.pc), length_ok ? "matches" : "MISMATCH");
    M5_LOGI("CRC : %04X (%s)", tag.crc, crc_ok ? "valid" : "INVALID");
    M5_LOGI("RSSI: %d dBm", tag.rssi);

    lcd.printf("%s\n", epc.c_str());
    lcd.printf("  %ddBm UMI=%u CRC=%s\n", tag.rssi, m5::uhf::pcUserMemoryIndicator(tag.pc) ? 1 : 0,
               crc_ok ? "OK" : "NG");
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

    // Resume detection now that the settings have been read
    unit.startPolling(unit.config().polling_count);

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: detect()  raw stream below");
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
