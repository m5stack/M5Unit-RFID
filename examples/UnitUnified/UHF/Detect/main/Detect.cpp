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

//! @brief Bring the display and the unit up, or stop
void begin_unit()
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

    m5::uhf::ModuleInformation info{};
    if (unit.readModuleInformation(info)) {
        M5_LOGI("HW:%s SW:%s MFR:%s", info.hardware_version.c_str(), info.software_version.c_str(),
                info.manufacturer.c_str());
    }
}

void print_tag(const m5::uhf::Tag& tag)
{
    const std::string epc = tag.epc.toString();

    // The PC carries the EPC length, the user memory indicator and the numbering system, which
    // tells a lot about an unknown tag without reading any memory bank
    const uint8_t words  = m5::uhf::pcEPCLengthWords(tag.pc);
    const bool length_ok = (words * 2 == static_cast<int>(tag.epc.size));
    const bool crc_ok    = m5::uhf::verify_tag_crc(tag);

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
}  // namespace

void setup()
{
    begin_unit();

    // Polling makes the module run inventory rounds continuously and report every tag it sees.
    // The queue those reports land in is not deduplicated, which is what lets an application
    // watch a tag's RSSI change or notice that it has gone
    if (!unit.startPolling(unit.config().polling_count)) {
        M5_LOGE("Failed to start polling");
    }

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: detect() once");
    lcd.println("raw stream below");
}

void loop()
{
    M5.update();
    Units.update();

    // Button A asks UHFLayer for one deduplicated look at the field, which is what an
    // application wants when it needs a list rather than a stream
    if (M5.BtnA.wasClicked()) {
        std::vector<m5::uhf::Tag> tags{};
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        if (uhf.detect(tags, 1000)) {
            lcd.printf("detect: %u tag(s)\n", (unsigned)tags.size());
            for (auto&& tag : tags) {
                print_tag(tag);
            }
        } else {
            lcd.println("detect: no tag");
        }
        return;
    }

    // The raw queue keeps every notification, so a tag that stays in the field appears again
    // and again with a fresh RSSI each time
    while (unit.available()) {
        print_tag(unit.oldest());
        unit.discard();
    }
    if (unit.dropped()) {
        M5_LOGW("Dropped %u notifications; raise tag_queue_size or update() more often", unit.dropped());
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
