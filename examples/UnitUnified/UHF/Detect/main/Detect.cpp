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
#include <cstdlib>
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
    // Notifications keep coming while the module is polling, and the driver's default 256-byte
    // receive buffer holds only about 22ms of them at this baud rate. Anything that keeps the
    // sketch busy for longer than that, printing included, costs bytes out of the middle of a
    // frame. The port has to be closed for a new size to be accepted
#if defined(ARDUINO)
    constexpr size_t RX_BUFFER_BYTES{2048};
    auto& serial = m5::unit::wiring::defaultUartSerial();
    serial.end();
    // The call answers with the size it settled on, and with zero when it would not take. There
    // is no way to ask afterwards, so this is the only chance to find out
    if (serial.setRxBufferSize(RX_BUFFER_BYTES) != RX_BUFFER_BYTES) {
        M5_LOGW("The receive buffer kept its default size; frames may arrive in pieces");
    }
#endif
    // Unit UHF-RFID is a UART unit; PortC is preferred and PortA is the fallback
    if (!(m5::unit::wiring::addUART(Units, unit, 115200) && Units.begin())) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    // The module keeps these across a power cycle, so what it holds now is whatever was last
    // written to it rather than a default. They decide how far a tag can be and how faint an
    // answer still counts, which is the difference between a tag that reads and one that does
    // not. What is there is reported, not changed: how much power may be radiated is a question
    // of where the unit is being used
    int16_t dbm100{};
    if (unit.readTransmitPower(dbm100)) {
        M5_LOGI("Transmit power: %d.%02d dBm", dbm100 / 100, dbm100 % 100);
    }
    m5::unit::m100::DemodulatorParameters dp{};
    if (unit.readDemodulatorParameters(dp)) {
        M5_LOGI("Demodulator: mixer=%udB if=%udB threshold=0x%04X", m5::unit::m100::mixerGainDb(dp.mixer_gain),
                m5::unit::m100::ifGainDb(dp.if_gain), dp.threshold);
    }

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
}

/*
  Watching the field through the raw queue.

  The queue is deliberately not deduplicated: a tag that stays put is reported once per inventory
  round, about every 30ms, each time with a fresh RSSI. That repetition is the whole point of the
  queue, and it is what tells an application that a tag is still there, how well it is coupling,
  and when it has gone. Printing every report would say none of that and would saturate the serial
  line while it failed to, so what is tracked here is the change rather than the reports.

  RSSI wanders about 10dB on its own with the tag sitting still, so only a swing wider than that
  is worth calling a change.
*/
//! @brief How long a tag has to go unreported before it counts as gone
constexpr unsigned long GONE_AFTER_MS{1000};
//! @brief How far RSSI has to move before it is a change rather than the usual wander
constexpr int RSSI_REPORT_DB{10};

// Members left without initialisers so that C++11 still treats this as an aggregate: every one
// of them is given a value at the only place a Seen is made
struct Seen {
    m5::uhf::Epc epc;
    int8_t rssi;
    unsigned long at;
};
std::vector<Seen> seen{};

void draw_field()
{
    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setCursor(0, 0);
    lcd.printf("in field: %u\n", (unsigned)seen.size());
    for (auto&& s : seen) {
        lcd.printf("%s %ddBm\n", s.epc.toString().c_str(), s.rssi);
    }
}

void note(const m5::uhf::Tag& tag)
{
    const unsigned long now = m5::utility::millis();
    for (auto&& s : seen) {
        if (s.epc == tag.epc) {
            if (std::abs(static_cast<int>(tag.rssi) - static_cast<int>(s.rssi)) >= RSSI_REPORT_DB) {
                M5_LOGI("~ %s %ddBm -> %ddBm", s.epc.toString().c_str(), s.rssi, tag.rssi);
                s.rssi = tag.rssi;
                draw_field();
            }
            s.at = now;
            return;
        }
    }
    M5_LOGI("+ %s %ddBm", tag.epc.toString().c_str(), tag.rssi);
    seen.push_back(Seen{tag.epc, tag.rssi, now});
    draw_field();
}

//! @brief Drop the tags that have stopped being reported
void expire()
{
    const unsigned long now = m5::utility::millis();
    for (size_t i = seen.size(); i > 0; --i) {
        auto&& s = seen[i - 1];
        if (now - s.at >= GONE_AFTER_MS) {
            M5_LOGI("- %s gone", s.epc.toString().c_str());
            seen.erase(seen.begin() + (i - 1));
            draw_field();
        }
    }
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
    draw_field();
}

void loop()
{
    M5.update();
    Units.update();

    // Button A asks UHFLayer for one deduplicated look at the field and spells each tag out in
    // full, which is what an application wants when it needs a list rather than a stream
    if (M5.BtnA.wasClicked()) {
        std::vector<m5::uhf::Tag> tags{};
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        if (uhf.detect(tags)) {
            lcd.printf("detect: %u tag(s)\n", (unsigned)tags.size());
            for (auto&& tag : tags) {
                print_tag(tag);
                // detect() drains the queue itself and takes its whole window to do it, so the
                // tags it found have to be handed to the tracker or they look like they left
                note(tag);
            }
        } else {
            lcd.println("detect: no tag");
        }
        draw_field();
        return;
    }

    // Everything the module reports, turned into arrivals, departures and changes of coupling
    while (unit.available()) {
        note(unit.oldest());
        unit.discard();
    }
    expire();

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
