/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Switch an Impinj Monza 4QT between the two memory maps it keeps

  A Monza 4QT holds two profiles and shows one at a time. The private one carries a 128-bit
  EPC, 512 bits of user memory, and a TID with an XTID header and a serial number. The public
  one hides all of that and answers with a 96-bit EPC kept for the purpose, leaving nothing
  but the two words of TID that name the chip. Impinj's own example is a shop: the private
  profile carries what the goods are, and the tag is publicised at the till so that anyone
  with a reader afterwards learns nothing.

  Switching is done with the QT command, which needs the tag secured, so an access password
  protects it. The chip also offers a short-range mode that drops its sensitivity by about
  15dB once it is open or secured, which stops a reader more than about a metre away from
  switching it at all.

  What is written here has to outlast the tag losing power, because it does: the reader stops
  transmitting between one operation and the next, and a volatile switch is gone by the time
  the tag is looked for again. So the switch is written to stay.

  Clicking switches the tag and switches it straight back, which shows both maps and leaves
  the tag as it was found. Holding switches it and stops there, which is what to do to have a
  tag on its public map to work with.

  Neither direction is a one-way door. A tag left on its public map answers under an EPC of
  all zeroes, has no user memory and gives up only the two words of TID that name the chip,
  so every other example will find much less of it than before. Holding the button again puts
  it back.
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>
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
    // receive buffer holds only about 22ms of them at this baud rate
#if defined(ARDUINO)
    constexpr size_t RX_BUFFER_BYTES{2048};
    auto& serial = m5::unit::wiring::defaultUartSerial();
    serial.end();
    if (serial.setRxBufferSize(RX_BUFFER_BYTES) != RX_BUFFER_BYTES) {
        M5_LOGW("The receive buffer kept its default size; frames may arrive in pieces");
    }
#endif
    if (!(m5::unit::wiring::addUART(Units, unit, 115200) && Units.begin())) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());
}

//! @brief The one tag in the field, or none
bool detect_one(m5::uhf::Tag& tag)
{
    std::vector<m5::uhf::Tag> tags{};
    if (!uhf.detect(tags) || tags.empty()) {
        M5_LOGI("No tag in the field");
        lcd.println("no tag");
        return false;
    }
    if (tags.size() > 1) {
        M5_LOGW("%zu tags in the field; leave one of them", tags.size());
        lcd.printf("%zu tags: leave one\n", tags.size());
        return false;
    }
    tag = tags.front();
    return true;
}

//! @brief Report what the tag is showing, which is what a switch has to be judged against
void report(const m5::uhf::Tag& tag, const m5::uhf::QTParameters& qt)
{
    M5_LOGI("PC    : %04X (%u word EPC, UMI=%u, XI=%u)", tag.pc, m5::uhf::pcEPCLengthWords(tag.pc),
            m5::uhf::pcUserMemoryIndicator(tag.pc) ? 1 : 0, m5::uhf::pcXPCIndicator(tag.pc) ? 1 : 0);
    M5_LOGI("EPC   : %s", tag.epc.toString().c_str());
    M5_LOGI("TID   : %s (%u bytes, XTID %s)", tag.tid.toString().c_str(), tag.tid.size, tag.has_xtid ? "yes" : "no");
    M5_LOGI("QT    : %s memory, short range %s", qt.public_memory ? "public" : "private",
            qt.short_range ? "on" : "off");
}

//! @brief Select the tag, read what it is showing, and report it
//! @param[in,out] tag Tag to work on, filled in by identify()
//! @param[out] qt What the tag answered
//! @return True if the tag was selected and answered
bool look(m5::uhf::Tag& tag, m5::uhf::QTParameters& qt)
{
    if (!uhf.select(tag)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return false;
    }
    // identify() fills in the TID, which is the half that the public map hides
    if (!uhf.identify(tag)) {
        M5_LOGW("Could not read the tag's TID");
        // A public tag has only the two words that name the chip, so reading the whole of a TID
        // overruns it. Those two are still there, and what they say about an extended TID is
        // what decides how much of a TID is worth asking for
        std::vector<uint8_t> base{};
        if (uhf.readBank(base, m5::uhf::Bank::Tid, 0, 2) && base.size() >= 4) {
            M5_LOGI("TID base: %02X%02X%02X%02X (XTID indicator %u)", base[0], base[1], base[2], base[3],
                    (base[1] & 0x80) ? 1 : 0);
        }
    }
    const auto read = uhf.readQTParameters(qt);
    if (!read) {
        M5_LOGE("The QT command did not go through (%s); is it a Monza 4QT?", m5::uhf::reasonAsString(read.error()));
        lcd.println("QT: no answer");
        uhf.deselect();
        return false;
    }
    report(tag, qt);
    return true;
}

//! @brief Show the tag as it is, switch it to the other map, and show it again
//! @param keep Leave the tag on the map it was switched to, rather than putting it back
//! @details Both halves are worth seeing. What the private map holds is what the public one
//! hides, and the tag answers under a different EPC afterwards, so it has to be found again
void switch_map(const bool keep)
{
    m5::uhf::Tag before{};
    if (!detect_one(before)) {
        return;
    }
    m5::uhf::QTParameters qt{};
    if (!look(before, qt)) {
        return;
    }

    m5::uhf::QTParameters wanted = qt;
    wanted.public_memory         = !qt.public_memory;
    M5_LOGI("Switching to the %s map", wanted.public_memory ? "public" : "private");
    lcd.printf("-> %s\n", wanted.public_memory ? "public" : "private");
    // Written to stay: a volatile switch would be gone before the tag is looked for again
    const auto switched = uhf.writeQTParameters(wanted, true);
    if (!switched) {
        M5_LOGE("Failed to switch the map: %s", m5::uhf::reasonAsString(switched.error()));
        lcd.println("switch: failed");
        uhf.deselect();
        return;
    }
    uhf.deselect();

    // The tag answers under a different EPC now, so it is looked for rather than selected again
    m5::uhf::Tag after{};
    if (!detect_one(after)) {
        M5_LOGE("The tag did not answer after the switch");
        return;
    }
    m5::uhf::QTParameters qt_after{};
    if (!look(after, qt_after)) {
        return;
    }
    if (keep) {
        M5_LOGW("Left on the %s map; hold the button again to put it back",
                qt_after.public_memory ? "public" : "private");
        lcd.printf("left %s\n", qt_after.public_memory ? "public" : "private");
        uhf.deselect();
        return;
    }
    // Whichever way it went, the tag is put back the way it was found
    M5_LOGI("Switching back to the %s map", qt.public_memory ? "public" : "private");
    const auto back = uhf.writeQTParameters(qt, true);
    if (!back) {
        M5_LOGE("Failed to switch it back (%s); it is showing its %s map", m5::uhf::reasonAsString(back.error()),
                qt_after.public_memory ? "public" : "private");
        lcd.println("NOT switched back");
        uhf.deselect();
        return;
    }
    uhf.deselect();
    M5_LOGI("Back to the %s map", qt.public_memory ? "public" : "private");
    lcd.println("back as found");
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: show both maps");
    lcd.println("A hold: switch and stay");
    lcd.println("(Monza 4QT only)");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        switch_map(false);
    }
    if (M5.BtnA.wasHold()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        switch_map(true);
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
