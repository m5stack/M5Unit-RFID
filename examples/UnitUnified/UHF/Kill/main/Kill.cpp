/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Kill a UHF-RFID tag with Unit UHF-RFID (U107)

  WARNING: a killed tag stops answering for good. There is no command that brings one back, and
  no reader that can reach it. Whatever was on it is gone with it.

  This sketch is in two halves, and only the second one destroys anything.

  - A short press reads the tag's kill password and asks it to die with what it finds. A tag
    that left the factory carries a zero password, EPC Gen2 forbids killing on one, and the
    attempt is refused. Nothing happens to the tag, and both the refusal from this library and
    the refusal from the tag can be watched.
  - A long press, twice, writes a kill password and then kills the tag. The first press only
    arms it and says so; the second one carries it out. The arming lapses after a few seconds.
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

//! @brief The password written to the tag before it is killed. Any nonzero value does
constexpr uint32_t KILL_PASSWORD{0x5A5AA5A5};
//! @brief Sent to a tag whose stored password is known to be zero, to watch it refuse
constexpr uint32_t PROBE_PASSWORD{0x12345678};
//! @brief How long the arming lasts. Long enough to think, short enough not to be forgotten
constexpr uint32_t ARMED_FOR_MS{8000};
//! @brief Word address of the kill password in the Reserved bank, fixed by EPC Gen2
constexpr uint16_t KILL_PASSWORD_WORD{0};

unsigned long armed_at{};

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
}

//! @brief The one tag in the field, or none. Killing the wrong tag cannot be taken back
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

//! @brief Read the kill password the tag holds. Returns false when it could not be read
bool read_kill_password(uint32_t& password)
{
    std::vector<uint8_t> data{};
    if (!uhf.readBank(data, m5::uhf::Bank::Reserved, KILL_PASSWORD_WORD, 2) || data.size() < 4) {
        M5_LOGE("Could not read the kill password");
        return false;
    }
    password = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
               (static_cast<uint32_t>(data[2]) << 8) | data[3];
    return true;
}

//! @brief Say what the tag is, so that the log records what was addressed and what was ended
void name_tag(m5::uhf::Tag& tag)
{
    if (!uhf.identify(tag)) {
        M5_LOGW("Could not read the TID; the tag is not named below");
        return;
    }
    M5_LOGI("Chip: %s", tag.chipAsString().c_str());
    lcd.printf("%s\n", tag.chipAsString().c_str());
}

//! @brief How many rounds out of ATTEMPTS the tag answered
//! @details One round is no proof either way. A tag that is alive is missed often enough that a
//! single silence would read as a kill, so the same count is taken before and after and the two
//! are compared. Before says whether the tag was there to begin with; after says whether it is
//! still there
uint8_t times_answered(const m5::uhf::Tag& target)
{
    constexpr uint8_t ATTEMPTS{3};
    uint8_t answered = 0;
    for (uint8_t i = 0; i < ATTEMPTS; ++i) {
        std::vector<m5::uhf::Tag> tags{};
        if (uhf.detect(tags)) {
            for (auto&& t : tags) {
                if (t.epc == target.epc) {
                    ++answered;
                    break;
                }
            }
        }
    }
    return answered;
}
constexpr uint8_t DETECT_ATTEMPTS{3};

//! @brief Show what the tag holds and let it refuse, without doing it any harm
void dry_run(m5::uhf::Tag& detected)
{
    if (!uhf.select(detected)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return;
    }
    name_tag(detected);
    uint32_t stored{};
    const bool read = read_kill_password(stored);
    uhf.deselect();
    if (!read) {
        lcd.println("password: unreadable");
        return;
    }
    M5_LOGI("Kill password on the tag: %08lX", (unsigned long)stored);
    lcd.printf("password %08lX\n", (unsigned long)stored);

    if (stored != 0) {
        // Sending this one would kill the tag, so it is not sent
        M5_LOGW("This tag has a kill password. A long press would end it; nothing is sent here");
        lcd.println("armed tag: not sent");
        return;
    }

    // Zero, so nothing that follows can destroy it. First this library refuses to send at all,
    // then the tag itself refuses a password it never agreed to
    if (!uhf.select(detected)) {
        return;
    }
    M5_LOGI("Killing with the zero password it holds: %s",
            uhf.kill(detected, stored) ? "sent, which it should not be" : "refused before sending");
    M5_LOGI("Killing with a password it does not hold: %s",
            uhf.kill(detected, PROBE_PASSWORD) ? "the tag accepted it" : "the tag refused it");
    uhf.deselect();
    lcd.println("refused, as it should");
}

//! @brief Give the tag a password and use it. Nothing here can be undone
void kill_for_good(m5::uhf::Tag& detected)
{
    if (!uhf.select(detected)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return;
    }
    name_tag(detected);
    const uint8_t before = times_answered(detected);
    M5_LOGI("Before: answered %u of %u rounds", before, DETECT_ATTEMPTS);
    if (before == 0) {
        M5_LOGE("The tag is not answering at all; nothing is written and nothing is killed");
        lcd.println("not answering");
        uhf.deselect();
        return;
    }

    const std::vector<uint8_t> password{static_cast<uint8_t>(KILL_PASSWORD >> 24),
                                        static_cast<uint8_t>(KILL_PASSWORD >> 16),
                                        static_cast<uint8_t>(KILL_PASSWORD >> 8), static_cast<uint8_t>(KILL_PASSWORD)};
    if (!uhf.writeBank(m5::uhf::Bank::Reserved, KILL_PASSWORD_WORD, password.data(),
                       static_cast<uint16_t>(password.size()))) {
        M5_LOGE("Could not write the kill password; the tag is untouched");
        lcd.println("password: not written");
        uhf.deselect();
        return;
    }
    // A write the tag answered is not the same as a write that landed. Sending a kill with a
    // password the tag does not hold would fail for a reason that looks nothing like the cause
    uint32_t stored{};
    if (!read_kill_password(stored) || stored != KILL_PASSWORD) {
        M5_LOGE("The tag holds %08lX, not the kill password; nothing is killed", (unsigned long)stored);
        lcd.println("password: not held");
        uhf.deselect();
        return;
    }
    M5_LOGW("Kill password written and read back. The tag can now be killed");

    const bool killed = uhf.kill(detected, KILL_PASSWORD);
    uhf.deselect();
    M5_LOGW("Kill: %s", killed ? "the tag carried it out" : "failed");

    // The answer to a kill can go missing on the way back, and a tag that did die cannot say so.
    // Asking the field is the only way to tell those two apart
    const uint8_t after = times_answered(detected);
    M5_LOGW("After: answered %u of %u rounds, against %u before", after, DETECT_ATTEMPTS, before);
    lcd.printf("%u/%u -> %u/%u\n", before, DETECT_ATTEMPTS, after, DETECT_ATTEMPTS);
    if (after == 0) {
        M5_LOGW("The tag is gone");
        lcd.println("gone");
    } else {
        M5_LOGW("The tag still answers, so it is alive");
        lcd.println("still alive");
    }
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: read and refuse");
    lcd.println("Hold twice: KILL");
    lcd.println("(cannot be undone)");
}

void loop()
{
    M5.update();
    Units.update();

    if (armed_at && m5::utility::millis() - armed_at > ARMED_FOR_MS) {
        armed_at = 0;
        M5_LOGI("No longer armed");
        lcd.println("no longer armed");
    }

    if (M5.BtnA.wasHold()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        if (!armed_at) {
            armed_at = m5::utility::millis();
            M5_LOGW("Armed. Hold again to write a kill password and end the tag for good");
            lcd.println("ARMED");
            lcd.println("hold again to KILL");
            lcd.printf("or wait %us\n", (unsigned)(ARMED_FOR_MS / 1000));
            return;
        }
        armed_at = 0;
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            kill_for_good(detected);
        }
        return;
    }

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            dry_run(detected);
        }
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
