/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  The two low power modes of Unit UHF-RFID (U107), and what each of them costs

  A module that is polling draws current continuously, and one that is doing nothing still
  draws enough to matter on a battery. There are two ways to stop that, and they are not
  interchangeable.

  Sleep turns the module off. Waking it resets the chip and reloads its firmware, and the mask
  naming the tag being addressed does not come back: the module returns holding a mask of zero
  length, which matches every tag rather than none. A read afterwards therefore still succeeds,
  against whichever tag answers, which is what makes this worth seeing rather than reading.

  IDLE turns off the radio and leaves the rest alone. What the module was holding is still
  there, the mask included, and it goes on answering commands.

  Holding the button takes one tag through both, and through the layer's sleep as well, which
  stores the mask again on the way back. Clicking measures how long waking takes.
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include <vector>
#include <cstring>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitUHFRFID unit{};
m5::uhf::UHFLayer uhf{unit};

//! @brief Word address the EPC starts at, after the stored CRC and the PC (Gen2 v2.1 6.3.2.1.2)
constexpr uint16_t EPC_FIRST_WORD{2};

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

    m5::uhf::ModuleInformation info{};
    if (unit.readModuleInformation(info)) {
        M5_LOGI("HW:%s SW:%s MFR:%s", info.hardware_version.c_str(), info.software_version.c_str(),
                info.manufacturer.c_str());
    }
}

//! @brief Ask the module one short question, sent once
//! @return True if it answered
//! @details The transmit power is asked for rather than the module information, because that
//! one sends the same command three times over and retries each: a report counting attempts
//! wants a call that makes exactly one
bool module_answers()
{
    int16_t dbm100{};
    return unit.readTransmitPower(dbm100);
}

//! @brief How many commands to send a sleeping module before giving up on one being answered
constexpr int PROBE_ATTEMPTS{4};

//! @brief Waits tried after waking, in milliseconds, before the module is asked anything
constexpr uint32_t WAKE_WAITS_MS[]{0, 250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2500, 3000};

//! @brief How long to leave the module alone between one sleep and the next
constexpr uint32_t SETTLE_MS{1000};

//! @brief Find how long the module needs after being woken before it answers
//! @details A command that fails takes the whole command timeout to say so, which is too coarse
//! to measure with. Waiting a set time before asking anything, and lengthening it until the
//! first question is answered, gives a figure as fine as the steps below
//! @note wake() waits on its own account before returning, and that wait is inside every figure
//! here. What is reported is the time from the wake byte, not from wake() returning
void measure_wake_time()
{
    for (const uint32_t wait : WAKE_WAITS_MS) {
        // Sleeping and waking again immediately has been seen to trip the module's watchdog,
        // which answers 0x05 and leaves it not answering at all for a while
        m5::utility::delay(SETTLE_MS);
        if (!unit.sleep()) {
            M5_LOGE("The module would not go to sleep");
            lcd.println("sleep: failed");
            return;
        }
        if (!unit.wake()) {
            M5_LOGE("Failed to send the wake byte");
            lcd.println("wake: failed");
            return;
        }
        const unsigned long woke_at = m5::utility::millis();
        m5::utility::delay(wait);
        if (module_answers()) {
            const unsigned long took = m5::utility::millis() - woke_at;
            M5_LOGI("Answered %lums after waking, having been left alone for %ums", took, wait);
            lcd.printf("wakes in %lums\n", took);
            return;
        }
        M5_LOGI("Left alone for %ums, and the question that followed went unanswered", wait);
    }
    M5_LOGW("Nothing was answered within %ums of waking", WAKE_WAITS_MS[sizeof(WAKE_WAITS_MS) / sizeof(uint32_t) - 1]);
    lcd.println("no answer");
}

//! @brief Sleep, then send commands until one is answered, counting them
//! @details A sleeping module is woken by the first byte it receives and throws that byte away,
//! which is why wake() sends one on its own. A command frame starts with a byte like any other,
//! so sending one wakes the module and is lost doing it. This is the reason a single command
//! that fails against a sleeping module says nothing about whether the module is reachable
void sleep_then_probe()
{
    if (!unit.sleep()) {
        M5_LOGE("The module would not go to sleep");
        lcd.println("sleep: failed");
        return;
    }
    M5_LOGI("Asleep");
    const unsigned long began = m5::utility::millis();
    for (int attempt = 1; attempt <= PROBE_ATTEMPTS; ++attempt) {
        if (module_answers()) {
            M5_LOGI("Command %d of %d was answered, %lums after the module went to sleep", attempt, PROBE_ATTEMPTS,
                    m5::utility::millis() - began);
            lcd.printf("woken by command %d\n", attempt);
            return;
        }
    }
    M5_LOGI("%d commands went unanswered over %lums", PROBE_ATTEMPTS, m5::utility::millis() - began);
    lcd.println("still asleep");
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

//! @brief Read the first word of the EPC through whatever selection is in force
//! @return True if a tag answered
//! @warning This says a tag answered and no more than that. With one tag in the field it reads
//! whether or not a mask is stored, so it cannot tell a selection that survived from a module
//! that has none and is answering with the only tag there is
bool epc_word_reads()
{
    std::vector<uint8_t> word{};
    return uhf.readBank(word, m5::uhf::Bank::Epc, EPC_FIRST_WORD, 1);
}

//! @brief Report the mask the module is holding, and hand back what it said
//! @param[out] sp Select parameter the module answered with
//! @return True if the module answered
bool report_selection(const char* when, m5::uhf::SelectParameter& sp)
{
    if (!unit.readSelectParameter(sp)) {
        M5_LOGE("%s: the module would not say what mask it holds", when);
        return false;
    }
    M5_LOGI("%s: bank=%u ptr=%ubit len=%ubit mask=%s", when, static_cast<uint8_t>(sp.bank), sp.pointer_bits,
            sp.mask_length_bits, sp.maskAsString().c_str());
    return true;
}

//! @brief Is this the same mask, byte for byte?
bool same_selection(const m5::uhf::SelectParameter& a, const m5::uhf::SelectParameter& b)
{
    return a.bank == b.bank && a.pointer_bits == b.pointer_bits && a.mask_length_bits == b.mask_length_bits &&
           a.mask_size == b.mask_size && memcmp(a.mask.data(), b.mask.data(), a.mask_size) == 0;
}

//! @brief What to put the module into, and what to call it in the report
enum class LowPower : uint8_t {
    UnitSleep,   //!< The module's own sleep, and nothing else
    LayerSleep,  //!< The same sleep through the layer, which stores the mask again on the way back
    Idle,        //!< IDLE, which keeps what the module was holding
};

//! @brief Take a selected tag down into one low power mode and back, and say what survived
//! @param detected Tag to work on
//! @param mode Which of the three ways down to use
//! @return True if the module went down and came back
//! @details The three are worth seeing side by side. The module's own sleep loses the mask.
//! The layer stores it again on the way back. IDLE never loses it, which is what makes it the
//! one to reach for while a tag is being addressed
bool selection_across(const m5::uhf::Tag& detected, const LowPower mode)
{
    {
        const char* who = (mode == LowPower::Idle) ? "idle" : (mode == LowPower::LayerSleep ? "layer" : "unit");
        if (!uhf.select(detected)) {
            M5_LOGE("Failed to select the tag");
            lcd.println("select: failed");
            return false;
        }
        m5::uhf::SelectParameter before{};
        if (!report_selection("Before going down", before)) {
            uhf.deselect();
            return false;
        }

        bool down{};
        switch (mode) {
            case LowPower::UnitSleep:
                down = unit.sleep();
                break;
            case LowPower::LayerSleep:
                down = uhf.sleep();
                break;
            case LowPower::Idle:
                down = unit.writeIdle(true);
                break;
        }
        if (!down) {
            M5_LOGE("The module would not go into %s", who);
            uhf.deselect();
            return false;
        }
        M5_LOGI("Down through the %s", who);

        bool up{};
        switch (mode) {
            case LowPower::UnitSleep:
                up = unit.wake();
                break;
            case LowPower::LayerSleep:
                up = uhf.wake();
                break;
            case LowPower::Idle:
                // Nothing wakes IDLE: the next command does it. Asking it to leave is only so
                // that the report below is of a module that is fully back
                up = unit.writeIdle(false);
                break;
        }
        if (!up) {
            M5_LOGE("Failed to come back through the %s", who);
            uhf.deselect();
            return false;
        }

        m5::uhf::SelectParameter after{};
        if (report_selection("After coming back", after)) {
            const bool kept = same_selection(before, after);
            M5_LOGI("Through the %s, the mask the module holds: %s", who, kept ? "is the one it had" : "IS NOT");
            lcd.printf("%s: mask %s\n", who, kept ? "kept" : "lost");
        }
        // A mask of zero length matches every tag, so this succeeds either way with one tag in
        // the field. That is what makes a lost selection something a caller does not notice
        M5_LOGI("Through the %s, reading the tag as before: %s", who, epc_word_reads() ? "it answers" : "nothing does");
        uhf.deselect();
    }
    return true;
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: sleep, with a tag");
    lcd.println("   or without, how long");
    lcd.println("   waking takes");
    lcd.println("A hold: idle, with a tag");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        // With a tag to hand, what a sleep does to a selection is the thing worth seeing. With
        // none, the module on its own is all there is to measure
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            if (selection_across(detected, LowPower::UnitSleep)) {
                selection_across(detected, LowPower::LayerSleep);
            }
        } else {
            measure_wake_time();
            sleep_then_probe();
        }
    }
    if (M5.BtnA.wasHold()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        // IDLE on its own. Sleeping is unreliable enough that running it first would keep this
        // from being reached at all, and this is the mode worth reaching for
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            selection_across(detected, LowPower::Idle);
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
