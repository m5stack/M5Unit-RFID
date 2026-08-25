/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Write to the User memory of a UHF-RFID tag and put it back
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include <algorithm>
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

/*!
  @brief Detect and hand back the one tag in the field
  @param[out] tag The tag that was found
  @return True when exactly one tag answered
  @details Writing to a tag has to know which tag, so this asks for the whole field rather than
  the first answer and refuses when there is more than one in it. UHFLayer::detect also has a
  form that hands back whichever tag answers first, which suits reading but not writing
 */
bool detect_one(m5::uhf::Tag& tag)
{
    std::vector<m5::uhf::Tag> tags{};
    if (!uhf.detect(tags)) {
        M5_LOGI("No tag in the field");
        lcd.println("no tag");
        return false;
    }
    if (tags.size() != 1) {
        M5_LOGI("%u tags in the field; leave one of them", (unsigned)tags.size());
        lcd.printf("%u tags: leave one\n", (unsigned)tags.size());
        return false;
    }
    tag = tags[0];
    return true;
}

std::string to_hex(const std::vector<uint8_t>& data)
{
    std::string s{};
    for (auto&& b : data) {
        s += m5::utility::formatString("%02X", b);
    }
    return s;
}

//! @brief Words the short write test replaces and then puts back
constexpr uint16_t WRITE_TEST_WORDS{2};
/*!
  @brief The most a single Write command carries
  @details The module programs the words into the tag one at a time and waits out the tag's
  write time for each, so a write of this many words is the longest a single command
  legitimately takes. Anything longer is split by writeBank into several
 */
constexpr uint16_t WRITE_TEST_MAX_WORDS{32};

/*!
  Write a stretch of User memory, read it back and put the original contents back.

  User memory is the only bank this touches. Rewriting EPC would change what the tag answers to
  in inventory, and writing Reserved would set the access and kill passwords, which is how a
  development tag stops being usable for development. Neither is worth it to prove a write works.
 */
void write_roundtrip(const m5::uhf::Tag& detected, const uint16_t words)
{
    if (!uhf.select(detected)) {
        M5_LOGE("write: failed to select the tag");
        return;
    }

    // A tag whose passwords have been set can refuse the write halfway, which would leave the
    // original contents already gone. Checking first keeps that from happening quietly
    std::vector<uint8_t> reserved{};
    if (!uhf.readBank(reserved, m5::uhf::Bank::Reserved, 0, 4)) {
        M5_LOGE("write: Reserved could not be read, so the passwords are unknown");
        uhf.deselect();
        return;
    }
    for (auto&& b : reserved) {
        if (b != 0x00) {
            M5_LOGW("write: this tag has a password set (%s); leaving it alone", to_hex(reserved).c_str());
            uhf.deselect();
            return;
        }
    }

    const unsigned long read_began = m5::utility::millis();
    std::vector<uint8_t> original{};
    if (!uhf.readBank(original, m5::uhf::Bank::User, 0, words)) {
        // The caller sized this from what the tag holds, so a refusal here is not the bank being
        // too small; it is the read itself failing
        M5_LOGE("write %u words: could not read what is there now, so nothing is written", words);
        uhf.deselect();
        return;
    }
    const unsigned long read_took = m5::utility::millis() - read_began;

    // A pattern that says where in the bank each byte came from, so a mismatch points at the
    // word it happened in rather than just failing
    std::vector<uint8_t> pattern(static_cast<size_t>(words) * 2);
    for (size_t i = 0; i < pattern.size(); ++i) {
        pattern[i] = static_cast<uint8_t>(0xA0 + (i & 0x0F));
    }

    const unsigned long began = m5::utility::millis();
    if (!uhf.writeBank(m5::uhf::Bank::User, 0, pattern)) {
        M5_LOGE("write %u words: failed", words);
        uhf.deselect();
        return;
    }
    const unsigned long took = m5::utility::millis() - began;

    std::vector<uint8_t> readback{};
    if (!uhf.readBank(readback, m5::uhf::Bank::User, 0, words)) {
        M5_LOGE("write %u words: wrote but could not read back, so the tag now holds the pattern", words);
        uhf.deselect();
        return;
    }
    M5_LOGI("write %2u words: read %lums, write %lums, read back %s", words, read_took, took,
            readback == pattern ? "matches" : "MISMATCH");
    lcd.printf("%2u words %lums %s\n", words, took, readback == pattern ? "ok" : "NG");
    uhf.dump(m5::uhf::Bank::User, 0, words);

    // Put it back the way it was, and say so loudly if that does not work: the tag is left
    // holding the pattern in that case
    if (!uhf.writeBank(m5::uhf::Bank::User, 0, original)) {
        M5_LOGE("write %u words: FAILED TO RESTORE; the tag still holds the pattern", words);
        uhf.deselect();
        return;
    }
    std::vector<uint8_t> restored{};
    if (uhf.readBank(restored, m5::uhf::Bank::User, 0, words)) {
        M5_LOGI("write %2u words: restored %s", words, restored == original ? "matches" : "MISMATCH");
    }
    uhf.dump(m5::uhf::Bank::User, 0, words);
    uhf.deselect();
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: write User memory");
    lcd.println("(restores it afterwards)");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        m5::uhf::Tag detected{};
        if (!detect_one(detected)) {
            return;
        }
        // How much user memory there is decides what is worth writing, and only the TID says
        // which chip this is
        if (!uhf.select(detected)) {
            M5_LOGE("Failed to select the tag");
            lcd.println("select: failed");
            return;
        }
        m5::uhf::Tag tag{};
        const bool identified = uhf.identify(tag);
        uhf.deselect();
        if (!identified) {
            M5_LOGE("Failed to identify the tag");
            lcd.println("identify: failed");
            return;
        }

        const uint16_t bank = static_cast<uint16_t>(tag.user_memory_bits / 16);
        M5_LOGI("%s: user memory %s", tag.chipAsString().c_str(),
                bank ? m5::utility::formatString("%u words", bank).c_str()
                     : (m5::uhf::chipHasNoUserMemory(tag.chip) ? "none" : "size not known"));
        lcd.printf("%s\n", tag.chipAsString().c_str());

        if (bank == 0 && m5::uhf::chipHasNoUserMemory(tag.chip)) {
            M5_LOGI("Nothing to write to on this tag");
            lcd.println("no user memory");
            return;
        }

        // Two words to see it work at all
        write_roundtrip(tag, WRITE_TEST_WORDS);
        // Then the most a single command carries, or the whole bank if that is smaller
        write_roundtrip(tag, bank ? std::min<uint16_t>(WRITE_TEST_MAX_WORDS, bank) : WRITE_TEST_MAX_WORDS);
        // Then the whole bank, which only tells us anything when it takes more than one command
        if (bank > WRITE_TEST_MAX_WORDS) {
            write_roundtrip(tag, bank);
        } else if (bank) {
            M5_LOGI("The whole bank fits in one command, so there is no split to exercise");
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
