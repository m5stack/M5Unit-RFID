/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Read every memory bank of a UHF-RFID tag with Unit UHF-RFID (U107)
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

std::string to_hex(const std::vector<uint8_t>& data)
{
    std::string s{};
    for (auto&& b : data) {
        s += m5::utility::formatString("%02X", b);
    }
    return s;
}

//! @brief Words to read when the tag holds user memory but never says how much
constexpr uint16_t USER_PROBE_WORDS{2};
//! @brief Enough of the User bank to see what is in it without filling the log with it
constexpr uint16_t USER_DUMP_MAX_WORDS{16};

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
    // Zero means the tag never said, not that the tag has none of it. Every chip tried so far
    // carries an XTID with nothing in it but a serial number, so these stay zero in practice
    M5_LOGI("Sizes : user=%ubit maxEPC=%ubit permalockBlock=%ubit blockPermalock=%s", tag.user_memory_bits,
            tag.epc_max_bits, tag.permalock_block_bits, tag.supports_block_permalock ? "yes" : "no");

    lcd.printf("%s / %s\n", tag.vendorAsString().c_str(), tag.chipAsString().c_str());

    // Kill password then access password. All zeros is the state a tag leaves the factory in,
    // and it is also what keeps the tag from ever being killed by accident
    dump_bank("Reserved", m5::uhf::Bank::Reserved, 0, 4);

    // The XTID says how big the User bank is when it says anything at all. When it stays silent
    // the PC still carries an indicator, so a short probe read is worth trying before giving up
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
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: identify and dump");
    lcd.println("one tag in the field");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        // Whichever tag answers first. Reading a bank of the wrong one costs nothing, so
        // this does not insist on there being only one the way writing to a tag would
        m5::uhf::Tag tag{};
        if (uhf.detect(tag)) {
            identify_and_dump(tag);
        } else {
            M5_LOGI("No tag in the field");
            lcd.println("no tag");
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
