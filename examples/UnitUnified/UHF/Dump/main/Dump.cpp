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

//! @brief Name the XTID segments a header says are there
std::string xtid_segments(const uint16_t header)
{
    std::string s{};
    if (header & m5::uhf::XTID_OPTIONAL_COMMAND_SUPPORT) {
        s += "cmdsupport ";
    }
    if (header & m5::uhf::XTID_BLOCKWRITE_BLOCKERASE) {
        s += "blockwrite ";
    }
    if (header & m5::uhf::XTID_USER_MEMORY_BLOCKPERMALOCK) {
        s += "usermem ";
    }
    if (header & m5::uhf::XTID_LOCK_BIT) {
        s += "lockbit ";
    }
    return s.empty() ? "none" : s;
}

//! @brief Say how big something is, or that nobody said
std::string bits_or_unknown(const uint32_t bits)
{
    return bits ? m5::utility::formatString("%ubit", bits) : std::string{"unknown"};
}

/*!
  @brief Say how much user memory there is
  @details A size of zero means nothing said how much, which is not the same as there being
  none. The PC carries an indicator that settles that one way, so a tag stating it has no user
  memory reads as none rather than as unknown
 */
std::string user_memory_size(const m5::uhf::Tag& tag)
{
    if (tag.user_memory_bits == 0 && !m5::uhf::pcUserMemoryIndicator(tag.pc)) {
        return "none";
    }
    return bits_or_unknown(tag.user_memory_bits);
}

void print_identity(const m5::uhf::Tag& tag)
{
    M5_LOGI("TID   : %s", tag.tid.toString().c_str());
    M5_LOGI("Vendor: %s (MDID 0x%03X)", tag.vendorAsString().c_str(), tag.mdid);
    M5_LOGI("Chip  : %s (TMN 0x%03X)", tag.chipAsString().c_str(), tag.model_number);

    // "XTID: yes" only says the indicator bit is set. What it carries is another matter, and on
    // every chip tried so far it carries a serial number and nothing else, which is why the
    // sizes below come from the chip rather than from the tag
    if (tag.has_xtid && tag.tid.size >= 6) {
        const uint16_t header = static_cast<uint16_t>((tag.tid[4] << 8) | tag.tid[5]);
        M5_LOGI("XTID  : header=0x%04X serial=%ubit segments=%s", header, tag.serial_bits,
                xtid_segments(header).c_str());
    } else {
        M5_LOGI("XTID  : no");
    }
    M5_LOGI("Flags : security=%u file=%u blockPermalock=%s", tag.supports_security ? 1 : 0, tag.supports_file ? 1 : 0,
            tag.supports_block_permalock ? "yes" : "no");
    M5_LOGI("Sizes : user=%s maxEPC=%s permalockBlock=%s", user_memory_size(tag).c_str(),
            bits_or_unknown(tag.epc_max_bits).c_str(), bits_or_unknown(tag.permalock_block_bits).c_str());

    lcd.printf("%s / %s\n", tag.vendorAsString().c_str(), tag.chipAsString().c_str());
    lcd.printf("user %s\n", user_memory_size(tag).c_str());
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
    print_identity(tag);
    uhf.dump();
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
