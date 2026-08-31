/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Read and change the Config-Word of an NXP UCODE G2iM, and its article surveillance flag

  The UCODE G2X family keeps a sixteen-bit Config-Word beside the EPC. It holds what the chip
  is doing that Gen2 has no word for: how loudly it answers, whether it drives its output pad,
  which of its banks read back as zeroes, and a Product Status Flag that an article surveillance
  gate can ask about without singulating anything.

  Changing it is not a write. The ChangeConfig command inverts the bits a mask names and leaves
  the rest alone, so sending the same mask twice puts the word back where it was. That is why
  nothing here is called a write, and why a run can undo itself exactly.

  Reading the word costs nothing. Changing anything does not: a tag answers ChangeEAS only from
  the secured state, and ignores it outright while its access password is zero, which is how it
  leaves the factory. So one is written first and cleared again at the end, and that is why this
  run has more steps than it looks as though it should.

  Clicking reads the word and spells it out, and asks the field whether anything is flagged.
  Nothing is changed. Holding gives the tag a password, raises the Product Status Flag, asks
  again so that the difference can be heard, lowers it and clears the password.

  @warning The bits are those of the UCODE G2iM and G2iM+. Other NXP chips keep a Config-Word
  too and lay it out differently, so what is printed here would be untrue of them
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

//! @brief Password the tag is given so that the command is not ignored. Cleared again at the end
constexpr uint32_t ACCESS_PASSWORD{0xA5A55A5A};
//! @brief Where the access password lives, as a word address in the Reserved bank
constexpr uint16_t ACCESS_PASSWORD_WORD{2};
//! @brief Its length in words
constexpr uint16_t ACCESS_PASSWORD_WORDS{2};

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

//! @brief Say what the Config-Word holds
void report(const uint16_t word)
{
    const auto cfg = m5::uhf::decodeNxpConfigWord(word);
    M5_LOGI("Config-Word: %04X", word);
    M5_LOGI("  PSF alarm      : %s", cfg.psf_alarm ? "raised" : "down");
    M5_LOGI("  Read protection: EPC %s / TID %s / user %s", cfg.protect_epc ? "on" : "off",
            cfg.protect_tid ? "on" : "off", cfg.protect_user ? "on" : "off");
    M5_LOGI("  Backscatter    : %s", cfg.max_backscatter ? "as loud as it can" : "reduced");
    M5_LOGI("  Range          : %s%s", cfg.range_reduction ? "reduced" : "full",
            cfg.conditional_range ? ", and reduced again when the condition holds" : "");
    M5_LOGI("  Output pad     : %s%s", cfg.digital_output ? "driven" : "idle", cfg.invert_output ? ", inverted" : "");
    M5_LOGI("  Chip says      : tamper %s, external supply %s", cfg.tamper_alarm ? "broken" : "intact",
            cfg.external_supply ? "present" : "absent");
    lcd.printf("cfg %04X\nPSF %s\n", word, cfg.psf_alarm ? "up" : "down");
}

//! @brief Ask the field whether anything is flagged, and say so
void listen_for_the_alarm(const char* when)
{
    std::vector<uint8_t> alarm{};
    if (!uhf.nxpEASAlarm(alarm)) {
        M5_LOGE("The alarm could not be put to the field at all");
        lcd.println("alarm: failed");
        return;
    }
    if (alarm.empty()) {
        M5_LOGI("The alarm %s: nothing answered", when);
        lcd.printf("alarm %s: no\n", when);
        return;
    }
    std::string code{};
    char pair[3]{};
    for (const uint8_t b : alarm) {
        snprintf(pair, sizeof(pair), "%02X", b);
        code += pair;
    }
    M5_LOGI("The alarm %s: %s", when, code.c_str());
    lcd.printf("alarm %s: %s\n", when, code.c_str());
}

//! @brief Select the tag and read its Config-Word
//! @param[in,out] tag Tag to work on
//! @param[out] word Word the tag holds
//! @return True when it was read
bool look(m5::uhf::Tag& tag, uint16_t& word)
{
    if (!uhf.select(tag)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return false;
    }
    if (uhf.identify(tag)) {
        M5_LOGI("Chip: %s", tag.chipAsString().c_str());
    }
    const auto read = uhf.readNxpConfigWord(word);
    if (!read) {
        // A chip that never had the command is a different answer from one that did not reply
        M5_LOGE("The Config-Word could not be read: %s", m5::uhf::reasonAsString(read.error()));
        lcd.println("no Config-Word");
        uhf.deselect();
        return false;
    }
    report(word);
    return true;
}

//! @brief Read the word and listen for the alarm, changing nothing
void show()
{
    m5::uhf::Tag tag{};
    if (!detect_one(tag)) {
        return;
    }
    uint16_t word{};
    if (!look(tag, word)) {
        return;
    }
    uhf.deselect();
    // The alarm asks the field rather than one tag, so the selection is dropped first to make
    // the point that none is needed
    listen_for_the_alarm("now");
}

//! @brief Store an access password and address the tag with it
//! @details The value is read back, because a write the tag answered is not the same as a
//! write that landed, and the step below turns on the tag holding this exact value
bool set_access_password(const m5::uhf::Tag& tag, const uint32_t password)
{
    const uint8_t data[]{static_cast<uint8_t>(password >> 24), static_cast<uint8_t>(password >> 16),
                         static_cast<uint8_t>(password >> 8), static_cast<uint8_t>(password)};
    const auto stored = uhf.writeBank(m5::uhf::Bank::Reserved, ACCESS_PASSWORD_WORD, data, sizeof(data));
    if (!stored) {
        M5_LOGE("Failed to store the access password %08X: %s", password, m5::uhf::reasonAsString(stored.error()));
        return false;
    }
    // The write changed the very password the selection carries, so the tag is addressed again
    if (!uhf.select(tag, password)) {
        M5_LOGE("Failed to address the tag with %08X", password);
        return false;
    }
    std::vector<uint8_t> back{};
    if (!uhf.readBank(back, m5::uhf::Bank::Reserved, ACCESS_PASSWORD_WORD, ACCESS_PASSWORD_WORDS) || back.size() != 4) {
        M5_LOGE("Failed to read the access password back");
        return false;
    }
    const uint32_t read = (static_cast<uint32_t>(back[0]) << 24) | (static_cast<uint32_t>(back[1]) << 16) |
                          (static_cast<uint32_t>(back[2]) << 8) | back[3];
    if (read != password) {
        M5_LOGE("The tag holds %08X, not %08X", read, password);
        return false;
    }
    M5_LOGI("Access password: %08X", password);
    return true;
}

//! @brief Put the tag back the way it was found, as far as anything here can
void restore_tag(const m5::uhf::Tag& tag, const bool was_raised)
{
    if (!uhf.select(tag, ACCESS_PASSWORD) && !uhf.select(tag, 0)) {
        M5_LOGE("Could not address the tag to put it back; hold the button again");
        lcd.println("NOT restored");
        return;
    }
    uint16_t word{};
    if (uhf.readNxpConfigWord(word) && m5::uhf::decodeNxpConfigWord(word).psf_alarm != was_raised) {
        const auto put_back = uhf.writeNxpEAS(was_raised);
        if (!put_back) {
            M5_LOGE("THE FLAG IS STILL %s (%s); hold the button again", was_raised ? "DOWN" : "UP",
                    m5::uhf::reasonAsString(put_back.error()));
            lcd.println("flag left");
        }
    }
    if (set_access_password(tag, 0)) {
        M5_LOGI("Access password: cleared");
    } else {
        M5_LOGE("THE TAG MAY STILL HOLD %08X; hold the button again", ACCESS_PASSWORD);
        lcd.println("password left");
    }
    uhf.deselect();
}

//! @brief Raise the Product Status Flag, listen, and lower it again
void flag_and_unflag()
{
    m5::uhf::Tag tag{};
    if (!detect_one(tag)) {
        return;
    }
    uint16_t word{};
    if (!look(tag, word)) {
        return;
    }
    const bool was_raised = m5::uhf::decodeNxpConfigWord(word).psf_alarm;

    // Without one the command is ignored and nothing at all happens, and nothing says so
    if (!set_access_password(tag, ACCESS_PASSWORD)) {
        lcd.println("password: failed");
        restore_tag(tag, was_raised);
        return;
    }
    const auto changed = uhf.writeNxpEAS(!was_raised);
    if (!changed) {
        M5_LOGE("Failed to change the flag: %s", m5::uhf::reasonAsString(changed.error()));
        lcd.println("EAS: failed");
        restore_tag(tag, was_raised);
        return;
    }
    uint16_t now{};
    if (uhf.readNxpConfigWord(now)) {
        M5_LOGI("Now: %04X", now);
    }
    uhf.deselect();
    listen_for_the_alarm(was_raised ? "with it lowered" : "with it raised");

    restore_tag(tag, was_raised);
    lcd.println("back as found");
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: read the word");
    lcd.println("A hold: flag and unflag");
    lcd.println("(NXP UCODE G2X only)");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        show();
    }
    if (M5.BtnA.wasHold()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        flag_and_unflag();
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
