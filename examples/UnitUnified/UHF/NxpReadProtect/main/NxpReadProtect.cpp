/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Turn the read protection of an NXP UCODE G2iM on and off

  A protected tag does not fall silent. It goes on answering an inventory round; what changes
  is that its EPC and its TID read back as zeroes. That is what makes it recoverable, and also
  what makes it awkward: every protected tag reads back the same all-zero EPC, so two of them
  in the same field cannot be told apart and neither can be addressed on purpose.

  Because of that this works with one tag in the field and says so if it finds more.

  The protection is reached by inverting the two bits of the Config-Word that carry it rather
  than through the ReadProtect command that exists for it. The chip's own datasheet offers that
  as the alternative, and it is the better one to show: ReadProtect answers every kind of
  failure with the same silence, while inverting a bit hands back the word the tag ended up
  with. It also lets the two bits be reached one at a time.

  Either way the tag has to be in the secured state, which it will not enter while its access
  password is zero, and that is how they leave the factory. So one is written first and cleared
  again at the end, which is why this run has more steps than it looks as though it should.

  What the protection hides is hidden from a reader that does not hold the password. One that
  does goes on seeing everything, which is the point of it, so the reading that shows the
  difference is done without the password even though the password is what set it.

  What it hides of the TID is the serial number. The two words that name the mask designer and
  the model are left readable, so a protected tag can still be recognised as the kind of thing
  it is while no longer being telling apart from others of its kind.

  Clicking reports whether the tag is protected and changes nothing. Holding gives the tag a
  password, protects it, reads the EPC and the TID back without the password to show what a
  protected tag gives up to anyone else, puts it back and clears the password.

  Protecting also hides the very bytes an EPC mask matches on, so the selection is dropped each
  time and the tag is found again by whatever it answers with now.

  @warning The bits are those of the UCODE G2iM and G2iM+. Other NXP chips lay their
  Config-Word out differently
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

//! @brief Say whether the tag is protected, from the word it holds
void report(const uint16_t word)
{
    const auto cfg = m5::uhf::decodeNxpConfigWord(word);
    M5_LOGI("Config-Word %04X: EPC %s / TID %s / user %s", word, cfg.protect_epc ? "protected" : "open",
            cfg.protect_tid ? "protected" : "open", cfg.protect_user ? "protected" : "open");
    lcd.printf("EPC %s\nTID %s\n", cfg.protect_epc ? "prot" : "open", cfg.protect_tid ? "prot" : "open");
}

//! @brief Spell out bytes, since what matters here is whether they came back as zeroes
std::string as_hex(const std::vector<uint8_t>& bytes)
{
    std::string out{};
    char pair[3]{};
    for (const uint8_t b : bytes) {
        snprintf(pair, sizeof(pair), "%02X", b);
        out += pair;
    }
    return out;
}

//! @brief Show what the tag gives up of itself
void show_what_reads(const m5::uhf::Tag& tag)
{
    std::vector<uint8_t> epc{};
    if (uhf.readBank(epc, m5::uhf::Bank::Epc, 2, m5::uhf::pcEPCLengthWords(tag.pc))) {
        M5_LOGI("EPC reads back: %s", as_hex(epc).c_str());
    } else {
        M5_LOGI("EPC could not be read at all");
    }
    // The whole of it: what the protection hides is the serial number, and the two words that
    // name the chip are left readable so that it can still be recognised
    std::vector<uint8_t> tid{};
    if (uhf.readBank(tid, m5::uhf::Bank::Tid, 0, 6)) {
        M5_LOGI("TID reads back: %s", as_hex(tid).c_str());
    } else {
        M5_LOGI("TID could not be read at all");
    }
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
    // Which chip this is comes from the TID, and what a chip is decides whether it can have
    // the command at all. Asking a chip that cannot costs a round trip and comes back saying
    // nothing answered, which is not the same as being told it was never there to answer
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

//! @brief Report the state and change nothing
void show()
{
    m5::uhf::Tag tag{};
    if (!detect_one(tag)) {
        return;
    }
    uint16_t word{};
    if (look(tag, word)) {
        show_what_reads(tag);
        uhf.deselect();
    }
}

//! @brief Store an access password and address the tag with it
//! @details The value is read back, because a write the tag answered is not the same as a
//! write that landed, and the whole run turns on the tag holding this exact value
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

//! @brief Find the one tag again and address it with whichever password it is holding
//! @details A run that stopped part way may have left the tag with the password or without it,
//! and addressing it with the wrong one is refused, so both are tried
bool find_and_address(m5::uhf::Tag& tag)
{
    if (!detect_one(tag)) {
        return false;
    }
    return uhf.select(tag, ACCESS_PASSWORD) || uhf.select(tag, 0);
}

//! @brief Put the tag back the way it was found, as far as anything here can
//! @details Called after a run that may have stopped anywhere, so it assumes nothing about
//! what the tag is showing or which password the selection is carrying
void restore_tag()
{
    m5::uhf::Tag tag{};
    if (!find_and_address(tag)) {
        M5_LOGE("Could not address the tag to put it back; hold the button again");
        lcd.println("NOT restored");
        return;
    }
    uint16_t word{};
    if (uhf.readNxpConfigWord(word)) {
        // Sending the same bits again inverts them back, so only the ones that are set are sent
        const auto cfg = m5::uhf::decodeNxpConfigWord(word);
        uint16_t undo  = 0;
        if (cfg.protect_epc) {
            undo |= m5::uhf::NXP_CONFIG_PROTECT_EPC;
        }
        if (cfg.protect_tid) {
            undo |= m5::uhf::NXP_CONFIG_PROTECT_TID;
        }
        if (undo != 0) {
            const auto undone = uhf.toggleNxpConfigWord(word, undo);
            if (!undone) {
                M5_LOGE("THE TAG IS STILL PROTECTED (%s); hold the button again",
                        m5::uhf::reasonAsString(undone.error()));
                lcd.println("still protected");
                return;
            }
            // The protection is off again; only getting hold of the tag afterwards failed
            if (!find_and_address(tag)) {
                M5_LOGE("The protection is off, but the tag could not be addressed to clear the password");
                lcd.println("password left");
                return;
            }
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

//! @brief Give the tag a password, protect it, show what it gives up, and put it all back
void protect_and_release()
{
    m5::uhf::Tag tag{};
    if (!detect_one(tag)) {
        return;
    }
    uint16_t word{};
    if (!look(tag, word)) {
        return;
    }
    if (m5::uhf::decodeNxpConfigWord(word).protect_epc) {
        M5_LOGI("The tag is protected already, so this run only puts it back");
        uhf.deselect();
        restore_tag();
        return;
    }

    // Without one the command is ignored and nothing at all happens
    if (!set_access_password(tag, ACCESS_PASSWORD)) {
        lcd.println("password: failed");
        uhf.deselect();
        restore_tag();
        return;
    }
    // Inverting both at once, which is what ReadProtect would have done to them
    constexpr uint16_t BOTH{m5::uhf::NXP_CONFIG_PROTECT_EPC | m5::uhf::NXP_CONFIG_PROTECT_TID};
    uint16_t now{};
    const auto protect = uhf.toggleNxpConfigWord(now, BOTH);
    if (!protect) {
        M5_LOGE("Failed to protect the tag: %s", m5::uhf::reasonAsString(protect.error()));
        lcd.println("protect: failed");
        restore_tag();
        return;
    }
    M5_LOGI("Now: %04X", now);
    if (!m5::uhf::decodeNxpConfigWord(now).protect_epc) {
        M5_LOGE("The tag answered but kept the word it had; it was not in the secured state");
        lcd.println("not secured");
        restore_tag();
        return;
    }

    // The EPC it answers with may have changed, so it is looked for rather than selected again,
    // and addressed without the password: holding the password is what the protection lets past
    m5::uhf::Tag hidden{};
    if (!detect_one(hidden) || !uhf.select(hidden, 0)) {
        M5_LOGE("The tag did not answer once protected");
        lcd.println("lost it");
        restore_tag();
        return;
    }
    uint16_t hidden_word{};
    if (uhf.readNxpConfigWord(hidden_word)) {
        report(hidden_word);
    }
    M5_LOGI("Without the password:");
    show_what_reads(hidden);

    // And again with it, which is what the password is for
    if (find_and_address(hidden)) {
        M5_LOGI("With the password:");
        show_what_reads(hidden);
    }

    restore_tag();
    lcd.println("back as found");
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: is it protected");
    lcd.println("A hold: protect, then not");
    lcd.println("one tag in the field");
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
        protect_and_release();
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
