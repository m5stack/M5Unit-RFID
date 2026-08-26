/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Lock and unlock a memory bank of a UHF-RFID tag with Unit UHF-RFID (U107)

  Everything here is reversible. The User bank is locked and opened again, the word written to
  see what that changes is put back, and the access password the tag is given for the length of
  the run is taken away at the end. A permanent lock is asked for once to show that it is
  refused unless it is named.

  The tag is identified before anything is locked, since a chip without a User bank has nothing
  here to lock and would refuse every operation.

  The password is what makes the lock visible. A lock says who may write, not whether anyone
  may, and a tag whose access password is zero puts every reader in the secured state, so a
  locked bank stays writeable (Gen2 v2.1 6.3.2.1.1.2). Given a password, the same bank refuses
  a reader that does not know it.

  A run that stops part way can leave the User bank locked, the access password set, or both.
  Holding the button undoes whichever of them is there, and works whatever the tag is holding,
  since a write goes through from the open state and so does not need the current password.
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

//! @brief The access password the tag holds while the lock is being shown, and only then. A tag
//! leaves the factory with zero, which is what would make the lock below look like it does nothing
constexpr uint32_t ACCESS_PASSWORD{0xA5A55A5A};
//! @brief Where the access password lives, as a word address in the Reserved bank (Gen2 v2.1 6.3.2.1.1.2)
constexpr uint16_t ACCESS_PASSWORD_WORD{2};
//! @brief Length of the access password in words
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

    // Writing a tag takes more power than reading one, so a module left at a low setting reads
    // tags and then refuses every write. What the module holds is reported, not changed: how
    // much power may be radiated is a question of where the unit is being used
    int16_t dbm100{};
    if (unit.readTransmitPower(dbm100)) {
        M5_LOGI("Transmit power: %d.%02d dBm", dbm100 / 100, dbm100 % 100);
    }
}

//! @brief The one tag in the field, or none. Locking the wrong tag is worth avoiding
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

//! @brief Name the selected tag and say whether it has a User bank to lock at all
bool has_user_bank(m5::uhf::Tag& tag)
{
    if (!uhf.identify(tag)) {
        M5_LOGE("Failed to read the TID");
        lcd.println("identify: failed");
        return false;
    }
    M5_LOGI("Chip: %s, User memory %u bit", tag.chipAsString().c_str(), tag.user_memory_bits);
    lcd.printf("%s\n", tag.chipAsString().c_str());
    if (m5::uhf::chipHasNoUserMemory(tag.chip)) {
        M5_LOGW("This chip has no User bank; there is nothing here to lock");
        lcd.println("no User bank");
        return false;
    }
    return true;
}

//! @brief Write one word and put it back, to see whether the lock stops it
//! @details Putting the word back is tried more than once. A probe that fails costs nothing,
//! while a word left overwritten is a change to the tag that was never asked for, and this
//! module drops a write now and then even when the tag is well within range
void write_probe(const char* when)
{
    constexpr uint8_t RESTORE_ATTEMPTS{4};
    constexpr uint32_t RESTORE_INTERVAL_MS{20};

    const std::vector<uint8_t> pattern{0x5A, 0xA5};
    const bool wrote = uhf.writeBank(m5::uhf::Bank::User, 0, pattern.data(), static_cast<uint16_t>(pattern.size()));
    M5_LOGI("Writing one word %s: %s", when, wrote ? "allowed" : "refused");
    lcd.printf("write %s: %s\n", when, wrote ? "ok" : "no");
    if (!wrote) {
        return;
    }
    const std::vector<uint8_t> zero{0x00, 0x00};
    for (uint8_t i = 0; i < RESTORE_ATTEMPTS; ++i) {
        if (uhf.writeBank(m5::uhf::Bank::User, 0, zero.data(), static_cast<uint16_t>(zero.size()))) {
            return;
        }
        m5::utility::delay(RESTORE_INTERVAL_MS);
    }
    M5_LOGE("FAILED TO RESTORE the word; the tag still holds 5AA5");
}

//! @brief Address the tag again with a password, since that is what decides the state it is in
bool address_with(const m5::uhf::Tag& tag, const uint32_t password)
{
    // The selection is dropped first rather than replaced. A command that follows a lock has
    // been seen to ride the state the lock left behind, which is not the state being asked for
    uhf.deselect();
    if (!uhf.select(tag, password)) {
        M5_LOGE("Failed to select the tag with password %08X", password);
        lcd.println("select: failed");
        return false;
    }
    return true;
}

//! @brief Store an access password, which is what lets the tag tell one reader from another
//! @details The value is read back, because a write the tag answered is not the same as a write
//! that landed, and every step below turns on the tag holding this exact value. The tag is
//! addressed again in between, since the write changed the very password the selection carries
bool set_access_password(const m5::uhf::Tag& tag, const uint32_t password)
{
    const std::vector<uint8_t> data{static_cast<uint8_t>(password >> 24), static_cast<uint8_t>(password >> 16),
                                    static_cast<uint8_t>(password >> 8), static_cast<uint8_t>(password)};
    if (!uhf.writeBank(m5::uhf::Bank::Reserved, ACCESS_PASSWORD_WORD, data.data(),
                       static_cast<uint16_t>(data.size()))) {
        M5_LOGE("Failed to store the access password %08X", password);
        lcd.println("password: failed");
        return false;
    }
    if (!address_with(tag, password)) {
        return false;
    }
    std::vector<uint8_t> back{};
    if (!uhf.readBank(back, m5::uhf::Bank::Reserved, ACCESS_PASSWORD_WORD, ACCESS_PASSWORD_WORDS)) {
        M5_LOGE("Failed to read the access password back");
        lcd.println("password: unread");
        return false;
    }
    if (back != data) {
        if (back.size() == data.size()) {
            M5_LOGE("The tag holds %02X%02X%02X%02X, not %08X", back[0], back[1], back[2], back[3], password);
        } else {
            M5_LOGE("Read %zu bytes of the access password back, not %zu", back.size(), data.size());
        }
        lcd.println("password: wrong");
        return false;
    }
    M5_LOGI("The tag holds the access password %08X", password);
    return true;
}

//! @brief Set one bank to one lock state and say whether the tag carried it out
bool set_lock(const m5::uhf::LockTarget target, const m5::uhf::LockAction action, const char* what)
{
    const std::vector<m5::uhf::LockSetting> settings{m5::uhf::LockSetting(target, action)};
    const bool ok = uhf.lock(settings);
    M5_LOGI("%s: %s", what, ok ? "the tag carried it out" : "failed");
    lcd.printf("%s: %s\n", what, ok ? "ok" : "NG");
    return ok;
}

void lock_and_open(m5::uhf::Tag& tag)
{
    if (!address_with(tag, 0)) {
        return;
    }
    if (!has_user_bank(tag)) {
        uhf.deselect();
        return;
    }

    // A permanent lock cannot be undone, so it is refused unless the caller says the word. This
    // one never reaches the tag
    const std::vector<m5::uhf::LockSetting> permanent{
        m5::uhf::LockSetting(m5::uhf::LockTarget::User, m5::uhf::LockAction::PermanentLock)};
    M5_LOGI("Asking for a permanent lock without allow_permanent: %s",
            uhf.lock(permanent) ? "accepted, which it should not be" : "refused, as it should be");

    write_probe("before locking");

    if (!set_access_password(tag, ACCESS_PASSWORD)) {
        // The write may have landed and only the reading back have failed, which leaves the tag
        // holding a password nobody asked it to keep
        M5_LOGE("THE TAG MAY HOLD THE ACCESS PASSWORD %08X; hold the button to put it back", ACCESS_PASSWORD);
        uhf.deselect();
        return;
    }

    if (set_lock(m5::uhf::LockTarget::User, m5::uhf::LockAction::Lock, "lock User")) {
        // Addressed without the password the tag is in the open state, which is where a locked
        // bank refuses to be written. This is the lock doing the thing it is for
        if (address_with(tag, 0)) {
            write_probe("while locked, without the password");
        }
        if (address_with(tag, ACCESS_PASSWORD)) {
            write_probe("while locked, with the password");
            if (!set_lock(m5::uhf::LockTarget::User, m5::uhf::LockAction::Open, "open User")) {
                M5_LOGE("THE USER BANK MAY STILL BE LOCKED; hold the button to put it back");
            }
        }
    }

    // Put the tag back the way it was found. The password goes last, since clearing it is what
    // the steps above needed it for
    if (set_access_password(tag, 0)) {
        write_probe("after opening");
    } else {
        M5_LOGE("THE TAG STILL HOLDS THE ACCESS PASSWORD %08X; hold the button to put it back", ACCESS_PASSWORD);
    }
    uhf.deselect();
}
//! @brief Undo both of the things a run can leave behind: a lock and a password
//! @details A run that stops part way can leave the User bank locked, the access password set,
//! or both, and a run whose lock fails never reaches the step that opens the bank again. The
//! bank is opened before the password goes, since opening it is what the password is for
//! @note Zero is tried first because it costs nothing when it is wrong: a reader with no
//! password to send does not send one, so there is no failed access to knock the tag out of
//! the round. Offering the wrong password does, which is why it is only the second try
void restore_tag(const m5::uhf::Tag& detected)
{
    if (!address_with(detected, 0) && !address_with(detected, ACCESS_PASSWORD)) {
        return;
    }
    set_lock(m5::uhf::LockTarget::User, m5::uhf::LockAction::Open, "open User");
    if (set_access_password(detected, 0)) {
        lcd.println("password: cleared");
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
    lcd.println("A: lock and open");
    lcd.println("A hold: put it back");
    lcd.println("(nothing is kept)");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            lock_and_open(detected);
        }
    }
    if (M5.BtnA.wasHold()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            restore_tag(detected);
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
