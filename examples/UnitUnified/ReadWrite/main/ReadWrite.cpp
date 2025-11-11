/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Read/Write example
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <vector>

using namespace m5::nfc::a;
using namespace m5::nfc::a::mifare;
using namespace m5::nfc::a::mifare::classic;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitRFID2 unit;
m5::unit::nfc::NFCLayerA nfc_a{unit};

// KeyA that can authenticate all blocks
// If it's a different key value, change it
constexpr Key keyA = DEFAULT_KEY;  // Default as 0xFFFFFFFFFFFF

constexpr char long_msg[] =
    "This is a sample message buffer used for testing NFC page writes and data integrity verification purposes.";
constexpr char short_msg[] = "0123456789ABCDEFGHIJ";

void read_all_user_area(const Key& key)
{
    static uint8_t buf[4096]{};
    uint16_t rx_len{4096};
    memset(buf, 0x52, sizeof(buf));

    if (nfc_a.read(buf, rx_len, 0, key)) {
        M5.Log.printf("User area %u\n", rx_len);
        M5.Log.printf("--------------------------------\n");
        m5::utility::log::dump(buf, rx_len, false);
        M5.Log.printf("--------------------------------\n");
    } else {
        M5_LOGE("Failed to read");
    }
}

// Using read/write for all
bool read_write(const uint8_t sblock, const char* msg, const Key& key)
{
    auto len = strlen(msg);
    uint8_t buf[(strlen(msg) + 15) / 16 * 16]{};
    uint16_t rx_len = sizeof(buf);

    // Write
    M5.Log.printf("================================ WRITE\n");
    if (nfc_a.write(sblock, (const uint8_t*)msg, len, key)) {
        lcd.fillScreen(TFT_ORANGE);
        nfc_a.dump();

        // Verify
        if (nfc_a.read(buf, rx_len, sblock, key)) {
            lcd.fillScreen(TFT_BLUE);
            M5.Log.printf("================================ VERIFY:%s\n", memcmp(buf, msg, len) == 0 ? "OK" : "NG");
            m5::utility::log::dump(buf, rx_len, false);

            // Clear
            memset(buf, 0, sizeof(buf));
            lcd.fillScreen(TFT_MAGENTA);
            if (nfc_a.write(sblock, buf, sizeof(buf), key)) {
                M5.Log.printf("================================ CLEAR\n");
                nfc_a.dump();
                return true;
            } else {
                M5_LOGE("Failed to write");
            }
        } else {
            M5_LOGE("Failed to read");
        }
    } else {
        M5_LOGE("Failed to write %u", sblock);
    }
    return false;
}

// Using read16/write16 for MIFARE classic
void read_write_sector_structure(const UID& uid, const uint8_t block, const Key& key)
{
    constexpr char msg[16] = "M5Unit-RFID";

    // Read and write access with A authentication
    if (!nfc_a.mifareClassicAuthenticateA(uid, block, key)) {
        M5_LOGE("Failed to AuthA");
        return;
    }

    M5.Log.printf("Before[%u] ----\n", block);
    nfc_a.dump(block);

    M5.Log.printf("Write\n");
    if (!nfc_a.write16(block, (const uint8_t*)msg)) {
        M5_LOGE("Failed to write");
        return;
    }
    M5.Log.printf("After[%u] ----\n", block);
    nfc_a.dump(block);

    // Read
    uint8_t rbuf[16]{};
    if (!nfc_a.read16(rbuf, block)) {
        M5_LOGE("Failed to read");
        return;
    }

    // Verify
    bool verify = std::memcmp(rbuf, (const uint8_t*)msg, 16) == 0;
    M5.Log.printf("Verify %s\n", verify ? "OK" : "NG");

    // Clear
    M5.Log.printf("Clear\n");
    uint8_t c[16]{};
    if (!nfc_a.write16(block, c)) {
        M5_LOGE("Failed to write");
        return;
    }
    nfc_a.dump(block);
}

// Using read4,16/write4 for Ultralight,NTAG
void read_write_page_structure(const UID& uid, const uint8_t page)
{
    constexpr char msg[4] = "M5S";

    // Ultralight can only be read in 4page (16bytes) units
    uint8_t aligned_page = page & ~0x03;

    M5.Log.printf("Before[%u] ----\n", page);
    nfc_a.dump(aligned_page);

    // Write (If less than 4 bytes, 0x00 is padded)
    if (!nfc_a.write4(page, (const uint8_t*)msg)) {
        M5_LOGE("Failed to write");
        return;
    }
    M5.Log.printf("After[%u] ----\n", page);
    nfc_a.dump(aligned_page);

    // Read
    uint8_t rbuf[16]{};
    if (uid.isNTAG()) {
        if (!nfc_a.read4(rbuf, page)) {
            M5_LOGE("Failed to read");
            return;
        }
    } else {
        if (!nfc_a.read16(rbuf, page & ~0x03)) {
            M5_LOGE("Failed to read");
            return;
        }
    }

    bool verify = std::memcmp(rbuf, (const uint8_t*)msg, 4) == 0;
    M5.Log.printf("Verify %s\n", verify ? "OK" : "NG");

    // Clear
    M5.Log.printf("Clear\n");
    uint8_t c[4]{};
    if (!nfc_a.write4(page, c)) {
        M5_LOGE("Failed to write");
        return;
    }
    nfc_a.dump(aligned_page);
}

}  // namespace

void setup()
{
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5.Log.printf("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 100 * 1000U);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5.Log.printf("M5UnitUnified has been begun");
    M5.Log.printf("%s", Units.debugInfo().c_str());

    if (lcd.width() < lcd.height()) {
        lcd.setRotation(1);
    }
    if (lcd.height() < 240) {
        lcd.setFont(&fonts::Font0);
    } else {
        lcd.setFont(&fonts::Font2);
    }

    lcd.clear(0);
    lcd.setCursor(8, 0);
    lcd.printf("Please put the device and click/hold A");
    M5.Log.printf("Please put the device and click/hold A\n");
}

void loop()
{
    M5.update();
    Units.update();
    auto touch = M5.Touch.getDetail();

    bool clicked = M5.BtnA.wasClicked() || touch.wasClicked();
    bool held    = M5.BtnA.wasHold() || touch.wasHold();
    if (clicked || held) {
        std::vector<UID> devices;
        if (nfc_a.detect(devices)) {
            lcd.fillScreen(TFT_DARKGREEN);
            // If multiple occurrences are detected, only the first one detected
            auto& uid = devices.front();
            if (nfc_a.activate(uid)) {
                M5.Log.printf("UID:%s %s %u/%u\n", uid.uidAsString().c_str(), uid.typeAsString().c_str(),
                              uid.userAreaSize(), uid.totalSize());

                if (clicked) {
                    M5.Speaker.tone(2000, 30);
                    // Need key if MIFARE classic, Ignore key if not MIFARE classic
                    read_all_user_area(keyA);
                    auto ret = read_write(0, uid.userAreaSize() >= 120 ? long_msg : short_msg, keyA);
                    lcd.fillScreen(ret ? 0 : TFT_RED);
                } else if (held) {
                    M5.Speaker.tone(4000, 30);
                    if (uid.isMifareClassic()) {
                        read_write_sector_structure(uid, 13, keyA);
                    } else if (uid.supportsNFC()) {
                        read_write_page_structure(uid, 10);
                    } else {
                        M5_LOGE("Not support");
                    }
                }
                nfc_a.deactivate();
            }
        } else {
            M5.Log.printf("No devices\n");
        }
        lcd.setCursor(0, 0);
        lcd.printf("Please put the device and click/hold A");
    }
}
