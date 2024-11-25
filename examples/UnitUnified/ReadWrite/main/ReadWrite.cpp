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

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitRFID2 unit;

}  // namespace

using namespace m5::rfid;

void setup()
{
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }

    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

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
    lcd.printf("Please put the devices\n and click A or touch screen...");
    M5_LOGI("Please put the devices\n and click A or touch screen...");
}

void read_write_sector_structure(const UID& uid, const uint8_t block)
{
    constexpr char msg[] = "M5Stack";

    // Auth
    if (!unit.mifareAuthenticateA(uid, block)) {  // using default key
        M5_LOGE("Auth error A");
        return;
    }

    M5_LOGI("Before[%u] ----", block);
    unit.dumpDevice(uid, block);

    // Write (If less than 16 bytes, 0x00 is padded)
    auto result = unit.mifareWrite(uid, block, (const uint8_t*)msg, m5::stl::size(msg));
    if (!result) {
        M5_LOGE("Failed to write %02X", result.error());
        return;
    }
    M5_LOGI("After[%u] ----", block);
    unit.dumpDevice(uid, block);

    // Read
    uint8_t rbuf[18]{};  // Need 18bytes or greater (rbuf[16,17] re CRC)
    uint8_t rlen{18};    // Number of bytes to be read
    result = unit.mifareRead(uid, rbuf, rlen, block);
    if (!result) {
        M5_LOGE("Failed to read %02X", result.error());
        return;
    }

    // Verify
    M5_LOGI("Read msg:[%s] %d,%d", (const char*)rbuf, rlen, std::memcmp(rbuf, (uint8_t*)msg, m5::stl::size(msg)));

    // Clear
    uint8_t c[1]{};
    result = unit.mifareWrite(uid, block, c, 1);
    if (!result) {
        M5_LOGE("Failed to write %02X", result.error());
        return;
    }
    M5_LOGI("Clear[%u] ----", block);
    unit.dumpDevice(uid, block);
}

void read_write_page_structure(const UID& uid, const uint8_t page)
{
    constexpr char msg[] = "M5S";

    M5_LOGI("Before[%u] ----", page);
    unit.dumpDevice(uid, page);

    // Write (If less than 16 bytes, 0x00 is padded)
    auto result = unit.mifareWriteUL(uid, page, (const uint8_t*)msg, m5::stl::size(msg));
    if (!result) {
        M5_LOGE("Failed to write %02X", result.error());
        return;
    }
    M5_LOGI("After[%u] ----", page);
    unit.dumpDevice(uid, page);

    // Read
    uint8_t rbuf[18]{};  // Need 18bytes or greater (rbuf[16,17] re CRC)
    uint8_t rlen{18};    // Number of bytes to be read
    result = unit.mifareRead(uid, rbuf, rlen, page);
    if (!result) {
        M5_LOGE("Failed to read %02X", result.error());
        return;
    }

    // Verify
    M5_LOGI("Read msg:[%s] %d,%d", (const char*)rbuf, rlen, std::memcmp(rbuf, (uint8_t*)msg, m5::stl::size(msg)));

    // Clear
    uint8_t c[1]{};
    result = unit.mifareWriteUL(uid, page, c, 1);
    if (!result) {
        M5_LOGE("Failed to write %02X", result.error());
        return;
    }
    M5_LOGI("Clear[%u] ----", page);
    unit.dumpDevice(uid, page);
}

void loop()
{
    M5.update();
    if (M5.BtnA.wasClicked() || M5.Touch.getCount()) {
        // Detect new devices?
        if (unit.detectIdleDevice()) {
            UID uid{};
            if (unit.activateDevice(uid)) {
                M5.Speaker.tone(1000, 20);
                M5_LOGI("UID:%s %s", uid.uidAsString().c_str(), uid.typeAsString().c_str());
                switch (uid.type) {
                    case Type::MIFARE_Classic_1K:
                        read_write_sector_structure(uid, 12);
                        break;
                    case Type::MIFARE_Classic_4K:
                        read_write_sector_structure(uid, 12);
                        read_write_sector_structure(uid, 145);
                        break;
                    case Type::MIFARE_UltraLight:
                        read_write_page_structure(uid, 12);
                        break;
                    default:
                        M5_LOGE("For Classic1/4K and LightC");
                        break;
                }
                unit.deactivateDevice();
            }
        }
    }
}
