/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Value block example (Only Classic)
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

using namespace m5::unit::mfrc522;
using m5::rfid::mifare::Key;
using m5::rfid::mifare::UID;
using m5::unit::UnitRFID2;

void setup()
{
    delay(3000);

    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 100 * 1000U);

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

// Set block to normal block and also set the trailer sector block to the default value
void restore_sector_trailer(const m5::rfid::mifare::UID& uid, const uint8_t block)
{
    if (!uid.isClassic()) {
        return;
    }

    uint8_t st_block = m5::rfid::mifare::classic::get_sector_trailer_block(block);
    if (unit.mifareAuthenticateB(uid, st_block) || unit.mifareAuthenticateA(uid, st_block)) {
        uint8_t buf[18]{};
        uint8_t permissions[4] = {0, 0, 0, 1 /*sector trailer*/};
        if (m5::rfid::mifare::classic::encode_access_bits(buf + 6, permissions)) {
            std::memcpy(buf, UnitRFID2::DEFAULT_CLASSIC_KEY.data(), 6);
            std::memcpy(buf + 10, UnitRFID2::DEFAULT_CLASSIC_KEY.data(), 6);
            if (unit.mifareWrite(st_block, buf, 16, false)) {
                M5_LOGI("block:%u sector trailer:%u to [001]", block, st_block);
                return;
            }
        }
    }
    M5_LOGE("Failed");
}

void value_block(const m5::rfid::mifare::UID& uid, const uint8_t block)
{
    // Write permission to the sector trailer is required to change to the value block
    uint8_t st_block = m5::rfid::mifare::classic::get_sector_trailer_block(block);
    if (!unit.mifareAuthenticateB(uid, st_block) || !unit.mifareAuthenticateA(uid, st_block)) {
        M5_LOGE("Auth error");
        return;
    }

    M5_LOGI("Before[%u] ----", block);
    unit.mifareDumpBlock(uid, block);

    ////////////////////////////////
    // Writable value block
    // To value block
    auto result = unit.mifareWriteValue(block, 12345678);  // initial value
    result = result ? unit.mifareEnableValueBlock(block, UnitRFID2::DEFAULT_CLASSIC_KEY, UnitRFID2::DEFAULT_CLASSIC_KEY)
                    : result;
    if (!result) {
        M5_LOGE("Failed to enable value block %02X", result.error());
        return;
    }
    M5_LOGI("To value block ----");
    unit.mifareDumpBlock(uid, block);

    // Key B authentication is required for value block increment operations
    if (!unit.mifareAuthenticateB(uid, block)) {
        M5_LOGE("Auth error B");
        return;
    }

    // Increment/decrement (bNote that it will not be applied to the device unless it is transferred)
    result = result ? unit.mifareIncrement(block, 987654321) : result;  // 12345678 -> 999999999
    result = result ? unit.mifareTransfer(block) : result;              // apply
    result = result ? unit.mifareDecrement(block, 9999) : result;       // 999999999 -> 999990000
    result = result ? unit.mifareTransfer(block) : result;              // apply
    if (!result) {
        M5_LOGE("Failed to value block operation %02X", result.error());
        return;
    }
    M5_LOGI("Inc/Dec ----");
    unit.mifareDumpBlock(uid, block);

}

void value_block_readonly(const m5::rfid::mifare::UID& uid, const uint8_t block)
{
    if (!uid.isClassic()) {
        return;
    }

    // Write permission to the sector trailer is required to change to the value block
    uint8_t st_block = m5::rfid::mifare::classic::get_sector_trailer_block(block);
    M5_LOGI("block:%u sector trailer block:%u", block, st_block);
    if (!unit.mifareAuthenticateB(uid, st_block) || !unit.mifareAuthenticateA(uid, st_block)) {
        // if (!unit.authenticateA(uid, st_block) || !unit.authenticateB(uid, st_block)) {
        M5_LOGE("Auth error");
        return;
    }

    ////////////////////////////////
    // Readnly value block (Decrementing is allowed)
    // raedonly value block does not allow write operation, so write the initial value first.
    auto result = unit.mifareWriteValue(block, 1234);  // initial value
    result = result ? unit.mifareEnableValueBlock(block, UnitRFID2::DEFAULT_CLASSIC_KEY, UnitRFID2::DEFAULT_CLASSIC_KEY,
                                                  true)
                    : result;  // // Using default keyAB
    if (!result) {
        M5_LOGE("Failed to enable value block %02X", result.error());
        return;
    }
    M5_LOGI("To value block ----");
    unit.mifareDumpBlock(uid, block);

    // Decrement
    result = result ? unit.mifareDecrement(block, 1234 * 2) : result;  // 1234 -> -1234
    result = result ? unit.mifareTransfer(block) : result;             // apply
    if (!result) {
        M5_LOGE("Failed to value block operation %02X", result.error());
        return;
    }
    M5_LOGI("Dec ----");
    unit.mifareDumpBlock(uid, block);

    #if 0
    // Key B authentication is required for disableValueBlock
    if (!unit.mifareAuthenticateB(uid, block)) {
        M5_LOGE("Auth error B");
        return;
    }
    #endif
    
}

void loop()
{
    M5.update();
    auto touch = M5.Touch.getDetail();

    // Value block operatrion (IDLE)
    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        // Detect new devices?
        if (unit.detectIdleDevice()) {
            UID uid{};
            if (unit.activateDevice(uid)) {
                if (uid.isClassic()) {
                    M5.Speaker.tone(1000, 20);
                    M5_LOGI("UID:%s %s", uid.uidString().c_str(), uid.typeString().c_str());

                    uint8_t block = uid.type == m5::rfid::mifare::Type::MIFARE_Classic_4K ? 128 : 44;
                    value_block(uid, block);
                    //value_block_readonly(uid, block);

                    unit.deactivateDevice();
                } else {
                    M5_LOGE("Classic only");
                }
            }
        }
    }

    // Restore sector traler (IDLE)
    if (M5.BtnA.isHolding() || touch.isHolding()) {
        // Detect new devices?
        while (unit.detectIdleDevice()) {
            UID uid{};
            if (unit.activateDevice(uid)) {
                if (uid.isClassic()) {
                    M5_LOGI("Restore");
                    M5.Speaker.tone(1000, 20);
                    M5_LOGI("UID:%s %s", uid.uidString().c_str(), uid.typeString().c_str());

                    uint8_t block = uid.type == m5::rfid::mifare::Type::MIFARE_Classic_4K ? 128 : 44;
                    restore_sector_trailer(uid, block);
                    if (unit.mifareAuthenticateA(uid, block)) {
                        unit.mifareDumpBlock(uid, block);
                    }

                    unit.deactivateDevice();
                } else {
                    M5_LOGE("Classic only");
                }
            }
        }
    }

    // Dump (IDLE/HLT)
    if (M5.BtnA.wasDoubleClicked() || touch.wasFlicked()) {
        if (unit.detectDevice()) {
            UID uid{};
            if (unit.activateDevice(uid)) {
                if (uid.isClassic()) {
                    M5_LOGI("Dump");
                    uint8_t block = uid.type == m5::rfid::mifare::Type::MIFARE_Classic_4K ? 128 : 44;
                    if (unit.mifareAuthenticateA(uid, block)) {
                        M5.Speaker.tone(1000, 20);
                        unit.mifareDumpBlock(uid, block);
                        unit.deactivateDevice();
                    }
                } else {
                    M5_LOGE("Classic only");
                }
            }
        }
    }
}
