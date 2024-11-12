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
#include "../../common/examples_common.hpp"
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

void value_block(const m5::rfid::mifare::UID& uid, const uint8_t block)
{
    // Need auth to sector trailer (using default key)
    uint8_t st_block = m5::rfid::mifare::classic::get_sector_trailer_block(block);
    if (!unit.authenticateA(uid, st_block)) {
        M5_LOGE("Auth error A");
        return;
    }

    // To normal
    {
        auto result =
            unit.mifareDisableValueBlock(block, UnitRFID2::DEFAULT_CLASSIC_KEY, UnitRFID2::DEFAULT_CLASSIC_KEY);
        if (!result) {
            M5_LOGE("Failed to disable value block %02X", result.error());
            return;
        }
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
        M5_LOGE("Failed to enavle value block %02X", result.error());
        return;
    }
    M5_LOGI("To value block ----");
    unit.mifareDumpBlock(uid, block);

    // Increment needs Auth B
    if (!unit.authenticateB(uid, block)) {
        M5_LOGE("Auth error A");
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

    // To normal block
    result = unit.mifareDisableValueBlock(block, UnitRFID2::DEFAULT_CLASSIC_KEY, UnitRFID2::DEFAULT_CLASSIC_KEY);
    if (!result) {
        M5_LOGE("Failed to disable value block %02X", result.error());
        return;
    }
    M5_LOGI("To normal block ----");
    unit.mifareDumpBlock(uid, block);

    ////////////////////////////////
    // Readnly value block (Decrementing is allowed)
    // raedonly value block does not allow write operation, so write the initial value first.
    result = unit.mifareWriteValue(block, 1234);  // initial value
    result = result ? unit.mifareEnableValueBlock(block, UnitRFID2::DEFAULT_CLASSIC_KEY, UnitRFID2::DEFAULT_CLASSIC_KEY,
                                                  true)
                    : result;  // // Using default keyAB
    if (!result) {
        M5_LOGE("Failed to enavle value block %02X", result.error());
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

    // To normal block
    result = unit.mifareDisableValueBlock(block, UnitRFID2::DEFAULT_CLASSIC_KEY, UnitRFID2::DEFAULT_CLASSIC_KEY);
    if (!result) {
        M5_LOGE("Failed to disable value block %02X", result.error());
        return false;
    }
    M5_LOGI("To normal block ----");
    unit.mifareDumpBlock(uid, block);
}

void loop()
{
    static UID prev{};

    M5.update();

    if (M5.BtnA.wasClicked() || M5.Touch.getCount()) {
        // Detect new devices?
        while (unit.detectIdleDevice()) {
            UID uid{};
            if (unit.activateDevice(uid)) {
                if (uid != prev) {
                    if (uid.isClassic()) {
                        M5.Speaker.tone(1000, 20);
                        printUID(uid);
                        // TODO 4K > 32 sector
                        value_block(uid, 44);
                        unit.deactivateDevice();
                        prev = uid;
                    } else {
                        M5_LOGE("ValueBlock operation is classic only");
                    }
                }
            }
        }
    }
    if (!unit.detectDevice()) {
        prev.clear();
    }
}
