/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Dump to serial
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
    lcd.printf("Please put the devices...");
    M5_LOGI("Please put the devices...");
}

void loop()
{
    static UID prev{};

    M5.update();

    std::vector<UID> devices;
    // Detect new devices?
    while (unit.detectIdleDevice()) {
        UID uid{};
        if (unit.activateDevice(uid)) {
            if (uid != prev) {
                M5.Speaker.tone(1000, 20);
                M5_LOGI("UID:%s %s", uid.uidString().c_str(), uid.typeString().c_str());
                unit.mifareDump(uid);  // Using defaukt keyA {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF }
                unit.deactivateDevice();
                prev = uid;
            }
        }
    }

    // No devices?
    if (!unit.detectDevice()) {
        prev.clear();
    }
}
