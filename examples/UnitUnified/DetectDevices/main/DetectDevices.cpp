/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Detect RFID devices
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
    Wire.end();
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
    lcd.printf("Please put the devices...");
    M5.Log.printf("Please put the devices...\n");
}

void loop()
{
    M5.update();
    Units.update();

    std::vector<UID> devices;
    // Detect new devices?
    while (unit.detectIdleDevice()) {
        // Store UID
        UID uid{};
        if (unit.activateDevice(uid)) {
            devices.push_back(uid);
            unit.deactivateDevice();
        } else {
            M5_LOGE("Failed to activate");
        }
    }
    // display / logging
    if (!devices.empty()) {
        M5.Log.printf(">>--------------\n");
        M5.Speaker.tone(1000, 20);
        lcd.fillRect(0, lcd.fontHeight(), lcd.width(), lcd.height() - lcd.fontHeight(), 0);

        uint32_t idx{1};
        for (auto&& uid : devices) {
            M5.Log.printf("UID:%s %s\n", uid.uidAsString().c_str(), uid.typeAsString().c_str());
            lcd.setCursor(8, lcd.fontHeight() * idx);
            lcd.printf("%s:%s", uid.uidAsString().c_str(), uid.typeAsString().c_str());
            ++idx;
        }
        M5.Log.printf("<<--------------\n");
    }
    m5::utility::delay(1);
}
