/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Detect NFC-A devices
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5UnitUnifiedNFC.h>
#include <M5Utility.h>
#include <vector>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitRFID2 unit;
m5::unit::nfc::NFCLayerA nfc_a{unit};

}  // namespace

using namespace m5::nfc::a;

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
    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
}

void loop()
{
    M5.update();
    auto touch = M5.Touch.getDetail();
    Units.update();

    std::vector<UID> devices;
    if (nfc_a.detect(devices)) {
        M5.Speaker.tone(3000, 10);
        lcd.fillRect(0, 0, lcd.width(), lcd.height());
        lcd.setCursor(0, 0);
        M5.Log.printf("Devices: %zu\n", devices.size());
        lcd.printf("Devices: %zu\n", devices.size());
        uint32_t idx{};
        for (auto&& u : devices) {
            M5.Log.printf("[%2u]:UID:<%s> %s\n", idx, u.uidAsString().c_str(), u.typeAsString().c_str());
            lcd.printf("[%2u]:UID:<%s> %s\n", idx, u.uidAsString().c_str(), u.typeAsString().c_str());
            ++idx;
        }
        nfc_a.deactivate();
    }
}
