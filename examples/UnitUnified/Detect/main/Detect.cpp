/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Detect NFC-A PICC
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
    Units.update();
    auto touch = M5.Touch.getDetail();

    std::vector<UID> uids;
    if (nfc_a.detect(uids)) {
        M5.Speaker.tone(3000, 10);
        lcd.fillScreen(0);
        lcd.setCursor(0, 0);
        lcd.printf("%zu PICC\n", uids.size());
        M5.Log.printf("%zu PICC\n", uids.size());
        uint32_t idx{};
        for (auto&& u : uids) {
            M5.Log.printf("UID:%s %s %u/%u\n", u.uidAsString().c_str(), u.typeAsString().c_str(), u.userAreaSize(),
                          u.totalSize());
            lcd.printf("[%2u]:UID:<%s> %s\n", idx, u.uidAsString().c_str(), u.typeAsString().c_str());
            ++idx;
        }
    }
}
