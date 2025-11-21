/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitRFID2
  Dump NFC-A PICC
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5UnitUnifiedNFC.h>
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
}  // namespace

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
    lcd.printf("Please put the PICC and click A");
    M5.Log.printf("Please put the PICC and click A\n");
}

void loop()
{
    M5.update();
    auto touch = M5.Touch.getDetail();
    Units.update();

    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        lcd.fillRect(0, lcd.fontHeight(), lcd.width(), lcd.height() - lcd.fontHeight());
        std::vector<UID> uids;
        if (nfc_a.detect(uids)) {
            // If multiple occurrences are detected, only the first one detected
            auto& uid = uids.front();
            if (nfc_a.reactivate(uid)) {
                M5.Speaker.tone(2000, 30);
                M5.Log.printf("==== Dump %s %s %u/%u====\n", uid.uidAsString().c_str(), uid.typeAsString().c_str(),
                              uid.userAreaSize(), uid.totalSize());
                nfc_a.dump(keyA);  // Need key if MIFARE classic, Ignore key if not MIFARE classic

                nfc_a.deactivate();
                M5.Log.printf("Please remove the PICC from the reader\n");
            } else {
                M5_LOGE("Failed to activate %s", uid.uidAsString().c_str());
            }
        } else {
            M5.Log.printf("PICC NOT exists\n");
        }
    }
}
