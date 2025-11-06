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

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitRFID2 unit;
m5::unit::nfc::NFCLayerA nfc_a{unit};

//
void read_write_sector_structure(const UID& uid, const uint8_t block, const Key& mkey = DEFAULT_CLASSIC_KEY)
{
    constexpr char msg[] = "M5Unit-RFID";

    // Read and write access with A authentication
    if (!nfc_a.mifareAuthenticateA(uid, block, mkey)) {
        M5_LOGE("Failed to AuthA");
        return;
    }

    M5.Log.printf("Before[%u] ----\n", block);
    nfc_a.dump(uid, block);

    M5.Log.printf("Write\n");
    // Write (If less than 16 bytes, 0x00 is padded)
    if (!nfc_a.writeBlock(block, (const uint8_t*)msg, m5::stl::size(msg))) {
        M5_LOGE("Failed to write");
        return;
    }
    M5.Log.printf("After[%u] ----\n", block);
    nfc_a.dump(uid, block);

    // Read
    uint8_t rbuf[16]{};
    uint16_t rlen{16};
    if (!nfc_a.readBlock(rbuf, rlen, block)) {
        M5_LOGE("Failed to read");
        return;
    }

    // Verify
    bool verify = std::memcmp(rbuf, (uint8_t*)msg, m5::stl::size(msg)) == 0;
    M5.Log.printf("Verify msg:[%s] %d %s\n", (const char*)rbuf, rlen, verify ? "OK" : "!!!VERIFY NG!!!");

    // Clear
    M5.Log.printf("Clear\n");
    uint8_t c[1]{};
    if (!nfc_a.writeBlock(block, c, 1)) {
        M5_LOGE("Failed to write");
        return;
    }
    nfc_a.dump(uid, block);
}

void read_write_page_structure(const UID& uid, const uint8_t page)
{
    constexpr char msg[] = "M5S";

    M5.Log.printf("Before[%u] ----\n", page);
    nfc_a.dump(uid, page);

    // Write (If less than 4 bytes, 0x00 is padded)
    if (!nfc_a.writeBlock(page, (const uint8_t*)msg, m5::stl::size(msg))) {
        M5_LOGE("Failed to write");
        return;
    }
    M5.Log.printf("After[%u] ----\n", page);
    nfc_a.dump(uid, page);

    // Read
    uint8_t rbuf[16]{};
    uint16_t rlen{16};
    if (!nfc_a.readBlock(rbuf, rlen, page)) {
        M5_LOGE("Failed to read");
        return;
    }

    bool verify = std::memcmp(rbuf, (uint8_t*)msg, m5::stl::size(msg)) == 0;
    M5.Log.printf("Verify msg:[%s] %d %s\n", (const char*)rbuf, rlen, verify ? "OK" : "!!!VERIFY NG!!!");

    // Clear
    M5.Log.printf("Clear\n");
    uint8_t c[1]{};
    if (!nfc_a.writeBlock(page, c, 1)) {
        M5_LOGE("Failed to write");
        return;
    }
    nfc_a.dump(uid, page);
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
    lcd.printf("Please put the devices\n and click A");
    M5.Log.printf("Please put the devices\n and click A\n");
}

void loop()
{
    M5.update();
    Units.update();
    auto touch = M5.Touch.getDetail();

    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        lcd.fillRect(0, lcd.fontHeight(), lcd.width(), lcd.height() - lcd.fontHeight());
        std::vector<UID> devices;
        if (nfc_a.detect(devices)) {
            M5.Speaker.tone(2000, 30);
            // If multiple occurrences are detected, only the first one detected
            auto& uid = devices.front();
            if (nfc_a.activate(uid)) {
                // nfc_a.dump(uid);
                M5.Log.printf("UID:%s %s\n", uid.uidAsString().c_str(), uid.typeAsString().c_str());
                if (uid.isClassic()) {
                    read_write_sector_structure(uid, 13);
                    if (uid.type == Type::MIFARE_Classic_4K) {
                        read_write_sector_structure(uid, 145);
                    }
                } else if (uid.supportsNFC()) {
                    read_write_page_structure(uid, 13);
                } else {
                }
                nfc_a.deactivate();
            }
        } else {
            M5.Log.printf("No devices\n");
        }
    }
}
