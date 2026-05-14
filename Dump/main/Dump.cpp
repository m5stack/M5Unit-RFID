/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-NFC/RFID
  Dump NFC-A PICC
  This example is shared with M5Unit-RFID
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedNFC.h>
#include <M5Utility.h>
#include <Wire.h>
#include <vector>

// *************************************************************
// Choose ONE define symbol to match the unit/board you are using
// *************************************************************
#if !defined(USING_UNIT_NFC) && !defined(USING_CAP_CC1101) && !defined(USING_UNIT_RFID2) && \
    !defined(USING_M5DIAL_BUILTIN_WS1850S)
// For UnitNFC (ST25R3916, I2C)
// #define USING_UNIT_NFC
// For CapCC1101NFC (ST25R3916, SPI)
// #define USING_CAP_CC1101
// For UnitRFID2 (WS1850S external, I2C GROVE)
// #define USING_UNIT_RFID2
// For M5Dial Builtin WS1850S (internal I2C)
// #define USING_M5DIAL_BUILTIN_WS1850S
#endif

#if defined(USING_UNIT_RFID2) || defined(USING_M5DIAL_BUILTIN_WS1850S)
#include <M5UnitUnifiedRFID.h>
#endif

using namespace m5::nfc::a;
using namespace m5::nfc::a::mifare;
using namespace m5::nfc::a::mifare::classic;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;

#if defined(USING_UNIT_NFC)
#pragma message "Choose UnitNFC"
m5::unit::UnitNFC unit{};  // I2C
#elif defined(USING_CAP_CC1101)
#pragma message "Choose CapCC1101NFC"
m5::unit::CapCC1101NFC unit{};  // CapCC1101 (SPI)
#elif defined(USING_UNIT_RFID2)
#pragma message "Choose UnitRFID2"
m5::unit::UnitRFID2 unit{};  // UnitRFID2 external (M5Unit-RFID, GROVE)
#elif defined(USING_M5DIAL_BUILTIN_WS1850S)
#pragma message "Choose UnitRFID2 (M5Dial Builtin)"
m5::unit::UnitRFID2 unit{};  // M5Dial builtin WS1850S (internal I2C)
#else
#error Choose ONE: USING_UNIT_NFC / USING_CAP_CC1101 / USING_UNIT_RFID2 / USING_M5DIAL_BUILTIN_WS1850S
#endif
m5::nfc::NFCLayerA nfc_a{unit};

// KeyA that can authenticate all blocks
// If it's a different key value, change it
constexpr Key keyA = DEFAULT_KEY;  // Default as 0xFFFFFFFFFFFF
}  // namespace

void setup()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);

    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    bool unit_ready{};

#if defined(USING_M5DIAL_BUILTIN_WS1850S)
    // M5Dial builtin WS1850S: small loop antenna; reduce RxGain to 33dB to mitigate
    // reflection interference (default 48dB causes unstable WUPA on Builtin).
    {
        auto cfg          = unit.config();
        cfg.receiver_gain = m5::unit::mfrc522::ReceiverGain::dB33;
        unit.config(cfg);
    }
    // M5Dial builtin WS1850S on In_I2C (G12/G11, shared with RTC8563)
    M5_LOGI("Using M5.In_I2C for builtin WS1850S");
    unit_ready = Units.add(unit, M5.In_I2C) && Units.begin();

#elif defined(USING_UNIT_NFC) || defined(USING_UNIT_RFID2)
    // External I2C unit (GROVE port).
    // NessoN1: Arduino Wire (I2C_NUM_0) cannot be used for GROVE port (used by In_I2C internals).
    //   Use QWIIC (port_a) with Wire. (Requires QWIIC-GROVE conversion cable)
    // NanoC6: Wire.begin() on GROVE pins conflicts with Ex_I2C on I2C_NUM_0; use M5.Ex_I2C directly.
    if (M5.getBoard() == m5::board_t::board_M5NanoC6) {
        M5_LOGI("Using M5.Ex_I2C");
        unit_ready = Units.add(unit, M5.Ex_I2C) && Units.begin();
    } else {
        auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
        auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
        M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        Wire.end();
        Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);
        unit_ready = Units.add(unit, Wire) && Units.begin();
    }

#elif defined(USING_CAP_CC1101)
    if (!SPI.bus()) {
        auto spi_sclk = M5.getPin(m5::pin_name_t::sd_spi_sclk);
        auto spi_mosi = M5.getPin(m5::pin_name_t::sd_spi_mosi);
        auto spi_miso = M5.getPin(m5::pin_name_t::sd_spi_miso);
        M5_LOGI("getPin: %d,%d,%d", spi_sclk, spi_mosi, spi_miso);
        SPI.begin(spi_sclk, spi_miso, spi_mosi /* SS is shared SD, CC1101, ST25R3916 */);
    }
    SPISettings settings = {10000000, MSBFIRST, SPI_MODE1};
    unit_ready           = Units.add(unit, SPI, settings) && Units.begin();
#endif

    if (!unit_ready) {
        M5_LOGE("Failed to begin");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    lcd.setFont(&fonts::Font0);
    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setCursor(0, lcd.height() / 2);
    lcd.printf("Please put the PICC and click BtnA");
    M5.Log.printf("Please put the PICC and click BtnA\n");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, lcd.height() / 2);
        PICC picc{};
        if (nfc_a.detect(picc)) {
            if (nfc_a.identify(picc) && nfc_a.reactivate(picc)) {
                M5.Speaker.tone(3000, 20);
                lcd.printf("%s\n%s", picc.uidAsString().c_str(), picc.typeAsString().c_str());
                M5.Log.printf("==== Dump %s %s %u/%u ====\n", picc.uidAsString().c_str(), picc.typeAsString().c_str(),
                              picc.userAreaSize(), picc.totalSize());
                nfc_a.dump(keyA);  // Need key if MIFARE classic, Ignore key if not MIFARE classic
                nfc_a.deactivate();
            } else {
                lcd.printf("Failed to identify");
                M5_LOGE("Failed to identify/activate %s", picc.uidAsString().c_str());
            }
        } else {
            lcd.printf("PICC NOT exists");
            M5.Log.printf("PICC NOT exists\n");
        }
    }
}
