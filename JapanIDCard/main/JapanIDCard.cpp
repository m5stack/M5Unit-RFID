/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-NFC/RFID
  JapanIDCard example (read Mynumber Ticket Information Input Assistant AP)
  This example is shared with M5Unit-RFID
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedNFC.h>
#include <M5Utility.h>
#include <Wire.h>
#include <M5HAL.hpp>  // For NessoN1
#include <vector>

// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_NFC) && !defined(USING_CAP_CC1101) && !defined(USING_UNIT_RFID2)
// For UnitNFC
// #define USING_UNIT_NFC
// For CapNFC
// #define USING_CAP_CC1101
// For UnitRFID2
// #define USING_UNIT_RFID2
#endif

#if defined(USING_UNIT_RFID2)
#include <M5UnitUnifiedRFID.h>
#endif

using namespace m5::nfc;
using namespace m5::nfc::b;

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
m5::unit::UnitRFID2 unit{};  // UnitRFID2 (M5Unit-RFID, WS1850S)
#else
#error Choose unit please!
#endif
m5::nfc::NFCLayerB nfc_b{unit};

void dump()
{
    NFCBFileSystem fs(nfc_b);

    // Mynumber Ticket Information Input Assistant AP
    constexpr uint8_t DF1[] = {0xD3, 0x92, 0x10, 0x00, 0x31, 0x00, 0x01, 0x01, 0x04, 0x08};
    if (!fs.selectByDfName(DF1, sizeof(DF1), m5::nfc::apdu::SelectResponse::FCI)) {
        M5_LOGE("Not JapanIDCard");
        return;
    }

    constexpr uint8_t EF[] = {0x00, 0x11};
    if (!fs.selectFile(m5::nfc::apdu::SelectBy::EfUnderCurrentDf, m5::nfc::apdu::SelectOccurrence::FirstOrOnly,
                       m5::nfc::apdu::SelectResponse::None, EF, sizeof(EF))) {
        M5_LOGE("Failed to select EF");
        return;
    }

    // ********************************************
    // // Password (YOU MUST CHANGE IT!) as string
    // ********************************************
    constexpr char pass[] = "XXXX";
    if (!fs.verifyGlobal((uint8_t*)pass, 4)) {
        M5_LOGE("Failed to verify");
        return;
    }

    constexpr uint8_t EF2[] = {0x00, 0x02};  // Name etc...
    if (!fs.selectFile(m5::nfc::apdu::SelectBy::EfUnderCurrentDf, m5::nfc::apdu::SelectOccurrence::FirstOrOnly,
                       m5::nfc::apdu::SelectResponse::None, EF2, sizeof(EF2))) {
        M5_LOGE("Failed to select EF2");
        return;
    }

    std::vector<uint8_t> buf;
    if (!fs.readBinary(buf, 2, 1)) {
        M5_LOGE("Failed to readBinary");
        return;
    }
    M5_LOGI("readBinary size = %u", buf.empty() ? 0 : buf[0]);
    // buf[0] == size
    if (!fs.readBinary(buf, 0, buf[0])) {
        M5_LOGE("Failed to readBinary");
        return;
    }
    m5::utility::log::dump(buf.data(), buf.size(), false);
}

}  // namespace

void setup()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);

    // Select NFC-B mode via config() (applied at begin())
    auto cfg = unit.config();
    cfg.mode = NFC::B;
    unit.config(cfg);

#if defined(USING_UNIT_NFC) || defined(USING_UNIT_RFID2)
    // NessoN1: Arduino Wire (I2C_NUM_0) cannot be used for GROVE port.
    //   Wire is used by M5Unified In_I2C for internal devices.
    // NanoC6: Wire.begin() on GROVE pins conflicts with Ex_I2C on the same I2C_NUM_0.
    auto board = M5.getBoard();
    bool unit_ready{};
#if defined(USING_M5DIAL_BUILTIN_WS1850S)
    M5_LOGI("Using M5.In_I2C for builtin WS1850S");
    unit_ready = Units.add(unit, M5.In_I2C) && Units.begin();
#else
    if (board == m5::board_t::board_M5NanoC6) {
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
#endif  // USING_M5DIAL_BUILTIN_WS1850S
    if (!unit_ready) {
        M5_LOGE("Failed to begin");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
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
    if (!Units.add(unit, SPI, settings) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.fillScreen(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
#endif

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }
    lcd.setFont(&fonts::Font0);
    lcd.fillScreen(0);
    lcd.setCursor(0, 0);
    lcd.printf("Please put the PICC and click BtnA");
    M5.Log.printf("Please put the PICC and click BtnA\n");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillRect(0, lcd.fontHeight(), lcd.width(), lcd.height() - lcd.fontHeight());
        PICC picc{};
        if (nfc_b.select(picc)) {
            M5.Speaker.tone(3000, 20);
            M5.Log.printf("==== PICC %s %s ====\n", picc.pupiAsString().c_str(), picc.typeAsString().c_str());
            dump();
            nfc_b.deactivate();
        } else {
            M5.Log.printf("PICC NOT exists\n");
        }
    }
}
