/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitWS1850S
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <unit/unit_WS1850S.hpp>
#include <M5Utility.hpp>
#include <esp_random.h>
#include <cstring>

namespace {
}  // namespace

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::mfrc522;
using namespace m5::unit::mfrc522::command;
using namespace m5::nfc::a;
using namespace m5::nfc::a::mifare;
using namespace m5::nfc::a::mifare::classic;

class TestWS1850S : public I2CComponentTestBase<UnitWS1850S> {
protected:
    virtual UnitWS1850S* get_instance() override
    {
        return new m5::unit::UnitWS1850S();
    }

    virtual bool begin() override
    {
#if defined(USING_M5DIAL_BUILTIN_WS1850S)
        // M5Dial builtin WS1850S on In_I2C (G12/G11, shared with RTC8563)
        M5_LOGI("Using M5.In_I2C for builtin WS1850S");
        return Units.add(*unit, M5.In_I2C) && Units.begin();
#else
        // NessoN1: align with examples — use QWIIC (port_a) + Wire instead of the
        // base class default (SoftwareI2C on port_b GROVE). Requires a QWIIC-GROVE
        // conversion cable, identical wiring to examples/UnitUnified/NFC*/*.
        if (M5.getBoard() == m5::board_t::board_ArduinoNessoN1) {
            return begin_with_wire(Wire);
        }
        return I2CComponentTestBase<UnitWS1850S>::begin();
#endif
    }
};

TEST_F(TestWS1850S, selfTest)
{
    SCOPED_TRACE(ustr);
    EXPECT_FALSE(unit->selfTest());  // WS1850S failed always
}

TEST_F(TestWS1850S, coprocessorCRC)
{
    const std::array<uint8_t, 8> tdata{
        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    };

    struct Table {
        const char* name;
        uint8_t mode;  // ModeReg
        uint16_t init, poly;
        bool refIn, refOut;
        uint16_t xorout;
    };
    constexpr Table table[] = {
        // -- LSB first --
        // CRC-16/KERMIT
        {"0x3C", 0x3C, 0x0000, 0x1021, true, true, 0x0000},
        // CRC-A [ISO/IEC14443-3 Type A]
        {"0x3D", 0x3D, 0xC6C6, 0x1021, true, true, 0x0000},  // reverse 0x6363
        // ???
        {"0x3E", 0x3E, 0x8E65, 0x1021, true, true, 0x0000},  // reverse 0xA671
        // // CRC-16/MCRF4XX
        {"0x3F", 0x3F, 0xFFFF, 0x1021, true, true, 0x0000},
        // -- MSB first --
        {"0xBC", 0xBC, 0x0000, 0x1021, false, false, 0x0000},
        {"0xBD", 0xBD, 0xC6C6, 0x1021, false, false, 0x0000},
        {"0xBE", 0xBE, 0x8E65, 0x1021, false, false, 0x0000},
        {"0xBF", 0xBF, 0xFFFF, 0x1021, false, false, 0x0000},
    };

    // Change preset value for CRC coprocessor and check CRC values
    for (auto&& e : table) {
        SCOPED_TRACE(e.name);
        unit->writeRegister8(MODE_REG, e.mode);  // write MSGFirst:7, CRCPreset[1:0]

        uint16_t crc_h{}, crc_s{};
        EXPECT_TRUE(unit->calculateCRC(crc_h, tdata.data(), tdata.size()));
        EXPECT_TRUE(unit->calculateSoftwareCRC(crc_s, tdata.data(), tdata.size()));
        EXPECT_EQ(crc_h, crc_s);

#if 0
        m5::utility::CRC16 crc(e.init, e.poly, e.refIn, e.refOut, e.xorout);
        uint16_t crc_s = crc.range(tdata.data(), tdata.size());
        EXPECT_EQ(crc_h, crc_s);
#endif
    }
}

TEST_F(TestWS1850S, Antenna)
{
    SCOPED_TRACE(ustr);

    uint8_t prev{}, now{};
    bool status{};

    // on after begin
    EXPECT_TRUE(unit->readAntennaStatus(status));
    EXPECT_TRUE(status);

    // to ON
    EXPECT_TRUE(unit->readRegister8(TX_CONTROL_REG, prev, 0));

    EXPECT_TRUE(unit->turnOffAntenna());
    EXPECT_TRUE(unit->readRegister8(TX_CONTROL_REG, now, 0));
    EXPECT_TRUE(unit->readAntennaStatus(status));
    EXPECT_FALSE(status);
    EXPECT_NE(now, prev);
    prev = now;

    // to OFF
    EXPECT_TRUE(unit->turnOnAntenna());
    EXPECT_TRUE(unit->readRegister8(TX_CONTROL_REG, now, 0));
    EXPECT_TRUE(unit->readAntennaStatus(status));
    EXPECT_TRUE(status);
    EXPECT_NE(now, prev);

    // Change gain
    constexpr ReceiverGain table[] = {
        ReceiverGain::dB18, ReceiverGain::dB23, ReceiverGain::dB33,
        ReceiverGain::dB38, ReceiverGain::dB43, ReceiverGain::dB48,
    };

    EXPECT_TRUE(unit->readRegister8(RFC_FG_REG, prev, 0));

    for (auto&& e : table) {
        EXPECT_TRUE(unit->writeReceiverGain(e)) << (int)e;
        EXPECT_TRUE(unit->readRegister8(RFC_FG_REG, now, 0));

        ReceiverGain gain{};
        EXPECT_TRUE(unit->readReceiverGain(gain)) << (int)e;
        EXPECT_EQ(gain, e) << (int)e;
        EXPECT_NE(now, prev) << (int)e;
        prev = now;
    }
}

TEST_F(TestWS1850S, GsNRegister)
{
    SCOPED_TRACE(ustr);

    // GsN: 8-bit register (CWGsN[7:4] / ModGsN[3:0]), all 256 values valid
    for (uint16_t v = 0x00; v <= 0xFF; ++v) {
        SCOPED_TRACE(m5::utility::formatString("GsN=0x%02X", v));
        EXPECT_TRUE(unit->writeGsN(static_cast<uint8_t>(v)));
        uint8_t r{};
        EXPECT_TRUE(unit->readGsN(r));
        EXPECT_EQ(r, static_cast<uint8_t>(v));
    }

    // Restore reset default
    EXPECT_TRUE(unit->writeGsN(0x88));
}

TEST_F(TestWS1850S, Tprescaler)
{
    SCOPED_TRACE(ustr);

    // Raw I/O
    uint32_t cnt{32};
    while (cnt--) {
        uint16_t tps{};
        tps = esp_random() & 0x0FFF;  // 12 bits
        SCOPED_TRACE(m5::utility::formatString("tps: %u", tps));
        EXPECT_TRUE(unit->writeTPrescaler(tps));

        uint16_t v{};
        EXPECT_TRUE(unit->readTPrescaler(v));

        EXPECT_EQ(tps, v);
    }
}

TEST_F(TestWS1850S, AccessBit)
{
    SCOPED_TRACE(ustr);

    for (uint8_t i0 = 0; i0 < 8; i0++) {
        for (uint8_t i1 = 0; i1 < 8; i1++) {
            for (uint8_t i2 = 0; i2 < 8; i2++) {
                for (uint8_t i3 = 0; i3 < 8; i3++) {
                    uint8_t permissions[4]{i0, i1, i2, i3};
                    uint8_t abits[3]{};
                    uint8_t p2[4]{};
                    auto s = m5::utility::formatString("P:%02X:%02X:%02X:%02X", i0, i1, i2, i3);
                    SCOPED_TRACE(s);

                    EXPECT_TRUE(encode_access_bits(abits, permissions));
                    EXPECT_TRUE(decode_access_bits(p2, abits));
                    EXPECT_TRUE(std::memcmp(permissions, p2, 4) == 0);
                }
            }
        }
    }
}

#if 0
// Condition : Classic1K and RFID2 must be in contact
TEST_F(TestWS1850S, Detect)
{
    SCOPED_TRACE(ustr);

    UID uid{};
    uint8_t data[16] = {                          // Hexspeak :)
                        0xAB, 0xAD, 0xBA, 0xBE,   // a bad babe
                        0xCA, 0xFE, 0xBA, 0xBE,   // cafe babe
                        0xDE, 0xAD, 0xBE, 0xEF,   // dead beef
                        0xFA, 0xCE, 0xFE, 0xED};  // face feed
    uint8_t rbuf[18];
    uint8_t rlen{18};

    // activate
    EXPECT_TRUE(unit->detectIdleDevice());
    EXPECT_TRUE(unit->activateDevice(uid));
    EXPECT_EQ(uid.type, Type::MIFARE_Classic_1K);
    EXPECT_TRUE(uid.isClassic());

    // auth
    EXPECT_TRUE(unit->mifareAuthenticateA(uid, 52));

    // write,read,compare
    EXPECT_TRUE(unit->writeDevice(uid, 52, data, 16));
    EXPECT_TRUE(unit->readDevice(uid, rbuf, rlen, 52));
    EXPECT_TRUE(std::memcmp(data, rbuf, 16) == 0);

    // deactivate
    EXPECT_TRUE(unit->deactivateDevice());
}
#endif

TEST_F(TestWS1850S, NFCBMode)
{
    SCOPED_TRACE(ustr);
    uint8_t v{};

    // Switch to NFC-B
    EXPECT_TRUE(unit->configureNFCMode(m5::nfc::NFC::B));

    EXPECT_TRUE(unit->readRegister8(MODE_REG, v, 0));
    EXPECT_EQ(v & 0x03, 0x03);  // CRCPreset=0b11 (CRC_B 0xFFFF preset)

    EXPECT_TRUE(unit->readRegister8(TX_MODE_REG, v, 0));
    EXPECT_EQ(v, 0x03);  // TxCRCEn=0, TxFraming=0b11

    EXPECT_TRUE(unit->readRegister8(RX_MODE_REG, v, 0));
    EXPECT_EQ(v, 0x03);  // RxCRCEn=0, RxFraming=0b11

    EXPECT_TRUE(unit->readTypeBReg(v));
    EXPECT_EQ(v, 0x10);  // EOFSOFWidth=1

    // Back to NFC-A
    EXPECT_TRUE(unit->configureNFCMode(m5::nfc::NFC::A));

    EXPECT_TRUE(unit->readRegister8(MODE_REG, v, 0));
    EXPECT_EQ(v & 0x03, 0x01);  // CRCPreset=0b01 (CRC_A 0x6363 preset)

    EXPECT_TRUE(unit->readRegister8(TX_MODE_REG, v, 0));
    EXPECT_EQ(v, 0x00);  // TxCRCEn=0 (SW CRC by M5Unit-NFC), TxFraming=00 (Type A)

    EXPECT_TRUE(unit->readRegister8(RX_MODE_REG, v, 0));
    EXPECT_EQ(v, 0x00);  // RxCRCEn=0, RxFraming=00 (Type A)

    EXPECT_TRUE(unit->readRegister8(TX_ASK_REG, v, 0));
    EXPECT_EQ(v, 0x40);  // Force100ASK=1 (required for ISO/IEC 14443-3 Type A)
}

TEST_F(TestWS1850S, NFCBRegisterRW)
{
    SCOPED_TRACE(ustr);

    // CWGsP (6-bit, 0x00-0x3F)
    for (uint8_t v = 0x00; v <= 0x3F; ++v) {
        SCOPED_TRACE(m5::utility::formatString("CWGsP=0x%02X", v));
        EXPECT_TRUE(unit->writeCWGsP(v));
        uint8_t r{};
        EXPECT_TRUE(unit->readCWGsP(r));
        EXPECT_EQ(r & 0x3F, v);
    }

    // ModGsP (6-bit, 0x00-0x3F)
    for (uint8_t v = 0x00; v <= 0x3F; ++v) {
        SCOPED_TRACE(m5::utility::formatString("ModGsP=0x%02X", v));
        EXPECT_TRUE(unit->writeModGsP(v));
        uint8_t r{};
        EXPECT_TRUE(unit->readModGsP(r));
        EXPECT_EQ(r & 0x3F, v);
    }

    // RxThreshold round-trip is NOT testable on WS1850S: the chip silently ignores
    // writes to 0x18 regardless of mode/antenna state (verified by 0x00-0xFF scan).
    // The clamp test below still verifies bit[3] RFU validation in writeRxThreshold.

    // TypeBReg (bit[5] is RFU per PN512 8.2.2.15). Skip values with bit[5] set.
    constexpr uint8_t tb_table[] = {0x00, 0x10, 0x13, 0x40, 0x50, 0xC3};
    for (auto v : tb_table) {
        SCOPED_TRACE(m5::utility::formatString("TypeBReg=0x%02X", v));
        EXPECT_TRUE(unit->writeTypeBReg(v));
        uint8_t r{};
        EXPECT_TRUE(unit->readTypeBReg(r));
        EXPECT_EQ(r, v);
    }
}

TEST_F(TestWS1850S, NFCBRegisterClamp)
{
    SCOPED_TRACE(ustr);

    // Out-of-range values must return false
    EXPECT_FALSE(unit->writeCWGsP(0x40));
    EXPECT_FALSE(unit->writeCWGsP(0xFF));
    EXPECT_FALSE(unit->writeModGsP(0x40));
    EXPECT_FALSE(unit->writeModGsP(0xFF));
    EXPECT_FALSE(unit->writeRxThreshold(0x08));  // bit[3] RFU
    EXPECT_FALSE(unit->writeRxThreshold(0x88));
    EXPECT_FALSE(unit->writeTypeBReg(0x20));  // bit[5] RFU (per PN512 datasheet)
    EXPECT_FALSE(unit->writeTypeBReg(0xFF));
}

TEST_F(TestWS1850S, NFCBAskDepthTable)
{
    SCOPED_TRACE(ustr);

    // depth -> expected ModGsP value (same as MOD_GSP_TABLE in unit_WS1850S.cpp)
    constexpr uint8_t expected[16] = {
        0x3F, 0x3A, 0x35, 0x30, 0x2C, 0x28, 0x24, 0x20, 0x1E, 0x1C, 0x1B, 0x1A, 0x18, 0x16, 0x14, 0x10,
    };

    for (uint8_t depth = 0; depth < 16; ++depth) {
        SCOPED_TRACE(m5::utility::formatString("depth=%u", depth));
        auto cfg           = unit->config();
        cfg.nfcb_ask_depth = depth;
        unit->config(cfg);

        EXPECT_TRUE(unit->configureNFCMode(m5::nfc::NFC::B));

        uint8_t v{};
        EXPECT_TRUE(unit->readModGsP(v));
        EXPECT_EQ(v & 0x3F, expected[depth]);
    }
}
