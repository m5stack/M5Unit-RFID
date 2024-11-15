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
#include <chrono>
#include <cmath>
#include <random>
#include <array>

namespace {
}  // namespace

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::mfrc522;
using namespace m5::unit::mfrc522::command;
using namespace m5::rfid;
using namespace m5::rfid::mifare;
using namespace m5::rfid::mifare::classic;

const ::testing::Environment* global_fixture = ::testing::AddGlobalTestEnvironment(new GlobalFixture<400000U>());

class TestWS1850S : public ComponentTestBase<UnitWS1850S, bool> {
protected:
    virtual UnitWS1850S* get_instance() override
    {
        return new m5::unit::UnitWS1850S();
    }
    virtual bool is_using_hal() const override
    {
        return GetParam();
    };
};

// INSTANTIATE_TEST_SUITE_P(ParamValues, TestWS1850S,
//                          ::testing::Values(false, true));
//   INSTANTIATE_TEST_SUITE_P(ParamValues, TestWS1850S,
//   ::testing::Values(true));
INSTANTIATE_TEST_SUITE_P(ParamValues, TestWS1850S, ::testing::Values(false));

using namespace m5::unit::mfrc522;

namespace {
auto rng = std::default_random_engine{};
}

TEST_P(TestWS1850S, selfTest)
{
    SCOPED_TRACE(ustr);

    EXPECT_FALSE(unit->selfTest());  // WS1850S failed always
}

TEST_P(TestWS1850S, coporcessorCRC)
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

TEST_P(TestWS1850S, Antenna)
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

TEST_P(TestWS1850S, Tprescale)
{
    SCOPED_TRACE(ustr);

    // Raw I/O
    uint32_t cnt{32};
    while (cnt--) {
        uint16_t tps{};
        tps = rng() & 0x0FFF;  // 12 bits
        EXPECT_TRUE(unit->writeTPrescale(tps));

        uint16_t v{};
        EXPECT_TRUE(unit->readTPrescale(v));

        EXPECT_EQ(tps, v);
    }

    // float
    // Internal calculation makes uint16_t, so random input values and reverse calculation often do not match
    cnt = 32;
    while (cnt--) {
        uint16_t tps{};
        tps = rng() & 0x0FFF;  // 12 bits
        // M5_LOGI("TPS:%u/%04X", tps, tps);
        EXPECT_TRUE(unit->writeTPrescale(tps));

        // Therefore, test based on the raw values set
        float v{}, v2{};
        EXPECT_TRUE(unit->readTPrescale(v));
        EXPECT_TRUE(unit->writeTPrescale(v));
        EXPECT_TRUE(unit->readTPrescale(v2));

        EXPECT_FLOAT_EQ(v, v2);
    }
}

TEST_P(TestWS1850S, Power)
{
    // TODO
#if 0
    SCOPED_TRACE(ustr);

    uint8_t now{}, prev{};

    EXPECT_TRUE(
        unit->readRegister8(COMMAND_REG, prev, 0));
    M5_LOGW("prev:%x", prev);

    // power down
    EXPECT_TRUE(unit->enablePowerDownMode());

    EXPECT_TRUE(
        unit->readRegister8(COMMAND_REG, now, 0));
    EXPECT_EQ((now & 0x10), 0x10);
    EXPECT_NE(now, prev);
    prev = now;

    // powerup
    EXPECT_TRUE(unit->disablePowerDownMode());

    EXPECT_TRUE(
        unit->readRegister8(COMMAND_REG, now, 0));
    EXPECT_EQ((now & 0x10), 0x00);
    EXPECT_NE(now, prev);

    M5_LOGW("now:%x", now);
#endif
}

TEST_P(TestWS1850S, AccessBit)
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
