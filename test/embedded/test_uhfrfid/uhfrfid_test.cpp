/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitJRD4035 (on-board)
  @note UHF tags are not required; every test here is reproducible without a tag
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <M5UnitUnifiedRFID.hpp>
#include <M5Utility.hpp>
#include <googletest/test_template.hpp>
#include <wiring/m5_unit_unified_wiring.hpp>

using namespace m5::unit::googletest;
using namespace m5::unit;

class TestUnitJRD4035 : public UARTComponentTestBase<UnitJRD4035> {
protected:
    virtual UnitJRD4035* get_instance() override
    {
        auto ptr = new m5::unit::UnitJRD4035();
        if (ptr) {
            auto cfg = ptr->config();
            ptr->config(cfg);
        }
        return ptr;
    }

    virtual HardwareSerial* init_serial() override
    {
        // begin() (override below) drives the actual setup via the addUART wiring helper.
        // Release any previously installed UART driver here so its serial.begin() starts clean.
        auto& s = m5::unit::wiring::defaultUartSerial();
        s.end();
        return &s;
    }

    virtual bool begin() override
    {
        serial = init_serial();
        if (!serial) {
            return false;
        }
        // Unit UHF-RFID is a UART unit; PortC is preferred and PortA is the fallback
        return m5::unit::wiring::addUART(Units, *unit, 115200) && Units.begin();
    }
};

TEST_F(TestUnitJRD4035, ModuleInformation)
{
    SCOPED_TRACE(ustr);

    m5::uhf::ModuleInformation info{};
    EXPECT_TRUE(unit->readModuleInformation(info));
    EXPECT_FALSE(info.hardware_version.empty());
    EXPECT_FALSE(info.software_version.empty());
    M5_LOGI("HW:%s SW:%s MFR:%s", info.hardware_version.c_str(), info.software_version.c_str(),
            info.manufacturer.c_str());
}

TEST_F(TestUnitJRD4035, TransmitPower)
{
    SCOPED_TRACE(ustr);

    int16_t original{};
    if (!unit->readTransmitPower(original)) {
        EXPECT_TRUE(false) << "Failed to readTransmitPower";
        return;
    }
    M5_LOGI("TxPower: %d", original);
    EXPECT_GE(original, 1800);  // 18.00 dBm
    EXPECT_LE(original, 2600);  // 26.00 dBm

    EXPECT_TRUE(unit->writeTransmitPower(2000));
    int16_t readback{};
    EXPECT_TRUE(unit->readTransmitPower(readback));
    EXPECT_EQ(readback, 2000);

    EXPECT_TRUE(unit->writeTransmitPower(original));
}

TEST_F(TestUnitJRD4035, ReadRegionAndChannel)
{
    SCOPED_TRACE(ustr);

    // The library never writes the region unless asked, so this only reads the factory setting
    m5::uhf::Region region{};
    EXPECT_TRUE(unit->readRegion(region));
    M5_LOGI("Region code: %u", static_cast<uint8_t>(region));

    uint8_t channel{};
    EXPECT_TRUE(unit->readChannel(channel));
    M5_LOGI("Channel: %u", channel);
}

TEST_F(TestUnitJRD4035, PollingEmitsFramesWithoutTag)
{
    SCOPED_TRACE(ustr);

    // While polling, the module answers every round even when no tag is present (Inventory Fail
    // 0x15), so frames keep flowing. A no-tag notification is consumed by route_frame() and never
    // reaches the tag queue, so available() would stay 0 the whole time; lastFrameAt() is used
    // instead to observe that frames are actually arriving.
    if (!unit->startPolling(1000)) {
        EXPECT_TRUE(false) << "Failed to startPolling";
        return;
    }

    const unsigned long began_at  = m5::utility::millis();
    unsigned long last_frame_seen = unit->lastFrameAt();
    int frame_arrival_updates{};
    while (m5::utility::millis() - began_at < 1500) {
        unit->update();
        const unsigned long now = unit->lastFrameAt();
        if (now != last_frame_seen) {
            last_frame_seen = now;
            ++frame_arrival_updates;
        }
        m5::utility::delay(1);
    }
    M5_LOGI("Observed %d frame-arrival updates in 1500 ms", frame_arrival_updates);
    EXPECT_GT(frame_arrival_updates, 0) << "No frames observed while polling without a tag";

    EXPECT_TRUE(unit->stopPolling());
}

TEST_F(TestUnitJRD4035, PollingIsRenewedOnATimer)
{
    SCOPED_TRACE(ustr);

    // A count this small is exhausted in a fraction of the renewal interval, so the stream can
    // only keep going if update() reissues the command on its timer. As above, lastFrameAt()
    // stands in for available() since no-tag notifications never reach the tag queue.
    if (!unit->startPolling(10)) {
        EXPECT_TRUE(false) << "Failed to startPolling";
        return;
    }

    const unsigned long began_at  = m5::utility::millis();
    unsigned long last_frame_seen = unit->lastFrameAt();
    int bursts{};
    unsigned long longest_gap{};
    unsigned long previous_arrival = m5::utility::millis();
    while (m5::utility::millis() - began_at < 3000) {
        unit->update();
        const unsigned long now = unit->lastFrameAt();
        if (now != last_frame_seen) {
            const unsigned long gap = m5::utility::millis() - previous_arrival;
            if (gap > 100) {
                ++bursts;  // The stream resumed after the count ran out
                if (gap > longest_gap) {
                    longest_gap = gap;
                }
            }
            last_frame_seen  = now;
            previous_arrival = m5::utility::millis();
        }
        m5::utility::delay(1);
    }
    M5_LOGI("Observed %d resumptions, longest gap %lu ms", bursts, longest_gap);
    EXPECT_GE(bursts, 1) << "Polling was not renewed after the count was exhausted";

    EXPECT_TRUE(unit->stopPolling());
}

TEST_F(TestUnitJRD4035, ScanChannels)
{
    SCOPED_TRACE(ustr);

    // Both scans sweep the channels of the current region and report one signed level each
    m5::uhf::ChannelLevels blocking{};
    if (unit->readBlockingSignal(blocking)) {
        EXPECT_FALSE(blocking.dbm.empty());
        M5_LOGI("Blocking: %u channels from %u", (unsigned)blocking.dbm.size(), blocking.first_channel);
    } else {
        EXPECT_TRUE(false) << "Failed to readBlockingSignal";
    }

    m5::uhf::ChannelLevels rssi{};
    if (unit->readChannelRSSI(rssi)) {
        EXPECT_FALSE(rssi.dbm.empty());
        M5_LOGI("RSSI: %u channels from %u", (unsigned)rssi.dbm.size(), rssi.first_channel);
    } else {
        EXPECT_TRUE(false) << "Failed to readChannelRSSI";
    }

    // The two scans cover the same channels of the same region
    EXPECT_EQ(blocking.first_channel, rssi.first_channel);
    EXPECT_EQ(blocking.dbm.size(), rssi.dbm.size());
}

TEST_F(TestUnitJRD4035, RejectSettingsWhilePolling)
{
    SCOPED_TRACE(ustr);

    if (!unit->startPolling(1000)) {
        EXPECT_TRUE(false) << "Failed to startPolling";
        return;
    }

    // The module answers reader settings unreliably while it is running inventory rounds, so
    // they are refused outright instead of failing later with a timeout
    int16_t dbm100{};
    EXPECT_FALSE(unit->readTransmitPower(dbm100));
    m5::uhf::ModuleInformation info{};
    EXPECT_FALSE(unit->readModuleInformation(info));
    uint8_t channel{};
    EXPECT_FALSE(unit->readChannel(channel));

    // A scan cannot share the antenna with an inventory round either
    m5::uhf::ChannelLevels levels{};
    EXPECT_FALSE(unit->readChannelRSSI(levels));
    EXPECT_FALSE(unit->readBlockingSignal(levels));

    EXPECT_TRUE(unit->stopPolling());
}
