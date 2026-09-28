/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for the UHF-RFID polling state machine

  @note The state machine lives in UHFRFIDComponent and reaches the module only through
  start_polling_command() / stop_polling_command(). A stand-in for those is all the host needs,
  so what a failed stop leaves behind can be checked here rather than on a board, where a stop
  cannot be made to fail on purpose.
*/
#include <gtest/gtest.h>
#include <unit/unit_UHFRFID.hpp>
#include <chrono>
#include <thread>

namespace {

using namespace m5::unit;

// Answers every command with a refusal, except the two the state machine turns on, which the
// test drives. Counting the calls is what shows whether the renewal timer fired
class FakeUHFRFID : public UHFRFIDComponent {
public:
    FakeUHFRFID() : UHFRFIDComponent()
    {
    }

    bool start_succeeds{true};
    bool stop_succeeds{true};
    int start_calls{};
    int stop_calls{};

    // reject_while_polling() is what every reader setting asks first, so this is the guard the
    // caller actually feels
    bool settingsRefused() const
    {
        return reject_while_polling("test");
    }

protected:
    // Component identity. Nothing here is registered with M5UnitUnified, so any answer will do
    const char* unit_device_name() const override
    {
        return "FakeUHFRFID";
    }
    m5::unit::types::uid_t unit_identifier() const override
    {
        return 0;
    }
    m5::unit::types::attr_t unit_attribute() const override
    {
        return 0;
    }

    bool pump(const uint32_t) override
    {
        return false;
    }
    bool start_polling_command(const uint16_t) override
    {
        ++start_calls;
        return start_succeeds;
    }

    bool readModuleInformation(m5::uhf::ModuleInformation& info) override
    {
        return false;
    }
    bool readTransmitPower(int16_t& dbm100) override
    {
        return false;
    }
    bool writeTransmitPower(const int16_t dbm100) override
    {
        return false;
    }
    bool readRegion(m5::uhf::Region& region) override
    {
        return false;
    }
    bool writeRegion(const m5::uhf::Region region) override
    {
        return false;
    }
    bool readChannel(uint8_t& index) override
    {
        return false;
    }
    bool writeChannel(const uint8_t index) override
    {
        return false;
    }
    bool writeAutomaticFrequencyHopping(const bool enable) override
    {
        return false;
    }
    bool writeContinuousCarrier(const bool enable) override
    {
        return false;
    }
    bool readQueryParameters(m5::uhf::QueryParameters& qp) override
    {
        return false;
    }
    bool writeQueryParameters(const m5::uhf::QueryParameters& qp) override
    {
        return false;
    }
    bool writeAutoSleepTime(const uint8_t minutes) override
    {
        return false;
    }
    bool writeIdle(const bool enter, const uint8_t minutes) override
    {
        return false;
    }
    bool sleep() override
    {
        return false;
    }
    bool wake() override
    {
        return false;
    }
    bool writeOperatingChannels(const std::vector<uint8_t>& channels) override
    {
        return false;
    }
    bool readBlockingSignal(m5::uhf::ChannelLevels& levels) override
    {
        return false;
    }
    bool readChannelRSSI(m5::uhf::ChannelLevels& levels) override
    {
        return false;
    }
    bool readSelectParameter(m5::uhf::SelectParameter& sp) override
    {
        return false;
    }
    bool writeSelectParameter(const m5::uhf::Bank bank, const uint32_t pointer_bits, const uint8_t* mask,
                              const size_t mask_len) override
    {
        return false;
    }
    bool writeSelectEnabled(const bool enable) override
    {
        return false;
    }
    TagResult readTagMemory(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t word_address,
                            const uint16_t word_count, const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult writeTagMemory(const m5::uhf::Bank bank, const uint16_t word_address, const uint8_t* data,
                             const size_t len, const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult lockTagMemory(const uint32_t payload, const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult killTag(const uint32_t kill_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult blockPermalock(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t block_pointer,
                             const uint8_t block_range, const uint8_t* mask, const size_t mask_len,
                             const uint32_t access_password, const bool allow_permanent) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult qtCommand(uint16_t& control, const bool write, const bool persistent,
                        const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult nxpChangeConfig(uint16_t& config, const uint16_t toggle, const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    TagResult nxpChangeEAS(const bool enable, const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    bool nxpEASAlarm(std::vector<uint8_t>& alarm) override
    {
        return false;
    }
    TagResult nxpReadProtect(const bool protect, const uint32_t access_password) override
    {
        return m5::stl::unexpected<uint8_t>{0xFFU};
    }
    m5::uhf::Reason classify(const uint8_t error_code) const override
    {
        return m5::uhf::Reason::Unsupported;
    }
    bool stop_polling_command() override
    {
        ++stop_calls;
        return stop_succeeds;
    }
};

}  // namespace

TEST(UHFState, StopSuccessClearsTheGuard)
{
    FakeUHFRFID unit{};
    EXPECT_TRUE(unit.begin());
    EXPECT_TRUE(unit.startPolling(8));
    EXPECT_TRUE(unit.inPolling());
    EXPECT_TRUE(unit.settingsRefused());

    EXPECT_TRUE(unit.stopPolling());
    EXPECT_FALSE(unit.inPolling());
    EXPECT_FALSE(unit.settingsRefused());
}

TEST(UHFState, StopFailureKeepsTheGuard)
{
    FakeUHFRFID unit{};
    EXPECT_TRUE(unit.begin());
    EXPECT_TRUE(unit.startPolling(8));

    unit.stop_succeeds = false;
    EXPECT_FALSE(unit.stopPolling());

    // The caller asked for the polling to stop, so nothing renews it
    EXPECT_FALSE(unit.inPolling());
    // The module may still be running the rounds, so the settings stay refused. Reporting the
    // reader as idle here would let a setting go out into an inventory round
    EXPECT_TRUE(unit.settingsRefused());
}

TEST(UHFState, FailedStopDoesNotLetTheTimerReissue)
{
    FakeUHFRFID unit{};
    EXPECT_TRUE(unit.begin());
    EXPECT_TRUE(unit.startPolling(8));
    const auto issued = unit.start_calls;

    unit.stop_succeeds = false;
    EXPECT_FALSE(unit.stopPolling());

    // Long enough for the renewal interval to have passed
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    unit.update();
    EXPECT_EQ(unit.start_calls, issued);
}

TEST(UHFState, TheTimerReissuesWhilePollingStands)
{
    FakeUHFRFID unit{};
    EXPECT_TRUE(unit.begin());
    EXPECT_TRUE(unit.startPolling(8));
    const auto issued = unit.start_calls;

    // Before the interval, nothing is sent again
    unit.update();
    EXPECT_EQ(unit.start_calls, issued);

    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    unit.update();
    EXPECT_EQ(unit.start_calls, issued + 1);
}

TEST(UHFState, StartFailureLeavesTheReaderIdle)
{
    FakeUHFRFID unit{};
    EXPECT_TRUE(unit.begin());

    unit.start_succeeds = false;
    EXPECT_FALSE(unit.startPolling(8));
    EXPECT_FALSE(unit.inPolling());
    EXPECT_FALSE(unit.settingsRefused());
    EXPECT_EQ(unit.stop_calls, 0);
}
