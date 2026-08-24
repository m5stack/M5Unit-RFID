/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Read and tune the reader settings of Unit UHF-RFID (U107)
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include <string>
#include <vector>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitUHFRFID unit{};

//! @brief Bring the display and the unit up, or stop
void begin_unit()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }
    // Unit UHF-RFID is a UART unit; PortC is preferred and PortA is the fallback
    if (!(m5::unit::wiring::addUART(Units, unit, 115200) && Units.begin())) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    m5::uhf::ModuleInformation info{};
    if (unit.readModuleInformation(info)) {
        M5_LOGI("HW:%s SW:%s MFR:%s", info.hardware_version.c_str(), info.software_version.c_str(),
                info.manufacturer.c_str());
    }
}

/*
  Reader settings tuned for a tag resting on the antenna.

  A reader as it ships reaches about a metre and a half. At contact the reply overwhelms the
  receiver: most inventory rounds find nothing and about half of every read fails, however strong
  the reply is, so RSSI is no guide to what is wrong. Bringing the working distance in fixes it,
  either by lowering the transmit power or by lowering the receiver gain. Lowering the gain is the
  better of the two, since the transmit power is what energises the tag in the first place.

  WARNING: the module keeps these settings when it loses power, so running this sketch changes the
  unit until something changes it back. What one unit was found set to is spelled out below;
  assign those to the three constants in use to put it back the way it was.

  Those are not the values the chip's own documentation calls the defaults, which are a mixer gain
  of 9dB and a threshold of 0x01B0. That unit came set for distance rather than for stability, so
  read what is there before assuming what it was.
*/
namespace as_shipped {
constexpr int16_t TX_POWER_DBM100{2600};
constexpr m5::unit::m100::MixerGain MIXER_GAIN{m5::unit::m100::MixerGain::dB6};
constexpr uint16_t THRESHOLD{0x00B0};
}  // namespace as_shipped

//! @brief Transmit power in 1/100 dBm. The module accepts 1700 to 2600
constexpr int16_t TX_POWER_DBM100{2600};
//! @brief Demodulation threshold. 0x01B0 is the lowest the chip documents as worth using
constexpr uint16_t DEMODULATOR_THRESHOLD{0x01B0};
//! @brief Mixer gain, a step below what one unit was found shipped with
constexpr m5::unit::m100::MixerGain DEMODULATOR_MIXER_GAIN{m5::unit::m100::MixerGain::dB3};

const char* region_to_string(const m5::uhf::Region r)
{
    switch (r) {
        case m5::uhf::Region::China900MHz:
            return "China900MHz";
        case m5::uhf::Region::America:
            return "America";
        case m5::uhf::Region::Europe:
            return "Europe";
        case m5::uhf::Region::China800MHz:
            return "China800MHz";
        case m5::uhf::Region::SouthKorea:
            return "SouthKorea";
        default:
            return "Unspecified";
    }
}

const char* filter_to_string(const m5::uhf::SelectFilter f)
{
    switch (f) {
        case m5::uhf::SelectFilter::NotSelected:
            return "NotSelected";
        case m5::uhf::SelectFilter::Selected:
            return "Selected";
        default:
            return "All";
    }
}

void print_channel_levels(const char* what, const m5::uhf::ChannelLevels& levels)
{
    std::string s{};
    for (size_t i = 0; i < levels.dbm.size(); ++i) {
        s += m5::utility::formatString("%u:%d ", (unsigned)(levels.first_channel + i), levels.dbm[i]);
    }
    M5_LOGI("%s per channel (dBm): %s", what, s.c_str());
}

//! @brief Survey the band. Take the tag out of the field first: a tag reflects the carrier back
//! and shows up in the blocking scan as if it were interference
void survey_band()
{
    m5::uhf::ChannelLevels levels{};
    if (unit.readBlockingSignal(levels)) {
        print_channel_levels("Blocking", levels);
    }
    if (unit.readChannelRSSI(levels)) {
        print_channel_levels("RSSI", levels);
    }
}

void report_settings()
{
    int16_t dbm100{};
    if (unit.readTransmitPower(dbm100)) {
        M5_LOGI("TxPower: %d.%02d dBm", dbm100 / 100, dbm100 % 100);
    }
    m5::uhf::Region region{};
    if (unit.readRegion(region)) {
        M5_LOGI("Region: %s", region_to_string(region));
    }
    uint8_t channel{};
    if (unit.readChannel(channel)) {
        M5_LOGI("Channel: %u", channel);
    }
    // Which tags an inventory round invites. A select mask aimed at SL does nothing while this
    // reads All, which is what the reader ships set to
    m5::uhf::QueryParameters qp{};
    if (unit.readQueryParameters(qp)) {
        M5_LOGI("Query: Q=%u session=S%u target=%c sel=%s", qp.q, (unsigned)qp.session,
                qp.target == m5::uhf::Target::A ? 'A' : 'B', filter_to_string(qp.filter));
    }
    // How far the reader listens, as against how far it reaches
    m5::unit::m100::DemodulatorParameters dp{};
    if (unit.readDemodulatorParameters(dp)) {
        M5_LOGI("Demodulator: mixer=%udB if=%udB threshold=0x%04X", m5::unit::m100::mixerGainDb(dp.mixer_gain),
                m5::unit::m100::ifGainDb(dp.if_gain), dp.threshold);
    }
}
}  // namespace

void setup()
{
    begin_unit();

    M5_LOGI("--- as found ---");
    report_settings();

    if (!unit.writeTransmitPower(TX_POWER_DBM100)) {
        M5_LOGE("Failed to set the transmit power");
    }
    m5::unit::m100::DemodulatorParameters dp{};
    if (unit.readDemodulatorParameters(dp)) {
        dp.mixer_gain = DEMODULATOR_MIXER_GAIN;
        dp.threshold  = DEMODULATOR_THRESHOLD;
        if (!unit.writeDemodulatorParameters(dp)) {
            M5_LOGE("Failed to set the demodulator parameters");
        }
    }

    M5_LOGI("--- now ---");
    report_settings();
    M5_LOGW("These settings survive a power cycle. As shipped: mixer=%udB threshold=0x%04X, power %d.%02ddBm",
            m5::unit::m100::mixerGainDb(as_shipped::MIXER_GAIN), as_shipped::THRESHOLD,
            as_shipped::TX_POWER_DBM100 / 100, as_shipped::TX_POWER_DBM100 % 100);

    survey_band();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: survey the band");
    lcd.println("(take the tag away first)");
}

void loop()
{
    M5.update();
    Units.update();

    // Worth repeating: a tag that will not read while the antenna and the power are both fine is
    // usually drowned out by something on the air, and these two scans tell that apart from a
    // problem with the tag
    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        lcd.println("surveying...");
        survey_band();
    }
}

#if !defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#if CONFIG_FREERTOS_UNICORE
static inline void feedIdleTaskPeriodically(void)
{
    constexpr uint32_t FEED_INTERVAL_MS   = 2000;
    constexpr TickType_t FEED_SLEEP_TICKS = pdMS_TO_TICKS(5);
    static uint32_t s_next_feed_ms        = 0;
    const uint32_t now_ms                 = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (now_ms >= s_next_feed_ms) {
        s_next_feed_ms = now_ms + FEED_INTERVAL_MS;
        vTaskDelay(FEED_SLEEP_TICKS);
    }
}
#endif

extern "C" void app_main(void)
{
    setup();
    for (;;) {
#if CONFIG_FREERTOS_UNICORE
        feedIdleTaskPeriodically();
#endif
        loop();
    }
}
#endif
