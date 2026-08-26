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
    // Notifications keep coming while the module is polling, and the driver's default 256-byte
    // receive buffer holds only about 22ms of them at this baud rate. Anything that keeps the
    // sketch busy for longer than that, printing included, costs bytes out of the middle of a
    // frame. The port has to be closed for a new size to be accepted
#if defined(ARDUINO)
    constexpr size_t RX_BUFFER_BYTES{2048};
    auto& serial = m5::unit::wiring::defaultUartSerial();
    serial.end();
    // The call answers with the size it settled on, and with zero when it would not take. There
    // is no way to ask afterwards, so this is the only chance to find out
    if (serial.setRxBufferSize(RX_BUFFER_BYTES) != RX_BUFFER_BYTES) {
        M5_LOGW("The receive buffer kept its default size; frames may arrive in pieces");
    }
#endif
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
  Three sets of reader settings, and a button to move between them.

  A reader as it ships reaches about a metre and a half. At contact the reply overwhelms the
  receiver: most inventory rounds find nothing and about half of every read fails, however strong
  the reply is, so RSSI is no guide to what is wrong. Bringing the working distance in fixes it,
  and lowering the receiver gain is the better way of doing that, since the transmit power is what
  energises the tag in the first place.

  The three differ in the mixer gain and the demodulation threshold and in nothing else, which is
  what makes them worth comparing.

  WARNING: the module keeps these settings when it loses power. Nothing is written until the button
  is pressed, so starting this sketch leaves a unit alone, but pressing it does not. What a unit
  was set to before that is only recoverable because one of the three says what this product was
  found shipped with.
*/
struct Preset {
    const char* name;
    int16_t dbm100;
    m5::unit::m100::MixerGain mixer_gain;
    m5::unit::m100::IFGain if_gain;
    uint16_t threshold;
};

const Preset PRESETS[] = {
    // What the chip's own documentation gives as the defaults
    {"documented", 2600, m5::unit::m100::MixerGain::dB9, m5::unit::m100::IFGain::dB36, 0x01B0},
    // What one Unit UHF-RFID (U107) was found set to out of the box: tuned for reach, with a
    // threshold below the floor the same documentation gives. Only one unit was ever looked at,
    // so compare it against what this sketch prints at startup before trusting it
    {"as the unit shipped", 2600, m5::unit::m100::MixerGain::dB6, m5::unit::m100::IFGain::dB36, 0x00B0},
    // Measured to read a tag resting on the antenna, where the other two miss most rounds
    {"for a tag at contact", 2600, m5::unit::m100::MixerGain::dB3, m5::unit::m100::IFGain::dB36, 0x01B0},
};
size_t preset_index{};

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
    // A mask is stored against the session and the flag the round names, so these decide which
    // tag a selection ends up addressing
    m5::uhf::QueryParameters qp{};
    if (unit.readQueryParameters(qp)) {
        M5_LOGI("Query: Q=%u session=S%u target=%c filter=%u", qp.q, static_cast<uint8_t>(qp.session),
                qp.target == m5::uhf::Target::A ? 'A' : 'B', static_cast<uint8_t>(qp.filter));
    }
}

//! @brief Write one of the presets and say what the reader holds afterwards
void apply(const Preset& preset)
{
    M5_LOGI("--- applying: %s ---", preset.name);
    if (!unit.writeTransmitPower(preset.dbm100)) {
        M5_LOGE("Failed to set the transmit power");
    }
    m5::unit::m100::DemodulatorParameters dp{};
    dp.mixer_gain = preset.mixer_gain;
    dp.if_gain    = preset.if_gain;
    dp.threshold  = preset.threshold;
    if (!unit.writeDemodulatorParameters(dp)) {
        M5_LOGE("Failed to set the demodulator parameters");
    }
    // Reading back rather than reporting what was asked for: the two are not the same thing
    report_settings();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setCursor(0, 0);
    lcd.printf("%s\n", preset.name);
    lcd.printf("mixer %udB thr %04X\n", m5::unit::m100::mixerGainDb(preset.mixer_gain), preset.threshold);
    lcd.printf("%d.%02d dBm\n\n", preset.dbm100 / 100, preset.dbm100 % 100);
    lcd.println("A: next preset");
}
}  // namespace

void setup()
{
    begin_unit();

    // Read only. A unit that has never been written to still holds whatever it shipped with, and
    // that is worth seeing before anything replaces it
    M5_LOGI("--- as found ---");
    report_settings();
    survey_band();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("as found");
    lcd.println("(nothing written yet)");
    lcd.println("");
    lcd.println("A: next preset");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        apply(PRESETS[preset_index]);
        preset_index = (preset_index + 1) % (sizeof(PRESETS) / sizeof(PRESETS[0]));
        // The band looks different once the transmit power or the gain moves, and a tag left in
        // the field reflects the carrier back and reads as interference
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
