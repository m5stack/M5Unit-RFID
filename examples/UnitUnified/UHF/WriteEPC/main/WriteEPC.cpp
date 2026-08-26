/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Give a UHF-RFID tag an EPC of its own with Unit UHF-RFID (U107)

  Tags arrive from the factory sharing one EPC. Detection reports one entry per EPC, so a pile
  of them looks like a single tag, and a selection built from that EPC picks out every one of
  them at once: reads and writes then collide and are reported as a tag that did not answer.
  Giving each tag an EPC of its own is what makes all of that behave.

  The EPC written here is the first twelve bytes of the tag's own TID, which hold the class
  identifier, the mask designer, the model and the chip's factory serial. That is unique per
  tag without anything having to be counted or remembered, and running this twice over the same
  tag writes the same value, so the second run finds nothing to do.

  @warning This is not a GS1 EPC scheme. It is an identifier for a bench full of blank tags, not
  something to put on goods
  @warning The EPC that was there is gone once this has run. It is printed first, which is the
  only chance to write it down. Holding the button shows the same report and stops before
  writing anything, which is the way to check two tags really do differ before replacing either
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include <vector>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitUHFRFID unit{};
m5::uhf::UHFLayer uhf{unit};

//! @brief Words of TID that make up the new EPC, which is as long as the 96-bit EPC it replaces
constexpr uint16_t TID_WORDS{6};
//! @brief Word address the EPC starts at, after the stored CRC and the PC (Gen2 v2.1 6.3.2.1.2)
constexpr uint16_t EPC_FIRST_WORD{2};

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

    // The module keeps these across a power cycle, so what it holds now is whatever was last
    // written to it rather than a default. They decide how far a tag can be and how faint an
    // answer still counts, which is the difference between a tag that reads and one that does
    // not. What is there is reported, not changed: how much power may be radiated is a question
    // of where the unit is being used
    int16_t dbm100{};
    if (unit.readTransmitPower(dbm100)) {
        M5_LOGI("Transmit power: %d.%02d dBm", dbm100 / 100, dbm100 % 100);
    }
    m5::unit::m100::DemodulatorParameters dp{};
    if (unit.readDemodulatorParameters(dp)) {
        M5_LOGI("Demodulator: mixer=%udB if=%udB threshold=0x%04X", m5::unit::m100::mixerGainDb(dp.mixer_gain),
                m5::unit::m100::ifGainDb(dp.if_gain), dp.threshold);
    }
}

//! @brief The one tag in the field, or none
//! @warning Tags that share an EPC are reported as one, which is the very thing this example is
//! for. Present them one at a time until they have EPCs of their own
bool detect_one(m5::uhf::Tag& tag)
{
    std::vector<m5::uhf::Tag> tags{};
    if (!uhf.detect(tags) || tags.empty()) {
        M5_LOGI("No tag in the field");
        lcd.println("no tag");
        return false;
    }
    if (tags.size() > 1) {
        M5_LOGW("%zu tags in the field; leave one of them", tags.size());
        lcd.printf("%zu tags: leave one\n", tags.size());
        return false;
    }
    tag = tags.front();
    return true;
}

//! @brief Print bytes as hex on one line
void print_bytes(const char* what, const uint8_t* data, const size_t len)
{
    std::string text{};
    for (size_t i = 0; i < len; ++i) {
        char pair[3]{};
        snprintf(pair, sizeof(pair), "%02X", data[i]);
        text += pair;
    }
    M5_LOGI("%s: %s", what, text.c_str());
}

//! @brief Read the TID words that become the new EPC
//! @details The first two words name the chip and the three after the XTID header hold the
//! factory serial (TDS 16.1 and 16.2). They are read straight rather than through identify(),
//! because a chip whose XTID header says there is no serial can still carry one: UCODE G2iM
//! keeps a permalocked 48-bit serial at TID 30h to 5Fh with an XTID header of zero
bool read_tid_for_epc(std::vector<uint8_t>& tid)
{
    if (!uhf.readBank(tid, m5::uhf::Bank::Tid, 0, TID_WORDS) || tid.size() != TID_WORDS * 2) {
        M5_LOGE("Could not read %u words of TID", TID_WORDS);
        lcd.println("TID: unreadable");
        return false;
    }
    return true;
}

//! @brief Give the tag the EPC its own TID spells out
//! @param detected The tag to work on
//! @param dry_run Show what would be written and stop there. Nothing about the tag is changed
void write_own_epc(const m5::uhf::Tag& detected, const bool dry_run)
{
    if (!uhf.select(detected)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return;
    }
    m5::uhf::Tag tag = detected;
    if (uhf.identify(tag)) {
        M5_LOGI("Chip: %s", tag.chipAsString().c_str());
        lcd.printf("%s\n", tag.chipAsString().c_str());
    }

    std::vector<uint8_t> tid{};
    if (!read_tid_for_epc(tid)) {
        uhf.deselect();
        return;
    }
    print_bytes("EPC now", detected.epc.begin(), detected.epc.size);
    print_bytes("TID", tid.data(), tid.size());
    print_bytes("EPC to write", tid.data(), tid.size());
    // The same test the layer applies to a TID mask: everything before word 3 names the chip,
    // so a TID with nothing past it is the same on every tag of this kind and is no use either
    // as an EPC or as a way of addressing the tag while the EPC is being replaced
    if (!m5::uhf::tidTellsTagsApart(tid.data(), tid.size())) {
        M5_LOGE("This chip carries no serial in its TID; there is nothing unique to write");
        lcd.println("no serial in TID");
        uhf.deselect();
        return;
    }

    if (detected.epc.size == tid.size() && memcmp(detected.epc.begin(), tid.data(), tid.size()) == 0) {
        M5_LOGI("The tag already carries this EPC; nothing to do");
        lcd.println("already done");
        uhf.deselect();
        return;
    }
    if (detected.epc.size != tid.size()) {
        // Writing a different length would mean changing the PC as well, which is more than
        // this example takes on
        M5_LOGE("The tag holds a %u-byte EPC, and this writes %zu", detected.epc.size, tid.size());
        lcd.println("EPC length differs");
        uhf.deselect();
        return;
    }

    if (dry_run) {
        M5_LOGI("Nothing was written; the tag is as it was");
        lcd.println("(not written)");
        uhf.deselect();
        return;
    }

    // From here the tag is addressed by its TID. A mask built from the EPC matches on the very
    // bytes about to be replaced, so it stops picking this tag out half way through the write;
    // the TID is fixed at manufacture and goes on matching whatever happens to the EPC
    m5::uhf::Tid mask{};
    if (!mask.assign(tid.data(), tid.size()) || !uhf.select(mask)) {
        M5_LOGE("Failed to address the tag by its TID");
        lcd.println("TID select: failed");
        uhf.deselect();
        return;
    }

    const bool wrote = uhf.writeBank(m5::uhf::Bank::Epc, EPC_FIRST_WORD, tid.data(), static_cast<uint16_t>(tid.size()));
    if (!wrote) {
        // A failure part way through leaves the tag holding the front of the new EPC and the
        // back of the old one. Saying it kept the old one would be a guess
        M5_LOGE("Failed to write the EPC");
        lcd.println("write: failed");
    }

    // The TID mask still holds, so what the tag ended up with can be read straight back
    std::vector<uint8_t> back{};
    if (!uhf.readBank(back, m5::uhf::Bank::Epc, EPC_FIRST_WORD, TID_WORDS)) {
        M5_LOGE("Could not read the EPC back; present the tag again to see what it holds");
        lcd.println("check it again");
        uhf.deselect();
        return;
    }
    print_bytes("EPC after", back.data(), back.size());
    uhf.deselect();

    const bool took = back.size() == tid.size() && memcmp(back.data(), tid.data(), tid.size()) == 0;
    M5_LOGI("Writing the EPC: %s", took ? "done" : "the tag holds something else");
    lcd.println(took ? "written" : "MISMATCH");
}
}  // namespace

void setup()
{
    begin_unit();

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: write an EPC");
    lcd.println("A hold: show it only");
    lcd.println("(taken from the TID)");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked() || M5.BtnA.wasHold()) {
        // Holding shows what a click would do. An EPC cannot be put back once it is replaced,
        // so being able to look first is worth a button of its own
        const bool dry_run = M5.BtnA.wasHold();
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        m5::uhf::Tag detected{};
        if (detect_one(detected)) {
            write_own_epc(detected, dry_run);
        }
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
