/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-RFID
  Lock one block of a tag's user memory for good with Unit UHF-RFID (U107)

  Nothing here can be undone. A block locked this way is unwritable for the life of the tag.
  Reading it is unaffected, and so is everything else: the EPC, the TID, the other blocks and
  the tag itself all go on working. What is lost is the ability to write those bits again.

  A permalocked block is unwritable from then on whatever else is done to the tag. The bank's
  own lock bits and its access password have no bearing on it (Gen2 v2.1 6.3.2.11.3), and the
  tag turns such a write away with a memory-locked error.

  One block is locked per run, chosen at random from those the tag has not locked already. An
  Impinj Monza 4QT has four of 128 bits, so the same tag can be used four times before its user
  memory is entirely unwritable, and even then it still answers, reads and carries its EPC.

  The block is drawn once at startup so that the report and the act agree on which one it is.
  Clicking reports and stops there; holding does it.

  A tag reporting a block as locked and a tag refusing to write it are two different claims, so
  both are checked. Clicking writes one word in each block and says which of them refuse; it
  locks nothing. Holding writes a word inside the block before the lock and again after, and one
  in another block to show that only the one asked for was taken.

  Those writes change what the user memory holds, and the bytes in a locked block keep whatever
  they had at that moment.

  @warning A chip whose block size is not known is refused. Locking blocks of an unknown size
  would mean not being able to say what was about to be lost
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
#include <M5Utility.h>
#include <esp_random.h>
#include <wiring/m5_unit_unified_wiring.hpp>
#include <vector>

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitUHFRFID unit{};
m5::uhf::UHFLayer uhf{unit};

//! @brief The draw this run was started with, kept so that the report and the act agree
int draw{};
//! @brief Block it lands on once the tag has been read
int chosen_block{};

//! @brief Bring the display and the unit up, or stop
void begin_unit()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }
    // Notifications keep coming while the module is polling, and the driver's default 256-byte
    // receive buffer holds only about 22ms of them at this baud rate
#if defined(ARDUINO)
    constexpr size_t RX_BUFFER_BYTES{2048};
    auto& serial = m5::unit::wiring::defaultUartSerial();
    serial.end();
    if (serial.setRxBufferSize(RX_BUFFER_BYTES) != RX_BUFFER_BYTES) {
        M5_LOGW("The receive buffer kept its default size; frames may arrive in pieces");
    }
#endif
    if (!(m5::unit::wiring::addUART(Units, unit, 115200) && Units.begin())) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());
}

//! @brief The one tag in the field, or none
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

//! @brief Word a block starts at
uint16_t first_word_of(const m5::uhf::Tag& tag, const int block)
{
    return static_cast<uint16_t>(static_cast<uint32_t>(block) * m5::uhf::chipPermalockBlockBits(tag.chip) / 16U);
}

//! @brief Write one word and say whether the bits actually changed
/*!
  @enum Takes
  @brief What became of a write to one word
  @details Three answers, not two. Saying a word refused a write when the word could not be read
  is evidence of something that was never seen, and this is what a permanent lock is judged by
 */
enum class Takes : uint8_t {
    Yes,      //!< The bits moved
    No,       //!< The bits stayed as they were
    Unknown,  //!< The word could not be read, so nothing can be said either way
};

//! @brief Spell out a Takes
const char* takes_as_string(const Takes takes)
{
    switch (takes) {
        case Takes::Yes:
            return "takes a write";
        case Takes::No:
            return "REFUSES";
        default:
            return "could not be told";
    }
}

//! @details A tag that will not write a word may say so with a memory-locked error, but a
//! reader that reports the write as done either way would look the same. So the word is read,
//! written with something it did not already hold, and read again: only the last of those says
//! whether anything moved
Takes word_takes(const uint16_t word_address)
{
    std::vector<uint8_t> before{};
    if (!uhf.readBank(before, m5::uhf::Bank::User, word_address, 1) || before.size() != 2) {
        M5_LOGW("Word %u could not be read, so there is nothing to compare against", word_address);
        return Takes::Unknown;
    }
    // Anything the word does not already hold will do, and the complement is always that
    const uint8_t word[]{static_cast<uint8_t>(~before[0]), static_cast<uint8_t>(~before[1])};
    const auto wrote = uhf.writeBank(m5::uhf::Bank::User, word_address, word, sizeof(word));
    if (!wrote) {
        // A write that never reached the tag leaves the bits where they were, which looks from
        // the outside exactly like a tag refusing one. Only the tag saying so settles it
        if (wrote.error() != m5::uhf::Reason::Locked) {
            M5_LOGW("Word %u was not written: %s. Whether it would take one is not settled by that", word_address,
                    m5::uhf::reasonAsString(wrote.error()));
            return Takes::Unknown;
        }
        return Takes::No;
    }

    std::vector<uint8_t> after{};
    if (!uhf.readBank(after, m5::uhf::Bank::User, word_address, 1) || after.size() != 2) {
        M5_LOGW("Word %u could not be read back, so the write cannot be judged", word_address);
        return Takes::Unknown;
    }
    if ((after[0] == before[0]) && (after[1] == before[1])) {
        // A tag that will not take a write says so, and that was answered for above. Bits that
        // stayed put after the reader called the write done is the two disagreeing, which
        // settles nothing either way
        M5_LOGW("Word %u: the reader called the write done but the bits stayed at %02X%02X", word_address, before[0],
                before[1]);
        return Takes::Unknown;
    }
    return Takes::Yes;
}

//! @brief Mask naming one block, the first block being the most significant bit
uint16_t mask_for(const int block)
{
    return static_cast<uint16_t>(0x8000u >> block);
}

//! @brief Read and report which blocks are already locked
//! @param[out] locked What the tag answered
//! @return True if the tag answered
bool report_locked(uint16_t& locked)
{
    std::vector<uint8_t> mask{};
    if (!uhf.readBlockPermalock(mask, m5::uhf::Bank::User) || mask.size() < 2) {
        M5_LOGE("The tag did not say which blocks it has locked");
        lcd.println("lock state: unknown");
        return false;
    }
    locked = static_cast<uint16_t>((mask[0] << 8) | mask[1]);
    M5_LOGI("Locked now: %04X", locked);
    return true;
}

//! @brief Select the tag, work out what a run would do, and report it
//! @param[in,out] tag Tag to work on, filled in by identify()
//! @param[out] locked Blocks already locked
//! @return True when a block can be locked
bool look(m5::uhf::Tag& tag, uint16_t& locked)
{
    if (!uhf.select(tag)) {
        M5_LOGE("Failed to select the tag");
        lcd.println("select: failed");
        return false;
    }
    if (!uhf.identify(tag)) {
        M5_LOGE("Could not read the tag's TID, so the chip is not known");
        lcd.println("identify: failed");
        uhf.deselect();
        return false;
    }
    M5_LOGI("Chip: %s", tag.chipAsString().c_str());

    // A Monza 4QT showing its public map has no user memory to lock, and the chip is documented
    // as ignoring BlockPermalock outright while it does
    if (m5::uhf::tagQTSupport(tag) != m5::uhf::Support::No) {
        m5::uhf::QTParameters qt{};
        const auto read = uhf.readQTParameters(qt);
        if (read) {
            M5_LOGI("QT memory map: %s", qt.public_memory ? "public" : "private");
        } else {
            M5_LOGW("The tag did not say which memory map it is showing: %s", m5::uhf::reasonAsString(read.error()));
        }
    }

    // Not the user memory divided by the block size: a Higgs-3 has 32-bit blocks and four of
    // them, which covers 128 of its bits and not the whole bank. Only the chip can say
    const uint16_t blocks     = m5::uhf::chipPermalockBlockCount(tag.chip);
    const uint32_t block_bits = m5::uhf::chipPermalockBlockBits(tag.chip);
    if (m5::uhf::tagBlockPermalockSupport(tag) == m5::uhf::Support::No) {
        // Not the same as a chip that has the command and says nothing about its blocks
        M5_LOGE("%s does not have BlockPermalock at all; nothing can be locked", tag.chipAsString().c_str());
        lcd.println("no BlockPermalock");
        uhf.deselect();
        return false;
    }
    if (blocks == 0 || block_bits == 0) {
        // Without both there is no saying which bits a lock would cover
        M5_LOGE("%s does not say how its blocks are laid out; nothing will be locked", tag.chipAsString().c_str());
        lcd.println("blocks unknown");
        uhf.deselect();
        return false;
    }
    M5_LOGI("Blocks: %u of %ubit each", blocks, block_bits);

    if (!report_locked(locked)) {
        uhf.deselect();
        return false;
    }
    // The draw is made before the chip is known, and a tag can already have blocks locked, so
    // it is spent on the ones that are still free rather than on a block at random
    std::vector<int> free_blocks{};
    for (uint16_t block = 0; block < blocks; ++block) {
        if ((locked & mask_for(block)) == 0) {
            free_blocks.push_back(static_cast<int>(block));
        }
    }
    if (free_blocks.empty()) {
        M5_LOGI("Every block is locked already; there is nothing left to do to this tag");
        lcd.println("all locked");
        uhf.deselect();
        return false;
    }
    chosen_block        = free_blocks[static_cast<size_t>(draw) % free_blocks.size()];
    const uint16_t mask = mask_for(chosen_block);
    M5_LOGI("Would lock block %d, mask %04X, words %u to %u", chosen_block, mask, first_word_of(tag, chosen_block),
            first_word_of(tag, chosen_block + 1) - 1);
    lcd.printf("block %d (%04X)\n", chosen_block, mask);
    return true;
}

//! @brief Try a word in every block and say which of them refuse it
//! @details What a tag reports as locked and what it refuses to write are two different things,
//! and only the second is what a lock is for. Writing one word per block is what tells them
//! apart, and it locks nothing
void probe_blocks(const m5::uhf::Tag& tag)
{
    const uint16_t blocks     = m5::uhf::chipPermalockBlockCount(tag.chip);
    const uint32_t block_bits = m5::uhf::chipPermalockBlockBits(tag.chip);
    for (uint16_t block = 0; block < blocks; ++block) {
        const uint16_t first = first_word_of(tag, block);
        // The last block can run out before it is full: the user memory does not have to be a
        // multiple of the block size, and what is past its end is nobody's to lock
        const uint32_t bits_here = (block + 1 == blocks) ? m5::uhf::chipPermalockLastBlockBits(tag.chip) : block_bits;
        const uint16_t last      = static_cast<uint16_t>(first + bits_here / 16U - 1);
        M5_LOGI("Block %u (words %u to %u): %s", block, first, last, takes_as_string(word_takes(first)));
    }
}

//! @brief Report what a run would do, and which blocks refuse a write
void dry_run()
{
    m5::uhf::Tag tag{};
    if (!detect_one(tag)) {
        return;
    }
    uint16_t locked{};
    if (look(tag, locked)) {
        M5_LOGI("Nothing was locked; hold the button to mean it");
        lcd.println("(not locked)");
    }
    if (!uhf.isSelected()) {
        return;
    }
    probe_blocks(tag);
    uhf.deselect();
}

//! @brief A block that is neither the one being locked nor locked already, or -1 for none
//! @details It stands in for every block that was not asked for, so it has to be one a write
//! would reach. A block the tag locked on an earlier run would refuse the write for its own
//! reasons and say nothing about the lock made here
int control_block(const m5::uhf::Tag& tag, const uint16_t locked)
{
    const uint16_t blocks = m5::uhf::chipPermalockBlockCount(tag.chip);
    for (uint16_t block = 0; block < blocks; ++block) {
        if (static_cast<int>(block) != chosen_block && (locked & mask_for(static_cast<int>(block))) == 0) {
            return static_cast<int>(block);
        }
    }
    return -1;
}

//! @brief Lock the block this run drew, and read the state back
void lock_a_block()
{
    m5::uhf::Tag tag{};
    if (!detect_one(tag)) {
        return;
    }
    uint16_t locked{};
    if (!look(tag, locked)) {
        return;
    }

    // A word inside the block and one outside it. What the lock is supposed to do is stop the
    // first from being written and leave the second alone, which is worth proving rather than
    // taking the tag's word for. A tag with nothing else left to write cannot show the second
    const uint16_t inside = first_word_of(tag, chosen_block);
    const int control     = control_block(tag, locked);
    if (control < 0) {
        M5_LOGW("Every other block is locked already, so nothing can show that this one was the only one taken");
    }
    M5_LOGI("Before locking, word %u: %s", inside, takes_as_string(word_takes(inside)));

    const uint16_t mask = mask_for(chosen_block);
    const uint8_t payload[]{static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask)};
    // The guard is what turns a request into an act; passing it is the whole point of this run
    const auto locked_block = uhf.blockPermalock(m5::uhf::Bank::User, payload, sizeof(payload), true);
    if (!locked_block) {
        // Permalocking cannot be undone, and a tag that said nothing may have done it anyway
        M5_LOGE("Failed to lock block %d: %s. The block is %s", chosen_block,
                m5::uhf::reasonAsString(locked_block.error()),
                m5::uhf::tagUnchanged(locked_block.error()) ? "not locked" : "in a state that cannot be told");
        lcd.println("lock: failed");
        uhf.deselect();
        return;
    }

    uint16_t after{};
    const bool says_locked = report_locked(after) && (after & mask) != 0;
    // A mask read the wrong way round would agree with itself and still have locked the wrong
    // block, so what the tag says and what it does are checked separately. A word that could
    // not be read proves neither, and counting it as proof would be reporting something that
    // was never seen
    const Takes inside_now  = word_takes(inside);
    const Takes control_now = (control < 0) ? Takes::Unknown : word_takes(first_word_of(tag, control));
    M5_LOGI("Block %d says locked: %s", chosen_block, says_locked ? "yes" : "no");
    M5_LOGI("Word %u: %s", inside, takes_as_string(inside_now));
    if (control >= 0) {
        M5_LOGI("Block %d: %s", control, takes_as_string(control_now));
    }

    // Every block but this one being locked already leaves nothing to show the lock was not
    // spread wider than asked, so that half is not held against it
    const bool spared = (control < 0) || control_now == Takes::Yes;
    if (inside_now == Takes::Unknown || (control >= 0 && control_now == Takes::Unknown)) {
        M5_LOGW("Block %d: the tag says %s, and a write could not be judged, so this run does not say", chosen_block,
                says_locked ? "it is locked" : "it is not locked");
        lcd.println("not judged");
    } else {
        const bool took = says_locked && inside_now == Takes::No && spared;
        M5_LOGI("Block %d: %s", chosen_block, took ? "locked for good" : "NOT what was asked for");
        lcd.println(took ? "locked for good" : "NOT as asked");
    }

    uhf.deselect();
}
}  // namespace

void setup()
{
    begin_unit();

    // Drawn once, so that the report and the act name the same block. A chip with fewer blocks
    // than this brings it into range once the tag has been identified
    draw = static_cast<int>(esp_random() % 16);

    lcd.fillScreen(TFT_DARKGREEN);
    lcd.setTextSize(lcd.width() > 320 ? 2 : 1);
    lcd.setCursor(0, 0);
    lcd.println("A: what it would lock");
    lcd.println("A hold: lock it");
    lcd.println("(cannot be undone)");
}

void loop()
{
    M5.update();
    Units.update();

    if (M5.BtnA.wasClicked()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        dry_run();
    }
    if (M5.BtnA.wasHold()) {
        lcd.fillScreen(TFT_DARKGREEN);
        lcd.setCursor(0, 0);
        lock_a_block();
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
