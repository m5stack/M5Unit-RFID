/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file uhf_layer.hpp
  @brief EPC Gen2 semantics layer for UHF-RFID reader units
*/
#ifndef M5_UNIT_RFID_UHF_UHF_LAYER_HPP
#define M5_UNIT_RFID_UHF_UHF_LAYER_HPP

#include "uhf.hpp"
#include "unit/unit_UHFRFID.hpp"

namespace m5 {
namespace uhf {

/*!
  @typedef Result
  @brief What became of an operation that was meant to change a tag
  @details Nothing on success, and on failure why it did not go through. Operations that leave
  a tag as they found it whatever happens keep returning bool: knowing why a detect found
  nothing does not change what to do next
 */
using Result = m5::stl::expected<void, Reason>;

/*!
  @class UHFLayer
  @brief EPC Gen2 semantics layer for UHF-RFID reader units
  @details Holds semantics defined by the EPC Gen2 standard, which stay the same across
  reader chips. The frame level belongs to the unit class.
  @warning A tag's write range is shorter than its read range: writing takes more power, and
  these memories erase before they program, of which erasing is the cheaper half. A tag well
  within reading distance can therefore have a word erased and not programmed, leaving it
  neither as it was nor as it was asked to be. A command that goes unanswered is not a command
  that did nothing
 */
class UHFLayer {
public:
    explicit UHFLayer(m5::unit::UHFRFIDComponent& u) : _u{u}
    {
    }
    UHFLayer(const UHFLayer&) = delete;
    UHFLayer& operator=(const UHFLayer&) = delete;

    ///@name Detection
    ///@{
    /*!
      @brief Detect tags
      @param[out] tags Detected tags, deduplicated by EPC
      @param timeout_ms How long to keep looking, in milliseconds
      @return True if at least one tag was detected
      @details The timeout is how long the field is watched, not a deadline: this always looks
      for the whole of it, because a tag that answers rarely is only found by looking again.
      Shortening it returns sooner and finds fewer tags
      @note Starts polling internally and restores the previous polling state on return
     */
    bool detect(std::vector<Tag>& tags, const uint32_t timeout_ms = 1000U);
    /*!
      @brief Detect one tag
      @param[out] tag The first tag that answers
      @param timeout_ms How long to wait before giving up, in milliseconds
      @return True if a tag was detected
      @details Returns as soon as any tag answers rather than waiting the timeout out, so it
      costs one inventory round instead of the whole window. Unlike the form taking a vector the
      timeout is a deadline rather than a duration, and the default covers the longest a tag was
      measured to take to answer
      @note Which tag answers first is not something the caller decides: with several in the
      field this is whichever one won that round. Anything that goes on to address a tag wants
      to know there is only one of them, which the form taking a vector shows and this one
      cannot
      @note Starts polling internally and restores the previous polling state on return
     */
    bool detect(Tag& tag, const uint32_t timeout_ms = 500U);
    ///@}

    ///@name Target tag
    ///@{
    /*!
      @brief Point subsequent tag operations at one tag
      @param tag Target tag, whose EPC becomes the mask
      @param access_password Access password of the tag, 0 when it has none
      @param verify Read one word back to confirm that the tag answers
      @return True if successful
      @note Polling stops here and stays stopped until deselect(), because an inventory round
      leaves the tag flagged as already counted and it then answers nothing
      @note Selecting only stores a mask in the reader, so with verify off this succeeds even
      for a tag that is not in the field and the failure surfaces at the first access
      @note The password is not checked here: an unlocked bank answers whatever it is. A wrong
      one shows up as a memory-locked error the first time a locked area is touched
      @warning Impinj M700 and M800 series chips hold one 32-bit password that answers at both
      the access and the kill address, so an access password is a kill password as well. See
      kill()
     */
    bool select(const Tag& tag, const uint32_t access_password = 0, const bool verify = true);
    //! @brief Select by EPC, for a tag the caller holds nothing else of
    bool select(const Epc& epc, const uint32_t access_password = 0, const bool verify = true);
    /*!
      @brief Select by TID
      @param tid TID of the target tag
      @param access_password Access password of the tag, 0 when it has none
      @param verify Read one word back to confirm that the tag answers
      @return True if successful
      @details The TID is locked at manufacture, so a mask built from it keeps working across
      an EPC rewrite. The tag has to have been identified first, since detection alone only
      ever reveals an EPC
      @warning The TID has to reach past the three words that name the chip and carry something
      there, and this refuses one that does not: every tag of a model holds the same bytes in
      those words, so a mask built from them alone would address all of them at once. An
      extended TID is not required, since a chip can keep a serial without announcing one
      @note Reading that far is the caller's to do. identify() stops at the two fixed words on a
      tag whose XTID header says there is no serial, so a TID from it will be refused here
     */
    bool select(const Tid& tid, const uint32_t access_password = 0, const bool verify = true);
    //! @brief Stop addressing a single tag, and put polling back the way select() found it
    bool deselect();
    //! @brief Is a tag being addressed?
    inline bool isSelected() const
    {
        return _has_selection;
    }
    //! @brief The tag currently being addressed
    inline const Tag& selectedTag() const
    {
        return _selected;
    }
    /*!
      @brief Read the selected tag's TID and fill in what it says about the tag
      @param[out] tag The selected tag, with the identification half filled in
      @return True if successful
      @details Reads the two fixed TID words first, then the XTID if the tag carries one, since
      the header is what says how long the rest is. Chip, vendor, serial length and memory sizes
      all come out of that (TDS 16.1 and 16.2)
     */
    bool identify(Tag& tag);
    ///@}

    ///@name Dump
    ///@{
    /*!
      @brief Dump every memory bank of the selected tag
      @return True when every bank was read
      @details Prints the Reserved, EPC, TID and User banks. How much of each there is comes
      from the tag where it says so and from what its chip is known to hold where it does not;
      a bank whose size is not known either way is skipped and said to be so
      @pre A tag must have been selected
     */
    bool dump();
    /*!
      @brief Dump one memory bank
      @param bank Memory bank
      @return True if successful
      @pre A tag must have been selected
     */
    bool dump(const Bank bank);
    /*!
      @brief Dump part of one memory bank
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param words Number of 16-bit words
      @return True if successful
      @pre A tag must have been selected
     */
    bool dump(const Bank bank, const uint16_t word_address, const uint16_t words);
    ///@}

    ///@name Tag memory, addressed to the selected tag
    ///@{
    /*!
      @brief Read a memory bank
      @param[out] out Bytes read
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param word_count Number of 16-bit words to read
      @return Nothing on success, or why it did not go through
      @note Addresses and lengths count words because EPC Gen2 lays tag memory out in 16-bit
      words; a byte address off a word boundary means nothing to a tag
     */
    Result readBank(std::vector<uint8_t>& out, const Bank bank, const uint16_t word_address, const uint16_t word_count);
    /*!
      @brief Read a memory bank into a buffer of the caller's own
      @param[out] out Buffer, which has to hold word_count words
      @param[in,out] out_len Its size in bytes going in, and how many bytes arrived coming out
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param word_count Number of 16-bit words to read
      @return Nothing on success, or why it did not go through
      @note A tag can answer with less than was asked for, which is what out_len reports. A
      buffer too small for word_count words is refused rather than filled part way
     */
    Result readBank(uint8_t* out, uint16_t& out_len, const Bank bank, const uint16_t word_address,
                    const uint16_t word_count);
    /*!
      @brief Write a memory bank
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param data Bytes to write
      @param data_len Their length, which has to be a whole number of 16-bit words
      @return Nothing on success, or why it did not go through
      @note A bank larger than one command can carry is written in several, so the caller does
      not have to know how much that is. A write that fails partway leaves everything before the
      failure already on the tag, and says in the log how far it got
      @note Writing the EPC bank changes the very bytes an EPC mask matches on, so a selection
      made from an EPC is dropped afterwards and has to be made again
      @warning A write reported as done is not proof the memory could be written: a write of the
      value a word already holds comes back done even on locked and permalocked memory. It is the
      reader that decides that and not the tag, which is why it stops happening where the reader
      cannot read the word back. Only a write carrying something else says anything
     */
    Result writeBank(const Bank bank, const uint16_t word_address, const uint8_t* data, const uint16_t data_len);
    /*!
      @brief Change the lock state of memory banks and passwords
      @param settings What to change; anything left out keeps the state it has
      @param allow_permanent Permit PermanentOpen and PermanentLock, neither of which can be undone
      @return Nothing on success, or why it did not go through
      @warning PermanentLock leaves an area unwritable for the life of the tag and PermanentOpen
      leaves it impossible to ever lock. There is no way back from either
      @warning Impinj M700 and M800 series chips hold one 32-bit password that answers at both
      the access and the kill address, so locking one locks the other. They refuse a Lock that
      asks for anything else, and the two have to be given the same setting
     */
    Result lock(const std::vector<LockSetting>& settings, const bool allow_permanent = false);
    /*!
      @brief Kill a tag permanently
      @param tag Tag to kill, which must be the one selected
      @param kill_password Kill password; a tag whose kill password is zero refuses to be killed
      @return Nothing on success, or why it did not go through
      @warning The tag stops answering for good. There is no way to revive it
      @warning Impinj M700 and M800 series chips hold one 32-bit password that answers at both
      the access and the kill address. Giving such a tag an access password gives it the same
      kill password, so a tag that was only meant to be protected becomes one that can be killed
     */
    Result kill(const Tag& tag, const uint32_t kill_password);
    /*!
      @brief Read which blocks of a bank the selected tag has permanently locked
      @param[out] mask One bit per block, the first block in the most significant bit
      @param bank Bank the blocks are in
      @param block_pointer First block the mask covers, in units of sixteen
      @param block_range Words of mask to ask for, each covering sixteen blocks
      @return Nothing on success, or why it did not go through
      @note How large a block is, is the chip's to decide: an Impinj Monza 4QT divides its user
      memory into four of 128 bits. chipPermalockBlockBits() is what says so where it is known
      @note Gen2 leaves BlockPermalock optional. A tag without it answers with a failure
     */
    Result readBlockPermalock(std::vector<uint8_t>& mask, const Bank bank, const uint16_t block_pointer = 0,
                              const uint8_t block_range = 1);
    /*!
      @brief Permanently lock blocks of a bank on the selected tag
      @param bank Bank the blocks are in
      @param mask One bit per block, the first block in the most significant bit. A bit set
      locks that block; a bit clear leaves it as it is
      @param mask_len Its length in bytes, which is twice block_range
      @param allow_permanent Permit the operation, which cannot be undone
      @param block_pointer First block the mask covers, in units of sixteen
      @param block_range Words of mask, each covering sixteen blocks
      @return Nothing on success, or why it did not go through
      @warning A locked block is unwritable for the life of the tag. There is no way back, which
      is why this does nothing unless allow_permanent says otherwise
     */
    Result blockPermalock(const Bank bank, const uint8_t* mask, const size_t mask_len, const bool allow_permanent,
                          const uint16_t block_pointer = 0, const uint8_t block_range = 1);
    /*!
      @brief Read the QT control word of the selected tag
      @param[out] qt What the tag answered
      @return Nothing on success, or why it did not go through
      @note Impinj Monza 4QT only. A chip the table names and that does not have the command is
      turned away before anything is sent; one it does not name is tried
     */
    Result readQTParameters(QTParameters& qt);
    /*!
      @brief Write the QT control word of the selected tag
      @param qt What to write
      @param persistent True to write it where a loss of power does not undo it
      @return Nothing on success, or why it did not go through
      @details Switching to the public map hides the tag's user memory, the longer half of its
      EPC, and all of its TID but the first two words. The tag answers with an EPC of its own
      kept for that purpose, so afterwards it looks like a different tag
      @warning A volatile write lasts only while the tag is powered, which is until the reader
      stops transmitting: the map is back to what it was before the tag is next found. Anything
      meant to outlast that has to be persistent
      @note Either way the tag can be switched back, so neither is a one-way door
     */
    Result writeQTParameters(const QTParameters& qt, const bool persistent = false);
    /*!
      @brief Read the Config-Word of an NXP UCODE G2X
      @param[out] config Word the tag holds
      @return Nothing on success, or why it did not go through
      @details The word carries the read protection, the Product Status Flag and the range
      reduction among other things. Which bit is which is in the chip's own datasheet
     */
    Result readNxpConfigWord(uint16_t& config);
    /*!
      @brief Invert bits of the Config-Word of an NXP UCODE G2X
      @param[out] config Word the tag holds afterwards
      @param toggle Bits to invert
      @return Nothing on success, or why it did not go through
      @details The word is toggled rather than assigned: a one inverts the bit it stands over
      and a zero leaves it alone, so sending the same bits twice puts the word back. That is
      the command the chip offers, and calling it a write would say something else
      @warning A tag answers a toggle from the secured state alone. From the open state it
      hands back the word it already had and changes nothing, and says nothing about having
      refused (UCODE G2iM SL3S1003_1013 Rev3.7 Table 12). Reading costs nothing either way
      @warning A bit inverted by mistake changes what the tag will answer at all
     */
    Result toggleNxpConfigWord(uint16_t& config, const uint16_t toggle);
    /*!
      @brief Set or clear the Product Status Flag of an NXP UCODE G2X
      @param enable True to assert the flag, false to clear it
      @return Nothing on success, or why it did not go through
      @details A tag whose flag is asserted answers nxpEASAlarm()
      @warning A tag whose access password is zero ignores this command outright. Give it one
      first
     */
    Result writeNxpEAS(const bool enable);
    /*!
      @brief Ask the field whether any tag has its Product Status Flag asserted
      @param[out] alarm Code the tag backscattered, or empty when nothing answered
      @return True when the question was put and an answer came back
      @note This asks the field rather than one tag, so it needs no selection. Nothing being
      flagged is an answer and not a failure, which is why an empty code is reported apart
      from whether the question could be asked at all
     */
    bool nxpEASAlarm(std::vector<uint8_t>& alarm);
    /*!
      @brief Turn the read protection of an NXP UCODE G2X on or off
      @param protect True to protect, false to put it back
      @return Nothing on success, or why it did not go through
      @details Protected memory reads back as zeroes rather than falling silent, so the tag
      goes on answering an inventory round and can be addressed again to undo this
      @warning The EPC and the TID are protected together, so every protected tag reads back
      the same all-zero EPC and they can no longer be told apart. Undo this with one tag in
      the field
      @warning A tag whose access password is zero ignores this command outright. Give it one
      first
     */
    Result nxpReadProtect(const bool protect);
    ///@}

private:
    //! @brief Store a mask, switch the module over to it, and optionally prove a tag answers
    bool apply_selection(const Bank bank, const uint32_t pointer_bits, const uint8_t* mask, const size_t mask_len,
                         const uint32_t access_password, const bool verify);
    //! @brief Read one word of the EPC bank to prove the selected tag is there and answering
    bool verify_selection();
    //! @brief Drop the selection when a write has replaced the bytes its mask matches on
    void drop_selection_if_mask_rewritten(const Bank bank);
    //! @brief Store the mask of the selected tag again, so a lost singulation can be regained
    bool reapply_selection();
    //! @brief How many words of a bank there are to read, 0 when that is not known
    uint16_t bank_words(const Bank bank);
    //! @brief Print a stretch of one bank, sixteen bytes to a line
    bool dump_words(const char* what, const Bank bank, const uint16_t word_address, const uint16_t words);
    /*!
      @brief Stop polling for as long as a tag is being addressed
      @details An inventory round leaves every tag it saw flagged as already counted, and a tag
      in that state answers nothing until the flag decays. Letting polling run between two tag
      operations therefore breaks the second one, so it stays stopped from select() until
      deselect() rather than being stopped and restarted around each operation
      @return False when polling was running and would not stop. The module is still running
      rounds, and a tag operation attempted now would be answering into them
     */
    /*!
      @brief Pass on a reader's failure in terms the caller can act on
      @param result What the reader answered
      @return The same success, or what its error code means
     */
    Result relay(const m5::unit::TagResult& result) const;
    bool pause_polling();
    //! @brief Put polling back the way select() found it
    void resume_polling();

    m5::unit::UHFRFIDComponent& _u;
    Tag _selected{};
    uint32_t _access_password{};
    //! Bank the stored mask matches against, which decides whether an EPC rewrite invalidates it
    Bank _mask_bank{Bank::Epc};
    bool _has_selection{};
    //! Was polling running when the tag currently being addressed was selected?
    bool _resume_polling{};
};

}  // namespace uhf
}  // namespace m5
#endif
