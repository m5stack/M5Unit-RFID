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
  @class UHFLayer
  @brief EPC Gen2 semantics layer for UHF-RFID reader units
  @details Holds semantics defined by the EPC Gen2 standard, which stay the same across
  reader chips. The frame level belongs to the unit class.
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
      @return True if successful
      @note Addresses and lengths count words because EPC Gen2 lays tag memory out in 16-bit
      words; a byte address off a word boundary means nothing to a tag
     */
    bool readBank(std::vector<uint8_t>& out, const Bank bank, const uint16_t word_address, const uint16_t word_count);
    /*!
      @brief Read a memory bank into a buffer of the caller's own
      @param[out] out Buffer, which has to hold word_count words
      @param[in,out] out_len Its size in bytes going in, and how many bytes arrived coming out
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param word_count Number of 16-bit words to read
      @return True if successful
      @note A tag can answer with less than was asked for, which is what out_len reports. A
      buffer too small for word_count words is refused rather than filled part way
     */
    bool readBank(uint8_t* out, uint16_t& out_len, const Bank bank, const uint16_t word_address,
                  const uint16_t word_count);
    /*!
      @brief Write a memory bank
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param data Bytes to write
      @param data_len Their length, which has to be a whole number of 16-bit words
      @return True if successful
      @note A bank larger than one command can carry is written in several, so the caller does
      not have to know how much that is. A write that fails partway leaves everything before the
      failure already on the tag, and says in the log how far it got
      @note Writing the EPC bank changes the very bytes an EPC mask matches on, so a selection
      made from an EPC is dropped afterwards and has to be made again
     */
    bool writeBank(const Bank bank, const uint16_t word_address, const uint8_t* data, const uint16_t data_len);
    /*!
      @brief Change the lock state of memory banks and passwords
      @param settings What to change; anything left out keeps the state it has
      @param allow_permanent Permit PermanentOpen and PermanentLock, neither of which can be undone
      @return True if successful
      @warning PermanentLock leaves an area unwritable for the life of the tag and PermanentOpen
      leaves it impossible to ever lock. There is no way back from either
     */
    bool lock(const std::vector<LockSetting>& settings, const bool allow_permanent = false);
    /*!
      @brief Kill a tag permanently
      @param tag Tag to kill, which must be the one selected
      @param kill_password Kill password; a tag whose kill password is zero refuses to be killed
      @return True if successful
      @warning The tag stops answering for good. There is no way to revive it
     */
    bool kill(const Tag& tag, const uint32_t kill_password);
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
