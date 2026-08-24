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
    UHFLayer(const UHFLayer&)            = delete;
    UHFLayer& operator=(const UHFLayer&) = delete;

    ///@name Detection
    ///@{
    /*!
      @brief Detect tags
      @param[out] tags Detected tags, deduplicated by EPC
      @param timeout_ms Timeout in milliseconds
      @return True if at least one tag was detected
      @note Starts polling internally and restores the previous polling state on return
     */
    bool detect(std::vector<Tag>& tags, const uint32_t timeout_ms = 1000U);
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
      @brief Write a memory bank
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param data Bytes to write; the length must be a whole number of 16-bit words
      @return True if successful
      @note A bank larger than one command can carry is written in several, so the caller does
      not have to know how much that is. A write that fails partway leaves everything before the
      failure already on the tag, and says in the log how far it got
      @note Writing the EPC bank changes the very bytes an EPC mask matches on, so a selection
      made from an EPC is dropped afterwards and has to be made again
     */
    bool writeBank(const Bank bank, const uint16_t word_address, const std::vector<uint8_t>& data);
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
    /*!
      @brief Stop polling for as long as a tag is being addressed
      @details An inventory round leaves every tag it saw flagged as already counted, and a tag
      in that state answers nothing until the flag decays. Letting polling run between two tag
      operations therefore breaks the second one, so it stays stopped from select() until
      deselect() rather than being stopped and restarted around each operation
     */
    void pause_polling();
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
