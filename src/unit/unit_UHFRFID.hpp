/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_UHFRFID.hpp
  @brief Base class for UHF-RFID reader units
*/
#ifndef M5_UNIT_RFID_UNIT_UNIT_UHFRFID_HPP
#define M5_UNIT_RFID_UNIT_UNIT_UHFRFID_HPP

#include <memory>
#include <vector>

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <m5_utility/stl/expected.hpp>

#include "uhf/uhf.hpp"

namespace m5 {
namespace uhf {
class UHFLayer;
}  // namespace uhf

namespace unit {

/*!
  @typedef TagResult
  @brief What became of an operation on a tag
  @details Nothing on success. On failure the error code the reader reported, whose meaning is
  the reader's own; classify() is what turns it into something a caller can act on
 */
using TagResult = m5::stl::expected<void, uint8_t>;

/*!
  @name Codes this library puts in place of one the reader did not give
  @details A reader reports a code only when something came back over the air. Where a request
  never went out, or the answer could not be read, there is no code to report and one of these
  stands in its place. EPC Gen2 leaves 0xF0 to 0xFF outside the tag error codes and the M100
  uses none of them, so they are free to say what no reader would
 */
///@{
constexpr uint8_t READER_SILENT{0xFF};   //!< Nothing came back at all
constexpr uint8_t READER_BALKED{0xFE};   //!< The reader answered, and said it had not done it
constexpr uint8_t READER_GARBLED{0xFD};  //!< The answer came back and could not be read
constexpr uint8_t NOT_SELECTED{0xFC};    //!< No tag had been selected, so nothing was sent
constexpr uint8_t BAD_ARGUMENT{0xFB};    //!< The request could not succeed, so nothing was sent
constexpr uint8_t READER_BUSY{0xFA};     //!< Polling held the reader, so nothing was sent
///@}

/*!
  @class UHFRFIDComponent
  @brief Non-instantiable base class for UHF-RFID reader units
  @details Owns the UART transport, the frame pump and the raw tag queue.
  EPC Gen2 semantics live in m5::uhf::UHFLayer.
  @warning A tag's write range is shorter than its read range: writing takes more power, and
  these memories erase before they program, of which erasing is the cheaper half. A tag well
  within reading distance can therefore have a word erased and not programmed, leaving it
  neither as it was nor as it was asked to be. A command that goes unanswered is not a command
  that did nothing
 */
class UHFRFIDComponent : public Component {
public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! Rounds requested per multiple polling command. Kept short deliberately: the module
        //! runs the rounds on its own and cannot be stopped once the host is gone, so a large
        //! count leaves it transmitting for minutes after an MCU reset. update() reissues the
        //! command long before the count runs out, so polling still looks continuous.
        uint16_t polling_count{128};
        //! Operating region (Unspecified keeps the module's factory setting)
        m5::uhf::Region region{m5::uhf::Region::Unspecified};
        //! Capacity of the tag queue
        uint16_t tag_queue_size{32};
        /*!
          How long to wait for the answer to an ordinary command, in milliseconds. Measured, a
          module answers one of these within about 15ms; the room above that is for a module
          that is busy rather than for one that is working normally
          @note Tag operations and channel scans take far longer and carry limits of their own
         */
        uint32_t command_timeout_ms{1000};
        /*!
          Inactivity period after which the module sleeps by itself, 1 to 30 minutes. Zero
          turns that off, which is the default
          @note This is written at begin() rather than left as it was, because the module has
          no command to say what it is set to and the setting survives a power cycle. Writing
          it is the only way to know it
         */
        uint8_t auto_sleep_minutes{0};
    };

    virtual ~UHFRFIDComponent() = default;

    ///@name Settings for begin
    ///@{
    //! @brief Gets the configuration
    inline config_t config() const
    {
        return _cfg;
    }
    //! @brief Sets the configuration
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    ///@name Polling
    ///@{
    /*!
      @brief Start continuous polling
      @param count Rounds requested per multiple polling command (0 - 65535)
      @return True if successful
      @note The count is finite, and update() renews the command before it runs out, so the
      caller sees continuous operation. Keep it short: the module runs the rounds on its own and
      cannot be stopped once the host is gone
     */
    bool startPolling(const uint16_t count);
    /*!
      @brief Stop continuous polling
      @return True if the module confirmed it stopped
      @note Returning false does not leave the module running rounds for ever. Nothing renews
      the polling once this has been called, so the count it was started with runs out and it
      falls silent on its own within seconds
     */
    bool stopPolling();
    //! @brief In continuous polling?
    inline bool inPolling() const
    {
        return _polling;
    }
    /*!
      @brief Timestamp (m5::utility::millis()) of the most recently arrived frame
      @details Updated by note_frame_arrival() for every frame, including a no-tag notification
      (Inventory Fail). Useful to observe polling liveness from outside even when no tag is
      present, since available() alone cannot show that frames are still flowing.
     */
    inline unsigned long lastFrameAt() const
    {
        return _last_frame_at;
    }
    ///@}

    ///@name Tag queue (raw; not deduplicated)
    ///@{
    //! @brief Number of queued tag notifications
    inline size_t available() const
    {
        return _tags ? _tags->size() : 0U;
    }
    //! @brief Oldest queued tag
    inline const m5::uhf::Tag& oldest() const
    {
        return (*_tags)[0];
    }
    //! @brief Discard the oldest queued tag
    inline void discard()
    {
        if (_tags && !_tags->empty()) {
            _tags->pop_front();
        }
    }
    //! @brief Discard all queued tags
    inline void flush()
    {
        if (_tags) {
            _tags->clear();
        }
    }
    //! @brief Number of tag notifications dropped due to queue overflow
    inline uint32_t dropped() const
    {
        return _dropped;
    }
    //! @brief Clear the dropped counter
    inline void clearDropped()
    {
        _dropped = 0;
    }
    ///@}

    ///@name Reader settings
    ///@{
    //! @brief Read the module information
    virtual bool readModuleInformation(m5::uhf::ModuleInformation& info) = 0;
    //! @brief Read the transmit power in 1/100 dBm
    virtual bool readTransmitPower(int16_t& dbm100) = 0;
    //! @brief Write the transmit power in 1/100 dBm
    virtual bool writeTransmitPower(const int16_t dbm100) = 0;
    //! @brief Read the operating region
    virtual bool readRegion(m5::uhf::Region& region) = 0;
    //! @brief Write the operating region
    virtual bool writeRegion(const m5::uhf::Region region) = 0;
    /*!
      @brief Read the operating channel index
      @param[out] index Channel index within the operating region
      @return True if successful
      @details Whichever channel the reader is on at that moment, which is not the same as a
      setting: while it hops it moves on its own, and two reads a second apart disagree
     */
    virtual bool readChannel(uint8_t& index) = 0;
    //! @brief Write the operating channel index
    virtual bool writeChannel(const uint8_t index) = 0;
    //! @brief Enable or disable automatic frequency hopping
    virtual bool writeAutomaticFrequencyHopping(const bool enable) = 0;
    /*!
      @brief Turn the unmodulated carrier on or off
      @param enable True to transmit a carrier, false to stop
      @return True if successful
      @details A carrier with nothing modulated onto it, which is what a spectrum analyser or a
      power meter needs to measure the transmitter. No tag answers it and nothing else uses it
      @warning This transmits until it is turned off. What may be radiated, and for how long, is
      a question of where the unit is being used: a carrier that never stops is not what the
      channel rules of most regions have in mind
     */
    virtual bool writeContinuousCarrier(const bool enable) = 0;
    //! @brief Read the query parameters
    virtual bool readQueryParameters(m5::uhf::QueryParameters& qp) = 0;
    //! @brief Write the query parameters
    virtual bool writeQueryParameters(const m5::uhf::QueryParameters& qp) = 0;
    /*!
      @brief Write the inactivity period after which the module sleeps automatically
      @param minutes 1 to 30 minutes, or 0 to disable automatic sleep
      @return True if successful
      @warning This is the same sleep as sleep(), reached without anyone calling it, so it
      takes the select mask with it: a tag addressed before the module went to sleep is not
      the one answering afterwards. writeIdle() takes a period of its own and keeps the mask
      @warning The module has no command to say what this is set to, and it survives a power
      cycle, so what a module holds now cannot be found out
     */
    virtual bool writeAutoSleepTime(const uint8_t minutes) = 0;
    /*!
      @brief Put the module into IDLE, or bring it out
      @param enter True to enter IDLE, false to leave it
      @param minutes How long without work before the module enters IDLE by itself, 0 to 30.
      Zero means it does not
      @return True if successful
      @details IDLE turns off everything but the digital part and the serial port. Unlike
      sleep() it keeps what the module was holding, the select parameter included, and the
      module goes on answering commands
      @note The first tag operation after this brings the module back by itself, and that one
      may fail where a later one would not: the radio is still settling
     */
    virtual bool writeIdle(const bool enter, const uint8_t minutes = 0) = 0;
    /*!
      @brief Put the module into low power sleep
      @return True if successful
      @warning Any byte wakes a sleeping module, and the module throws that byte away. Sending
      a command to one therefore wakes it and loses the command, so the first command after a
      sleep goes unanswered
      @warning The select mask does not survive this. See wake()
      @warning A tag being addressed should be given up with UHFLayer::deselect() first. The
      mask does not survive the sleep, and a layer that still believes a tag is selected would
      read whichever tag answers
     */
    virtual bool sleep() = 0;
    /*!
      @brief Wake the module from low power sleep
      @return True if successful
      @details Sends one byte to wake it and waits for the chip firmware to reload. The byte
      itself is thrown away by the module, which is why it is sent on its own rather than as
      the first real command
      @warning Waking restores the power, the frequency, the hopping mode, the sleep time and
      the demodulator parameters, but not the select mode or the select parameter. The module
      comes back holding a mask of zero length, which matches every tag rather than none, so a
      read or a write goes on succeeding against whichever tag answers. A tag addressed before
      the sleep has to be selected again; UHFLayer::wake() does that for a caller working
      through the layer
      @warning This can return before the module answers. How long waking takes is not
      documented, and a module can take over a second
      @note This spends a command of its own, so it is not free to call on a module that is
      already awake
     */
    virtual bool wake() = 0;
    /*!
      @brief Replace the channel list used for automatic frequency hopping
      @param channels Channel indices of the operating region, in the order to hop through
      @return True if successful
      @note Automatic frequency hopping picks from this list once it is set, instead of the
      list preset in the module. Useful to keep the reader inside the channels a region permits
     */
    virtual bool writeOperatingChannels(const std::vector<uint8_t>& channels) = 0;
    /*!
      @brief Measure the blocking signal on every channel of the operating region
      @param[out] levels Measured level of each channel
      @return True if successful
      @details Scans for interference the reader itself does not produce, which is what keeps a
      tag from being read when the antenna and the power are both fine
     */
    virtual bool readBlockingSignal(m5::uhf::ChannelLevels& levels) = 0;
    /*!
      @brief Measure the RSSI on every channel of the operating region
      @param[out] levels Measured level of each channel
      @return True if successful
      @details Reveals other readers operating nearby
     */
    virtual bool readChannelRSSI(m5::uhf::ChannelLevels& levels) = 0;
    /*!
      @brief Read back the select mask the reader holds
      @param[out] sp Select parameter
      @return True if successful
      @details No tag replies to a Select, so this is the only way to confirm that the reader is
      addressing what the caller believes it is
     */
    virtual bool readSelectParameter(m5::uhf::SelectParameter& sp) = 0;
    ///@}

    ///@name Tag operations
    ///@{
    /*!
      @brief Store the mask that picks out one tag
      @param bank Bank the mask is matched against; Reserved is not selectable
      @param pointer_bits Where the mask starts, as a bit address inside the bank
      @param mask Mask bytes
      @param mask_len Length of mask in bytes
      @return True if successful
      @note Storing a mask does not talk to any tag, so this succeeds for a tag that is not in
      the field. Reading the tag back is the only way to learn whether it answered
      @note A mask of zero length matches every tag rather than none, so it is refused here
     */
    virtual bool writeSelectParameter(const m5::uhf::Bank bank, const uint32_t pointer_bits, const uint8_t* mask,
                                      const size_t mask_len) = 0;
    /*!
      @brief Apply the stored mask to tag operations, or stop applying it
      @param enable True to address only the selected tag, false to address any tag
      @return True if successful
      @note Inventory is left unfiltered either way, so polling keeps reporting every tag in
      the field while Read, Write, Lock and Kill are narrowed down to the selected one
     */
    virtual bool writeSelectEnabled(const bool enable) = 0;
    /*!
      @brief Read word_count 16-bit words from bank, starting at word_address
      @param[out] out Bytes read
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param word_count Number of 16-bit words
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
     */
    virtual TagResult readTagMemory(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t word_address,
                                    const uint16_t word_count, const uint32_t access_password) = 0;
    /*!
      @brief Write len bytes to bank, starting at word_address
      @param bank Memory bank
      @param word_address Start address in 16-bit words
      @param data Bytes to write
      @param len Their length, which has to be even
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @warning Writing the EPC bank changes the very bytes an EPC mask matches on, so the mask
      in force stops picking this tag out. Store it again from what the tag now holds
     */
    virtual TagResult writeTagMemory(const m5::uhf::Bank bank, const uint16_t word_address, const uint8_t* data,
                                     const size_t len, const uint32_t access_password) = 0;
    /*!
      @brief Apply a 20-bit Gen2 lock payload
      @param payload Lock payload, see m5::uhf::buildLockPayload
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @warning A payload asking for a permanent lock cannot be undone. m5::uhf::LockAction
      names which of them those are
     */
    virtual TagResult lockTagMemory(const uint32_t payload, const uint32_t access_password) = 0;
    /*!
      @brief Kill the addressed tag permanently
      @param kill_password Kill password the tag holds, which may not be zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @warning A killed tag never answers again. Nothing here can say whether the mask in force
      picks out the tag that was meant rather than another one in the field, so what is about
      to die is worth reading back first
      @code
      std::vector<uint8_t> epc{};
      if (unit.readTagMemory(epc, m5::uhf::Bank::Epc, 2, 6, 0) && epc == expected) {
          unit.killTag(password);
      }
      @endcode
     */
    virtual TagResult killTag(const uint32_t kill_password) = 0;
    /*!
      @brief Read or set which blocks of a bank are permanently locked
      @param[out] out Mask read, one bit per block with the first block in the most significant
      bit. Left alone when locking
      @param bank Bank the blocks are in
      @param block_pointer First block the mask covers, in units of sixteen
      @param block_range Words of mask, each covering sixteen blocks
      @param mask Mask to apply, or nullptr to read
      @param mask_len Its length in bytes, which is twice block_range
      @param access_password Access password the tag holds, or zero
      @param allow_permanent Say so to mean a lock, which is refused without it
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @warning A block locked this way is unwritable for the life of the tag. Reading it is
      unaffected, and so is every other block
     */
    virtual TagResult blockPermalock(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t block_pointer,
                                     const uint8_t block_range, const uint8_t* mask, const size_t mask_len,
                                     const uint32_t access_password, const bool allow_permanent) = 0;
    /*!
      @brief Read or write the QT control word of the addressed tag
      @param[in,out] control Word read, or word to write
      @param write True to write, false to read
      @param persistent True to write where a loss of power does not undo it
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @warning Switching an Impinj Monza 4QT to its public map changes the EPC it answers
      under, so the mask in force stops picking it out
     */
    virtual TagResult qtCommand(uint16_t& control, const bool write, const bool persistent,
                                const uint32_t access_password) = 0;
    /*!
      @brief Read the Config-Word of an NXP UCODE G2X, or invert bits of it
      @param[out] config Word the tag holds once the command has been carried out
      @param toggle Bits to invert. Zero reads the word without changing anything
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @details The word is toggled rather than assigned: a one inverts the bit it stands over
      and a zero leaves it alone. Sending the same bits twice puts the word back
      @warning Reading costs nothing, but a tag answers a toggle from the secured state alone.
      From the open state it hands back the word it already had and changes nothing, and says
      nothing about having refused (UCODE G2iM SL3S1003_1013 Rev3.7 Table 12)
      @warning The Config-Word carries the read protection and the range reduction among other
      things, so a bit inverted by mistake changes what the tag will answer at all
     */
    virtual TagResult nxpChangeConfig(uint16_t& config, const uint16_t toggle, const uint32_t access_password) = 0;
    /*!
      @brief Set or clear the Product Status Flag of an NXP UCODE G2X
      @param enable True to assert the flag, false to clear it
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @details A tag whose flag is asserted answers nxpEASAlarm(), which is what an article
      surveillance gate listens for
      @warning A tag whose access password is zero ignores this command outright. Give it one
      first
     */
    virtual TagResult nxpChangeEAS(const bool enable, const uint32_t access_password) = 0;
    /*!
      @brief Ask the field whether any tag has its Product Status Flag asserted
      @param[out] alarm Code the tag backscattered, or empty when nothing answered
      @return True when the question was put and an answer came back
      @details This asks the field rather than one tag, so it needs neither a selection nor a
      password. What answers hands back a code rather than a name: an NXP UCODE G2iM sends a
      fixed 64-bit one
      @note Nothing being flagged is an answer and not a failure, which is why an empty code
      is reported apart from whether the question could be asked at all
     */
    virtual bool nxpEASAlarm(std::vector<uint8_t>& alarm) = 0;
    /*!
      @brief Turn the read protection of an NXP UCODE G2X on or off
      @param protect True to protect, false to put it back
      @param access_password Access password the tag holds, or zero
      @return Nothing on success, or the error code the reader reported
      @pre A mask has to have been stored and applied, or this is refused
      @details Protected memory reads back as zeroes rather than falling silent, so the tag
      still answers an inventory round and can be addressed again to undo this
      @warning Both the EPC and the TID are protected at once, so every protected tag reads
      back the same all-zero EPC and they can no longer be told apart. Undo this with one tag
      in the field
      @warning A tag whose access password is zero ignores this command outright. Give it one
      first
     */
    virtual TagResult nxpReadProtect(const bool protect, const uint32_t access_password) = 0;
    /*!
      @brief Say what one of this reader's error codes means in EPC Gen2 terms
      @param error_code Code from the error of a TagResult
      @return What became of the operation
      @details Which codes a reader uses is its own affair, so each reader says what its own
      mean. That keeps the layer above able to act on a failure without knowing the reader
     */
    virtual m5::uhf::Reason classify(const uint8_t error_code) const = 0;
    ///@}

    virtual bool begin() override;
    virtual void update(const bool force = false) override;

protected:
    explicit UHFRFIDComponent(const uint8_t addr = 0x00) : Component(addr)
    {
    }

    // Pump the UART stream and route frames. Returns true if any frame was handled
    virtual bool pump(const uint32_t timeout_ms) = 0;
    // Issue the multiple polling command
    virtual bool start_polling_command(const uint16_t count) = 0;
    // Issue the stop polling command
    virtual bool stop_polling_command() = 0;

    /*
      Refuse a reader setting while polling is running
      @param what Name of the operation, used in the warning
      @return True when the caller must give up
      @details The module answers unreliably while it is running inventory rounds, so reader
      settings are rejected outright instead of failing later with a timeout
     */
    bool reject_while_polling(const char* what) const;

    /*
      Refuse a tag operation while no mask is picking a tag out
      @param what Name of the operation, used in the warning
      @return True when the caller must give up
      @details A mask of zero length matches every tag rather than none, so an operation sent
      without one does not fail: it succeeds against whichever tag answers. A write lands on a
      tag nobody chose and a kill cannot be taken back, so these are rejected outright
     */
    bool reject_without_selection(const char* what) const;

    // Push a tag into the queue, counting a drop when it overflows
    void push_tag(const m5::uhf::Tag& tag);
    // Note that a frame arrived (exposed through lastFrameAt)
    void note_frame_arrival();
    // Reissue the polling command before its round count runs out
    void reissue_polling_if_needed();

    config_t _cfg{};
    std::unique_ptr<m5::container::CircularBuffer<m5::uhf::Tag>> _tags{};
    uint32_t _dropped{};
    // Whether polling should be renewed. Cleared as soon as a stop is asked for
    bool _polling{};
    // Whether the module is believed to still be running rounds. Only a confirmed stop clears it
    bool _rounds_running{};
    // Whether a mask of some length has been stored in the module
    bool _select_mask_stored{};
    // Whether the stored mask is being applied to tag operations
    bool _select_enabled{};
    uint16_t _polling_count{};
    unsigned long _last_frame_at{};
    unsigned long _polling_issued_at{};

    friend class m5::uhf::UHFLayer;
};

}  // namespace unit
}  // namespace m5
#endif
