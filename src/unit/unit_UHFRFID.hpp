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

#include "uhf/uhf.hpp"

namespace m5 {
namespace uhf {
class UHFLayer;
}  // namespace uhf

namespace unit {

/*!
  @class UHFRFIDComponent
  @brief Non-instantiable base class for UHF-RFID reader units
  @details Owns the UART transport, the frame pump and the raw tag queue.
  EPC Gen2 semantics live in m5::uhf::UHFLayer.
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

    virtual bool begin() override;
    virtual void update(const bool force = false) override;

protected:
    explicit UHFRFIDComponent(const uint8_t addr = 0x00) : Component(addr)
    {
    }

    //! @brief Pump the UART stream and route frames. Returns true if any frame was handled
    virtual bool pump(const uint32_t timeout_ms) = 0;
    //! @brief Issue the multiple polling command
    virtual bool start_polling_command(const uint16_t count) = 0;
    //! @brief Issue the stop polling command
    virtual bool stop_polling_command() = 0;

    ///@name Tag operations, driven by m5::uhf::UHFLayer
    ///@{
    /*!
      @brief Store the mask that picks out one tag
      @param bank Bank the mask is matched against; Reserved is not selectable
      @param pointer_bits Where the mask starts, as a bit address inside the bank
      @param mask Mask bytes
      @param mask_len Length of mask in bytes
      @return True if successful
      @note Storing a mask does not talk to any tag, so this succeeds for a tag that is not in
      the field
     */
    virtual bool write_select_parameter(const m5::uhf::Bank bank, const uint32_t pointer_bits, const uint8_t* mask,
                                        const size_t mask_len) = 0;
    /*!
      @brief Apply the stored mask to tag operations, or stop applying it
      @param enable True to address only the selected tag, false to address any tag
      @return True if successful
      @note Inventory is left unfiltered either way, so polling keeps reporting every tag in
      the field while Read, Write, Lock and Kill are narrowed down to the selected one
     */
    virtual bool write_select_enabled(const bool enable) = 0;
    //! @brief Read word_count 16-bit words from bank, starting at word_address
    virtual bool read_tag_memory(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t word_address,
                                 const uint16_t word_count, const uint32_t access_password) = 0;
    //! @brief Write len bytes to bank, starting at word_address. len must be even
    virtual bool write_tag_memory(const m5::uhf::Bank bank, const uint16_t word_address, const uint8_t* data,
                                  const size_t len, const uint32_t access_password) = 0;
    //! @brief Apply a 20-bit Gen2 lock payload, see m5::uhf::buildLockPayload
    virtual bool lock_tag_memory(const uint32_t payload, const uint32_t access_password) = 0;
    //! @brief Kill the addressed tag permanently
    virtual bool kill_tag(const uint32_t kill_password) = 0;
    /*!
      @brief Read or write the QT control word of the addressed tag
      @param[in,out] control Word read, or word to write
      @param write True to write, false to read
      @param persistent True to write where a loss of power does not undo it
      @param access_password Access password of the tag
      @return True if successful
     */
    virtual bool qt_command(uint16_t& control, const bool write, const bool persistent,
                            const uint32_t access_password) = 0;
    ///@}

    /*!
      @brief Refuse a reader setting while polling is running
      @param what Name of the operation, used in the warning
      @return True when the caller must give up
      @details The module answers unreliably while it is running inventory rounds, so reader
      settings are rejected outright instead of failing later with a timeout
     */
    bool reject_while_polling(const char* what) const;

    //! @brief Push a tag into the queue, counting a drop when it overflows
    void push_tag(const m5::uhf::Tag& tag);
    //! @brief Note that a frame arrived (exposed through lastFrameAt)
    void note_frame_arrival();
    //! @brief Reissue the polling command before its round count runs out
    void reissue_polling_if_needed();

    config_t _cfg{};
    std::unique_ptr<m5::container::CircularBuffer<m5::uhf::Tag>> _tags{};
    uint32_t _dropped{};
    //! Whether polling should be renewed. Cleared as soon as a stop is asked for
    bool _polling{};
    //! Whether the module is believed to still be running rounds. Only a confirmed stop clears it
    bool _rounds_running{};
    uint16_t _polling_count{};
    unsigned long _last_frame_at{};
    unsigned long _polling_issued_at{};

    friend class m5::uhf::UHFLayer;
};

}  // namespace unit
}  // namespace m5
#endif
