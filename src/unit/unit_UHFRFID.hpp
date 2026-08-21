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
        //! Start polling on begin
        bool start_polling{false};
        //! Rounds requested per multiple polling command. Kept short deliberately: the module
        //! runs the rounds on its own and cannot be stopped once the host is gone, so a large
        //! count leaves it transmitting for minutes after an MCU reset. update() reissues the
        //! command long before the count runs out, so polling still looks continuous.
        uint16_t polling_count{128};
        //! Operating region (Unspecified keeps the module's factory setting)
        m5::uhf::Region region{m5::uhf::Region::Unspecified};
        //! Capacity of the tag queue
        uint16_t tag_queue_size{32};
        //! Serial I/O timeout (ms)
        uint32_t timeout_ms{1000};
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
    //! @brief Stop continuous polling
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
    //! @brief Read the operating channel index
    virtual bool readChannel(uint8_t& index) = 0;
    //! @brief Write the operating channel index
    virtual bool writeChannel(const uint8_t index) = 0;
    //! @brief Enable or disable automatic frequency hopping
    virtual bool writeAutomaticFrequencyHopping(const bool enable) = 0;
    //! @brief Read the query parameters
    virtual bool readQueryParameters(m5::uhf::QueryParameters& qp) = 0;
    //! @brief Write the query parameters
    virtual bool writeQueryParameters(const m5::uhf::QueryParameters& qp) = 0;
    /*!
      @brief Write the inactivity period after which the module sleeps automatically
      @param minutes 1 to 30 minutes, or 0 to disable automatic sleep
      @return True if successful
      @note Waking the module costs the first byte it receives and makes it reload the chip
      firmware, so a sleeping module ignores the command that woke it. Disabling automatic
      sleep avoids that entirely when the unit is permanently powered
     */
    virtual bool writeAutoSleepTime(const uint8_t minutes) = 0;
    /*!
      @brief Put the module into low power sleep
      @return True if successful
      @note Any byte wakes the module up again, but that byte is discarded and the module
      reloads the chip firmware before it can answer
     */
    virtual bool sleep() = 0;
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
    bool _polling{};
    uint16_t _polling_count{};
    unsigned long _last_frame_at{};
    unsigned long _polling_issued_at{};

    friend class m5::uhf::UHFLayer;
};

}  // namespace unit
}  // namespace m5
#endif
