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
        bool start_polling{true};
        //! Polling count issued per multiple polling command
        uint16_t polling_count{0xFFFF};
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
      @param count Polling count issued per multiple polling command
      @return True if successful
      @note The count is finite (0 - 65535). When it is exhausted the command is reissued
      automatically, so the caller sees continuous operation
     */
    bool startPolling(const uint16_t count = 0xFFFF);
    //! @brief Stop continuous polling
    bool stopPolling();
    //! @brief In continuous polling?
    inline bool inPolling() const
    {
        return _polling;
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

    //! @brief Push a tag into the queue, counting a drop when it overflows
    void push_tag(const m5::uhf::Tag& tag);
    //! @brief Note that a frame arrived (used to detect polling count exhaustion)
    void note_frame_arrival();
    //! @brief Reissue the polling command when the count looks exhausted
    void reissue_polling_if_needed();

    config_t _cfg{};
    std::unique_ptr<m5::container::CircularBuffer<m5::uhf::Tag>> _tags{};
    uint32_t _dropped{};
    bool _polling{};
    uint16_t _polling_count{0xFFFF};
    unsigned long _last_frame_at{};
    uint32_t _reissue_threshold_ms{500};

    friend class m5::uhf::UHFLayer;
};

}  // namespace unit
}  // namespace m5
#endif
