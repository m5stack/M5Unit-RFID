/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_JRD4035.hpp
  @brief JRD-4035 (magicRF M100) UHF-RFID Unit for M5UnitUnified
*/
#ifndef M5_UNIT_RFID_UNIT_UNIT_JRD4035_HPP
#define M5_UNIT_RFID_UNIT_UNIT_JRD4035_HPP

#include <string>

#include "m100_frame.hpp"
#include "unit_UHFRFID.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitJRD4035
  @brief UHF-RFID unit based on the JRD-4035 module (magicRF M100)
  @details EPCglobal UHF Class 1 Gen 2 / ISO 18000-6C at 840-960MHz over UART
 */
class UnitJRD4035 : public UHFRFIDComponent {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitJRD4035, 0x00);

public:
    /*!
      @struct config_t
      @brief Settings for begin
      @details Adds the receiver demodulator settings on top of UHFRFIDComponent::config_t.
      They belong here rather than there because only this chip has them
     */
    struct config_t : public UHFRFIDComponent::config_t {
        /*!
          Receiver demodulator settings, written at begin() whatever they are set to
          @note The module keeps these across a power cycle, so what it holds when begin() runs
          is whatever was last written to it rather than a default. A unit can arrive set below
          the threshold its own documentation gives as the lowest worth using, which reaches
          tags at a distance but drops the exchanges that reading a tag's memory is made of
         */
        m100::DemodulatorParameters demodulator{};
    };

    UnitJRD4035() : UHFRFIDComponent(DEFAULT_ADDRESS)
    {
    }
    virtual ~UnitJRD4035() = default;

    virtual bool begin() override;

    ///@name Settings for begin
    ///@{
    //! @brief Gets the configuration
    inline config_t config() const
    {
        return _cfg_jrd4035;
    }
    //! @brief Sets the configuration
    inline void config(const config_t& cfg)
    {
        _cfg_jrd4035 = cfg;
        UHFRFIDComponent::config(static_cast<const UHFRFIDComponent::config_t&>(cfg));
    }
    ///@}

    ///@name Reader settings
    ///@{
    virtual bool readModuleInformation(m5::uhf::ModuleInformation& info) override;
    virtual bool readTransmitPower(int16_t& dbm100) override;
    virtual bool writeTransmitPower(const int16_t dbm100) override;
    virtual bool readRegion(m5::uhf::Region& region) override;
    virtual bool writeRegion(const m5::uhf::Region region) override;
    virtual bool readChannel(uint8_t& index) override;
    virtual bool writeChannel(const uint8_t index) override;
    virtual bool writeAutomaticFrequencyHopping(const bool enable) override;
    virtual bool writeContinuousCarrier(const bool enable) override;
    virtual bool readQueryParameters(m5::uhf::QueryParameters& qp) override;
    virtual bool writeQueryParameters(const m5::uhf::QueryParameters& qp) override;
    virtual bool writeAutoSleepTime(const uint8_t minutes) override;
    virtual bool writeIdle(const bool enter, const uint8_t minutes = 0) override;
    virtual bool wake() override;
    virtual bool writeOperatingChannels(const std::vector<uint8_t>& channels) override;
    virtual bool readBlockingSignal(m5::uhf::ChannelLevels& levels) override;
    virtual bool readChannelRSSI(m5::uhf::ChannelLevels& levels) override;
    virtual bool readSelectParameter(m5::uhf::SelectParameter& sp) override;

    ///@name Receiver tuning, specific to this chip
    ///@{
    /*!
      @brief Read the receiver demodulator settings
      @param[out] dp Demodulator parameters
      @return True if successful
     */
    bool readDemodulatorParameters(m100::DemodulatorParameters& dp);
    /*!
      @brief Write the receiver demodulator settings
      @param dp Demodulator parameters
      @return True if successful
      @details Transmit power decides how far the reader reaches and these decide how far it
      listens. Lowering both is what makes a tag resting on the antenna readable: left at the
      factory settings, which reach about a metre and a half, the reader misses most inventory
      rounds at contact however strong the reply is
     */
    bool writeDemodulatorParameters(const m100::DemodulatorParameters& dp);
    ///@}
    ///@}

    /*!
      @brief Put the module into low power sleep
      @return True if successful
      @warning Sleeping and waking repeatedly has been seen to leave this module answering
      nothing for tens of seconds, reporting a watchdog reset when it comes back. It does come
      back on its own, but nothing on the host can hurry it: on Unit UHF-RFID the module's
      enable pin is tied high on the board, and the connector carries only power and the
      serial port
     */
    virtual bool sleep() override;

    virtual bool writeSelectParameter(const m5::uhf::Bank bank, const uint32_t pointer_bits, const uint8_t* mask,
                                      const size_t mask_len) override;
    virtual bool writeSelectEnabled(const bool enable) override;
    virtual TagResult readTagMemory(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t word_address,
                                    const uint16_t word_count, const uint32_t access_password) override;
    virtual TagResult writeTagMemory(const m5::uhf::Bank bank, const uint16_t word_address, const uint8_t* data,
                                     const size_t len, const uint32_t access_password) override;
    virtual TagResult lockTagMemory(const uint32_t payload, const uint32_t access_password) override;
    virtual TagResult killTag(const uint32_t kill_password) override;
    virtual TagResult blockPermalock(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t block_pointer,
                                     const uint8_t block_range, const uint8_t* mask, const size_t mask_len,
                                     const uint32_t access_password, const bool allow_permanent) override;
    virtual TagResult qtCommand(uint16_t& control, const bool write, const bool persistent,
                                const uint32_t access_password) override;
    virtual TagResult nxpChangeConfig(uint16_t& config, const uint16_t toggle, const uint32_t access_password) override;
    virtual TagResult nxpChangeEAS(const bool enable, const uint32_t access_password) override;
    virtual bool nxpEASAlarm(std::vector<uint8_t>& alarm) override;
    virtual TagResult nxpReadProtect(const bool protect, const uint32_t access_password) override;
    virtual m5::uhf::Reason classify(const uint8_t error_code) const override;

protected:
    virtual bool pump(const uint32_t timeout_ms) override;
    virtual bool start_polling_command(const uint16_t count) override;
    virtual bool stop_polling_command() override;

    // Send a command frame
    bool send_command(const uint8_t command, const uint8_t* param, const uint16_t param_len);
    /*
      Send a command and wait for its response, routing notifications to the tag queue
      @param[out] response Response frame
      @param command Command to send
      @param param Parameter
      @param param_len Parameter length
      @param timeout_ms How long to wait, or 0 to wait config_t::command_timeout_ms
      @return True if successful
      @param answer_command Code the answer comes back under, where that is not the code sent
      @note Nearly every command is answered under the code it was sent, the two the protocol
      document says come back as something else included. A Monza QT write is the exception
     */
    bool send_and_wait(m100::Frame& response, const uint8_t command, const uint8_t* param, const uint16_t param_len,
                       const uint32_t timeout_ms = 0, const uint8_t answer_command = 0);
    // Read one frame from the stream. Returns false when nothing is pending
    bool read_frame(m100::Frame& out, const uint32_t timeout_ms);
    // Throw away everything received, in the driver and in the staging buffer alike
    void flush_rx();
    // Route a received frame
    void route_frame(const m100::Frame& f);
    // Wait out an answer that may still be on its way, so that it cannot answer the next command
    void resynchronize();
    // Read the module information for one kind (0x00 hardware / 0x01 software / 0x02 manufacturer)
    bool read_module_information_kind(std::string& out, const uint8_t kind);
    // Run one of the channel scans and decode its contiguous range of levels
    bool read_channel_levels(m5::uhf::ChannelLevels& levels, const uint8_t command);
    /*
      Did the module answer with a result rather than a failure?
      @param response Response frame
      @param what Name of the operation, used in the warning
      @return True when the response carries a result
      @details send_and_wait also returns true for a failure notification, since the module
      answering at all is what it waits for. Every tag operation has to tell the two apart
     */
    bool succeeded(const m100::Frame& response, const char* what) const;
    /*
      Did the tag report that it carried the operation out?
      @param response Response frame of a Write, Lock or Kill
      @param what Name of the operation, used in the warning
      @return Nothing when the tag answered with a success status
      @details The module answering says the tag was reached; the status byte inside says the
      tag actually did what it was told. That status is not one of the module's error codes, so
      a failure here reports READER_BALKED rather than a code of its own
     */
    TagResult tag_carried_it_out(const m100::Frame& response, const char* what) const;
    /*
      The error code a failure notification carries
      @param response Response frame
      @return The code, or READER_SILENT where the notification carries none
     */
    static uint8_t error_code_of(const m100::Frame& response);
    /*
      Send a tag operation, trying again when the tag simply did not answer
      @param[out] response Response frame
      @param command Command to send
      @param param Parameter
      @param param_len Parameter length
      @param what Name of the operation, used in the warnings
      @return Nothing when the module answered with a result rather than a failure
      @details Only a failure that says the tag did not answer is tried again. One where the tag
      answered with a reason of its own is reported at once, since asking again would produce
      the same reason more slowly
      @note A kill that is carried out but whose answer is lost reads as a failure here and on
      every attempt after it, because a killed tag answers nothing. There is no way to tell that
      apart from a kill that never happened
     */
    TagResult send_tag_operation(m100::Frame& response, const uint8_t command, const uint8_t* param,
                                 const uint16_t param_len, const char* what, const uint8_t answer_command = 0);

    // Frame header. A derived class for the R200 family sets this to m100::r200::FRAME_HEADER
    uint8_t _frame_header{m100::jrd::FRAME_HEADER};
    // Frame end. A derived class for the R200 family sets this to m100::r200::FRAME_END
    uint8_t _frame_end{m100::jrd::FRAME_END};

    // Bytes taken from the UART that have not been made into a frame yet. Keeping them
    // is what lets a frame that arrived in pieces be finished off by the next read
    std::vector<uint8_t> _rx{};
    // Response slot filled by route_frame while send_and_wait is pumping
    m100::Frame _response{};
    bool _response_pending{};
    uint8_t _awaiting_command{};

    // Settings this class reads. The setter passes the part it shares with
    // UHFRFIDComponent::_cfg down, which is what the base reads
    config_t _cfg_jrd4035{};
};

}  // namespace unit
}  // namespace m5
#endif
