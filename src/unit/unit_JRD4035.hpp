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
    UnitJRD4035() : UHFRFIDComponent(DEFAULT_ADDRESS)
    {
    }
    virtual ~UnitJRD4035() = default;

    virtual bool begin() override;

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
    virtual bool readQueryParameters(m5::uhf::QueryParameters& qp) override;
    virtual bool writeQueryParameters(const m5::uhf::QueryParameters& qp) override;
    virtual bool writeAutoSleepTime(const uint8_t minutes) override;
    virtual bool sleep() override;
    virtual bool wake() override;
    virtual bool writeOperatingChannels(const std::vector<uint8_t>& channels) override;
    virtual bool readBlockingSignal(m5::uhf::ChannelLevels& levels) override;
    virtual bool readChannelRSSI(m5::uhf::ChannelLevels& levels) override;
    virtual bool readSelectParameter(m5::uhf::SelectParameter& sp) override;
    ///@name Receiver tuning, specific to this chip
    ///@{
    //! @brief Read the receiver demodulator settings
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

protected:
    virtual bool pump(const uint32_t timeout_ms) override;
    virtual bool start_polling_command(const uint16_t count) override;
    virtual bool stop_polling_command() override;

    virtual bool write_select_parameter(const m5::uhf::Bank bank, const uint32_t pointer_bits, const uint8_t* mask,
                                        const size_t mask_len) override;
    virtual bool write_select_enabled(const bool enable) override;
    virtual bool read_tag_memory(std::vector<uint8_t>& out, const m5::uhf::Bank bank, const uint16_t word_address,
                                 const uint16_t word_count, const uint32_t access_password) override;
    virtual bool write_tag_memory(const m5::uhf::Bank bank, const uint16_t word_address, const uint8_t* data,
                                  const size_t len, const uint32_t access_password) override;
    virtual bool lock_tag_memory(const uint32_t payload, const uint32_t access_password) override;
    virtual bool kill_tag(const uint32_t kill_password) override;

    //! @brief Send a command frame
    bool send_command(const uint8_t command, const uint8_t* param, const uint16_t param_len);
    /*!
      @brief Send a command and wait for its response, routing notifications to the tag queue
      @param[out] response Response frame
      @param command Command to send
      @param param Parameter
      @param param_len Parameter length
      @param timeout_ms How long to wait, or 0 to wait config_t::command_timeout_ms
      @return True if successful
      @note The module answers every command under the code it was sent, including the two the
      protocol document says come back as something else
     */
    bool send_and_wait(m100::Frame& response, const uint8_t command, const uint8_t* param, const uint16_t param_len,
                       const uint32_t timeout_ms = 0);
    //! @brief Read one frame from the stream. Returns false when nothing is pending
    bool read_frame(m100::Frame& out, const uint32_t timeout_ms);
    //! @brief Throw away everything received, in the driver and in the staging buffer alike
    void flush_rx();
    //! @brief Route a received frame
    void route_frame(const m100::Frame& f);
    //! @brief Read the module information for one kind (0x00 hardware / 0x01 software / 0x02 manufacturer)
    bool read_module_information_kind(std::string& out, const uint8_t kind);
    //! @brief Run one of the channel scans and decode its contiguous range of levels
    bool read_channel_levels(m5::uhf::ChannelLevels& levels, const uint8_t command);
    /*!
      @brief Did the module answer with a result rather than a failure?
      @param response Response frame
      @param what Name of the operation, used in the warning
      @return True when the response carries a result
      @details send_and_wait also returns true for a failure notification, since the module
      answering at all is what it waits for. Every tag operation has to tell the two apart
     */
    bool succeeded(const m100::Frame& response, const char* what) const;
    /*!
      @brief Did the tag report that it carried the operation out?
      @param response Response frame of a Write, Lock or Kill
      @param what Name of the operation, used in the warning
      @return True when the tag answered with a success status
      @details The module answering says the tag was reached; the status byte inside says the
      tag actually did what it was told
     */
    bool tag_carried_it_out(const m100::Frame& response, const char* what) const;
    /*!
      @brief Send a tag operation, trying again when the tag simply did not answer
      @param[out] response Response frame
      @param command Command to send
      @param param Parameter
      @param param_len Parameter length
      @param what Name of the operation, used in the warnings
      @return True when the module answered with a result rather than a failure
      @details Only a failure that says the tag did not answer is tried again. One where the tag
      answered with a reason of its own is reported at once, since asking again would produce
      the same reason more slowly
      @note A kill that is carried out but whose answer is lost reads as a failure here and on
      every attempt after it, because a killed tag answers nothing. There is no way to tell that
      apart from a kill that never happened
     */
    bool send_tag_operation(m100::Frame& response, const uint8_t command, const uint8_t* param,
                            const uint16_t param_len, const char* what);

    //! Frame header. A derived class for the R200 family sets this to m100::r200::FRAME_HEADER
    uint8_t _frame_header{m100::jrd::FRAME_HEADER};
    //! Frame end. A derived class for the R200 family sets this to m100::r200::FRAME_END
    uint8_t _frame_end{m100::jrd::FRAME_END};

    //! Response slot filled by route_frame while send_and_wait is pumping
    //! @brief Bytes taken from the UART that have not been made into a frame yet. Keeping them
    //! is what lets a frame that arrived in pieces be finished off by the next read
    std::vector<uint8_t> _rx{};
    m100::Frame _response{};
    bool _response_pending{};
    uint8_t _awaiting_command{};
};

}  // namespace unit
}  // namespace m5
#endif
