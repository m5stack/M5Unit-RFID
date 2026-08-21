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

#include "jrd4035_frame.hpp"
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
    ///@}

protected:
    virtual bool pump(const uint32_t timeout_ms) override;
    virtual bool start_polling_command(const uint16_t count) override;
    virtual bool stop_polling_command() override;

    //! @brief Send a command frame
    bool send_command(const uint8_t command, const uint8_t* param, const uint16_t param_len);
    //! @brief Send a command and wait for its response, routing notifications to the tag queue
    bool send_and_wait(jrd4035::Frame& response, const uint8_t command, const uint8_t* param, const uint16_t param_len,
                       const uint32_t timeout_ms = 1000U);
    //! @brief Read one frame from the stream. Returns false when nothing is pending
    bool read_frame(jrd4035::Frame& out, const uint32_t timeout_ms);
    //! @brief Route a received frame
    void route_frame(const jrd4035::Frame& f);
    //! @brief Read the module information for one kind (0x00 hardware / 0x01 software / 0x02 manufacturer)
    bool read_module_information_kind(std::string& out, const uint8_t kind);

    //! Frame header. Derived classes for the R200 family override this with 0xAA
    uint8_t _frame_header{jrd4035::FRAME_HEADER};
    //! Frame end. Derived classes for the R200 family override this with 0xDD
    uint8_t _frame_end{jrd4035::FRAME_END};

    //! Response slot filled by route_frame while send_and_wait is pumping
    jrd4035::Frame _response{};
    bool _response_pending{};
    uint8_t _awaiting_command{};
};

}  // namespace unit
}  // namespace m5
#endif
