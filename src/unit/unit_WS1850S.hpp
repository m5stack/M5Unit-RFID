/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_WS1850S.hpp
  @brief WS1850S Unit for M5UnitUnified
*/
#ifndef M5_UNIT_RFID2_UNIT_WS1850S_HPP
#define M5_UNIT_RFID2_UNIT_WS1850S_HPP

#include "unit_MFRC522.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitWS1850S
  @brief Radio frequency identification unit
  @details Functionally compatible with MFRC522. Supports NFC-A (always) and NFC-B (if PN512-compatible silicon).
 */
class UnitWS1850S : public UnitMFRC522 {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitWS1850S, 0x28);

public:
    /*!
      @struct config_t
      @brief Settings for begin / configureNFCMode
      @details Adds NFC-B antenna driver / receiver tuning values on top of UnitMFRC522::config_t.
     */
    struct config_t : public UnitMFRC522::config_t {
        //! NFC-B ASK modulation depth (0=shallowest, 15=deepest).
        //! Mapped internally to ModGsP register.
        uint8_t nfcb_ask_depth{11};
    };

    /*!
      @brief Constructor
      @param addr I2C address
     */
    explicit UnitWS1850S(const uint8_t addr = DEFAULT_ADDRESS) : UnitMFRC522(addr)
    {
    }
    /*!
      @brief Destructor
     */
    virtual ~UnitWS1850S()
    {
    }

    /*!
      @brief Begin the unit
      @return True if successful
     */
    virtual bool begin() override;

    ///@name Settings for begin / configureNFCMode
    ///@{
    /*! @brief Gets the configuration */
    inline config_t config() const
    {
        return _cfg;
    }
    /*! @brief Set the configuration */
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
        UnitMFRC522::config(static_cast<const UnitMFRC522::config_t&>(cfg));
    }
    ///@}

    ///@name NFC mode
    ///@{
    /*!
      @brief Gets the current operating mode
      @return NFC::A or NFC::B
     */
    inline m5::nfc::NFC NFCMode() const
    {
        return _mode;
    }

    /*!
      @brief Configure NFC mode
      @param mode NFC::A or NFC::B
      @return True if successful
      @note NFC::B requires PN512-compatible silicon. Antenna tuning values are taken from config_t.
     */
    bool configureNFCMode(const m5::nfc::NFC mode) override;
    ///@}

    ///@name NFC-B (ISO/IEC 14443 Type B)
    ///@{
    /*!
      @brief Transceive an NFC-B frame
      @param rx Receive buffer (returned with trailing 2-byte CRC_B intact)
      @param[in,out] rx_len in:Size of rx buffer  out:Actual received length
      @param tx Transmit buffer (without CRC; CRC_B is appended internally)
      @param tx_len Length of tx
      @param timeout_ms Timeout in milliseconds
      @return True if successful
      @note Software CRC_B is used for transmission. Both software and hardware CRC are
            also computed and compared in M5_LIB_LOGI for silicon compatibility diagnosis.
     */
    bool nfcbTransceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                        const uint32_t timeout_ms);

    /*!
      @brief Read the TypeBReg (PN512 Page 1, address 0x1E)
      @param[out] value 8-bit raw value (RxSOFReq / EOFSOFWidth / TxEGT / EOFSOFAdjust)
      @return True if successful
     */
    bool readTypeBReg(uint8_t& value);
    /*!
      @brief Write the TypeBReg (PN512 Page 1, address 0x1E)
      @param value 8-bit raw value. bit[5] is RFU (must be 0) per PN512 datasheet 8.2.2.15.
      @return True if successful, false if reserved bit set or I2C error
      @note Effective only on PN512-compatible silicon (e.g. WS1850S).
     */
    bool writeTypeBReg(const uint8_t value);
    ///@}

protected:
    virtual bool self_test() override;

    /*! @brief Switch chip to NFC-A mode (default) */
    bool configure_nfca();
    /*! @brief Switch chip to NFC-B mode (PN512-compatible silicon only) */
    bool configure_nfcb();

protected:
    config_t _cfg{};
    m5::nfc::NFC _mode{m5::nfc::NFC::A};
};

}  // namespace unit
}  // namespace m5
#endif
