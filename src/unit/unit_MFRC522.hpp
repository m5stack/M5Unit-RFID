/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_MFRC522.hpp
  @brief MFRC522 Unit for M5UnitUnified
*/
#ifndef M5_UNIT_RFID_UNIT_MFRC522_HPP
#define M5_UNIT_RFID_UNIT_MFRC522_HPP

#include <M5UnitComponent.hpp>
#include <nfc/nfc.hpp>  // M5Unit-NFC

namespace m5 {
namespace unit {

namespace nfc {
struct AdapterMFRC522;  // For layer
}

namespace mfrc522 {

/*!
  @enum Command
  @brief PCD command
*/
enum class Command : uint8_t {
    Idle,                //!< No action, cancels current command execution
    Mem,                 //!< Stores 25 bytes into the internal buffer
    GenerateRandomID,    //!< Generates a 10-byte random ID number
    CalcCRC,             //!< Activates the CRC coprocessor or performs a self test
    Transmit,            //!< Transmits data from the FIFO buffer
    NoCmdChange = 0x07,  //!< No command change
    Receive,             //!< Activates the receiver circuits
    Transceive = 0x0C,   //!< Transmits data from FIFO buffer to antenna and automatically activates the receiver after
                         //!< transmission
    MFAuthent = 0x0E,    //!< Performs the MIFARE standard authentication as a reader
    SoftReset,           //!< Resets the MFRC522
};

/*!
  @enum ReceiverGain
  @brief The receiver’s signal voltage gain facor
 */
enum class ReceiverGain : uint8_t {
    dB18,  //!< 18 decibel
    dB23,  //!< 23 decibel
    // 2 = dB18 (duplicated),
    // 3 = dB23 (duplicated)
    dB33 = 0x04,  //!< 33 decibel
    dB38,         //!< 38 decibel
    dB43,         //!< 43 decibel
    dB48,         //!< 48 decibel
};

/*!
  @enum Error
  @brief API error code
 */
enum class Error : uint8_t {
    OCCUR_COLLISION,           //!< Ccollision occurs
    UID_NOT_COLMPLETED,        //!< UID is not yet complete
    ARGUMENT = 0x80,           //!< Error caused by arguments (0x80)
    COMMUNICATION,             //!< Error in communication (0x81)
    REGISTER,                  //!< Error by error register value (0x82)
    TIMEOUT,                   //!< Timeout occurs (0x83)
    MIFARE_NACK,               //!< MIFARE NACK detection (0x84)
    CRC,                       //!< CRC error (0x85)
    AUTH,                      //!< Authentication error(0x86)
    CANNOT_RESOLVE_COLLISION,  //!< Collision resolution failed (0x87)
    INTERNAL = 0xFE,           //!< Internal error
    UNKNOWN  = 0xFF            //!< Misc
};

}  // namespace mfrc522

/*!
  @class UnitMFRC522
  @brief Radio frequency identification unit
  @details Operating frequency: 13.56 MHz
  @details Supported protocols: ISO14443A, MIFARE and NTAG
 */
class UnitMFRC522 : public Component {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMFRC522, 0x28);

public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! mode reg value. See also 9.3.2.2 ModeReg register
        uint8_t mode_reg{0x3D};
        //! Enable antenna on begin if true
        bool enable_antenna{true};
        //! The receiver’s signal voltage gain factor
        mfrc522::ReceiverGain receiver_gain{mfrc522::ReceiverGain::dB48};
        //! Using sotware CRC
        bool software_crc{false};
    };

    explicit UnitMFRC522(const uint8_t addr = DEFAULT_ADDRESS) : Component(addr)
    {
        auto ccfg  = component_config();
        ccfg.clock = 100 * 1000U;
        component_config(ccfg);
    }
    virtual ~UnitMFRC522()
    {
    }

    virtual bool begin() override;
    virtual void update(const bool force = false) override;

    ///@name Settings for begin
    ///@{
    /*! @brief Gets the configration */
    inline config_t config()
    {
        return _cfg;
    }
    //! @brief Set the configration
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    /*!
      @brief Software reset
      @param blocking Blocks until the reset finishes and starts up again if True
      @return True if successful
    */
    bool softReset(const bool blocking = true);

    /*!
      @brief Self test
      @return True if successful
      @warning  Blocks until the test finishes and starts up again
     */
    inline bool selfTest()
    {
        bool ret = self_test();
        return ret && begin();
    }

    ///@name Antenna
    ///@{
    /*!
      @brief Read the antenna status
      @param[out] status ON if true
      @return True if successful
     */
    bool readAntennaStatus(bool& status);
    /*!
      @brief Turn on the antenna
      @return True if successful
     */
    bool turnOnAntenna();
    /*!
      @brief Turn off the antenna
      @return True if successful
     */
    bool turnOffAntenna();
    /*!
      @brief Gets the receiver gain
      @param[out] gain Gain
      @return True if successful
     */
    bool readReceiverGain(mfrc522::ReceiverGain& gain);
    /*!
      @brief Write the receiver gain
      @param gain Gain
      @return True if successful
     */
    bool writeReceiverGain(const mfrc522::ReceiverGain gain);
    ///@}

    ///@note Timer settings
    ///@name TPrescale
    ///@{
    //! @brief Read the TPrescale
    bool readTPrescale(uint16_t& tprescale);
    //! @brief Read the TPrescale
    bool readTPrescale(float& tprescale);
    //! @brief Write the TPrescale
    bool writeTPrescale(const uint16_t tprescale);
    //! @brief Write the TPrescale
    bool writeTPrescale(const float tprescale);
    ///@}

    ///@name CRC
    ///@{
    /*!
      @brief Calculate CRC by hardware
      @param[out] CRC value
      @param buf Buffer
      @param len The length of the buffer
     */
    bool calculateCRC(uint16_t& result, const uint8_t* buf, const uint8_t len);
    /*!
      @brief Calculate CRC by software
      @param[out] CRC value
      @param buf Buffer
      @param len The length of the buffer
     */
    bool calculateSoftwareCRC(uint16_t& result, const uint8_t* buf, const uint8_t len);
    ///@}

    // ----------------------------------------------------------------
    ///@name NFC-A
    ///@{
    /*!
      @brief Transceive
      @param rx Receive buffer
      @param[in/out] rx_len in:Size of receive buffer out:actual read size
      @param tx Send buffer
      @param tx_len Size of send buffer
      @param timeout_ms Timeout(ms)
      @return True if successful
     */
    inline bool nfcaTransceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                               const uint32_t timeout_ms)
    {
        uint8_t vbits{};
        return picc_transceive(rx, rx_len, tx, tx_len, vbits);
    }
    /*!
      @brief Request for idle devices
      @param[out atqa ATQA
      @return True if successful
     */
    inline bool request(uint16_t& atqa)
    {
        return request_wakeup(atqa, true);
    }
    /*!
      @brief Wakeup for idle/halt devices
      @param[out atqa ATQA
      @return True if successful
     */
    inline bool wakeup(uint16_t& atqa)
    {
        return request_wakeup(atqa, false);
    }
    /*!
      @brief Select device with anti-collision
      @param[out] completed Completed select device?
      @param[out]  uid Selected UID
      @param lv Cascade level (1-3)
      @return True if successful
     */
    bool selectWithAnticollision(bool& completed, m5::nfc::a::UID& uid, const uint8_t lv);
    /*!
      @brief Select specific UID
      @param  uid  UID
      @return True if successful
     */
    bool select(const m5::nfc::a::UID& uid);
    /*!
      @brief Read the 1 block
      @param rx Receiver buffer
      @param[in/out] rx_len in:Size of receive buffer out:actual read size
      @param block Block address
      @return True if successful
     */
    bool readBlock(uint8_t* rx, uint16_t& rx_len, const uint8_t block);
    /*!
      @brief Write the 1 block
      @param tx Send buffer
      @param tx_len Size of send buffer
      @return True if successful
     */
    bool writeBlock(const uint8_t block, const uint8_t* tx, const uint16_t tx_len);
    /*!
      @brief Hlt for devices
      @return True if successful
     */
    bool hlt();
    ///@}

    //
    //
    //
    //
    //
    //
    //
    //

    ///@name Read/Write
    ///@{
    /*!
      @brief Read data from the block for MIFARE,NTAG
      @param uid Device UID
      @param[out] rbuf The buffer to store
      @param[in,out]  rlen in:Length of the rbuf out:Number of bytes stored
      @param addr block address or page address
      @return True if successful
      @note Since the data is read in 16-byte units, care should be taken with devices that have a page structure
      @note Therefore, for devices with page configuration, specify an addr that is a multiple of 4
      @warning buf at least 18 bytes (16 bytes data + CRC16 2 bytes)
      @pre Requires block authentication if needed
     */
    bool readBlock(const m5::nfc::a::UID& uid, uint8_t* rbuf, uint8_t& rlen, const uint8_t addr);

    /*!
      @brief Write data to the block
      @param uid Device UID
      @param addr block address or page address
      @param buf buffer
      @param len Length of the buffer (-16)
      @param safety Fail to write to out of the user memory area if true (safety measure)
      @return True if successful
      @noteg Sector structure: If the buffer is less than 16 bytes, 0x00 is padded and written
      @note Page structure: If the buffer is less than 4 bytes, 0x00 is padded and written
      @pre Requires block authentication if needed
    */
    bool writeDevice(const m5::nfc::a::UID& uid, const uint8_t addr, const uint8_t* buf, const uint8_t len0,
                     const bool safety = true);

    /*!
      @brief Write data to the block for sector structure device
      @param uid Device UID
      @param block block address
      @param buf buffer
      @param len Length of the buffer (-16)
      @param safety Fail to write to out of the user memory area if true (safety measure)
      @return True if successful
      @note If the buffer is less than 16 bytes, 0x00 is padded and written
      @pre Requires block authentication if needed
    */
    bool writeDeviceBlock(const m5::nfc::a::UID& uid, const uint8_t block, const uint8_t* buf, const uint8_t len,
                          const bool safety = true);
    /*!
      @brief Write data to the page for page structure device
      @param uid Device UID
      @param page Page address
      @param buf buffer
      @param len Length of the buffer
      @param safety Fail to write to out of the user memory area if true (safety measure)
      @return True if successful
      @note If the buffer is less than 4 bytes, 0x00 is padded and written
      @note If the buffer is greater than 4 bytes, overwrite next pages until maximum user memory area
      @note Using WRITE_UL command
    */
    bool writeDevicePage(const m5::nfc::a::UID& uid, const uint8_t page, const uint8_t* buf, const uint32_t len,
                         const bool safety = true);

#if 0
    /*!
      @brief Read data from the page for NTAG21x
      @param uid Device UID
      @param block block address or page address
      @param[out] rbuf The buffer to store
      @param[in,out]  rlen in:Length of the rbuf out:Number of bytes stored
      @return True if successful
      @note Using FAST_READ command
     */
    bool readDevicePage(const m5::nfc::a::UID& uid, uint8_t* rbuf, uint8_t& rlen, const uint8_t page);
#endif
    ///@}

    ///@warning Executable only on MIFARE classic devices
    ///@name MIFARE classic
    ///@{
    /*!
      @brief Authentication by KeyA
      @param uid Device UID
      @param block Block address
      @param key Authentication key
      @return True if successful
      @note The scope of certification is the entire sector to which the block belongs
     */
    inline bool mifareClassicAuthenticateA(
        const m5::nfc::a::UID& uid, const uint8_t block,
        const m5::nfc::a::mifare::classic::Key& key = m5::nfc::a::mifare::classic::DEFAULT_CLASSIC_KEY)
    {
        return mifare_classic_authenticate(m5::nfc::a::Command::AUTH_WITH_KEY_A, uid, block, key);
    }
    /*!
      @brief Authentication by KeyB
      @copydetails authenticateA
    */
    inline bool mifareClassicAuthenticateB(
        const m5::nfc::a::UID& uid, const uint8_t block,
        const m5::nfc::a::mifare::classic::Key& key = m5::nfc::a::mifare::classic::DEFAULT_CLASSIC_KEY)
    {
        return mifare_classic_authenticate(m5::nfc::a::Command::AUTH_WITH_KEY_B, uid, block, key);
    }
    /*!
      @brief  Exit from authenticated state for MIFARE classic
      @return True if successful
     */
    bool mifareClassicStopCrypto1();

    /*!
      @brief Change the specified block to value block
      @param uid Device UID
      @param block Block address
      @param keyA Authentication key A
      @param keyB Authentication key B
      @param readOnly read only value block if true
      @return True if successful
      @pre Requires the sector to which the block belongs authentication
      @note Writes to sector trailer, so keyAB of the target sector is required
      @warning Sector trailer access bits are changed to 011 (need auth B for R/W)
      @warning 0 or sector trailer as block address is prohibited
    */
    bool mifareClassicEnableValueBlock(const m5::nfc::a::UID& uid, const uint8_t block,
                                       const m5::nfc::a::mifare::classic::Key& keyA,
                                       const m5::nfc::a::mifare::classic::Key& keyB, const bool readOnly = false);
    /*!
      @brief Change the specified block to normal block
      @param uid Device UID
      @param block Block address
      @param keyA Authentication key A
      @param keyB Authentication key B
      @param permission Permission values to be migrated
      @return True if successful
      @pre Requires the sector to which the block belongs authentication
      Writes to sector trailer, so keyAB of the target sector is required.
      @warning 0 or sector trailer as block address is prohibited
    */
    bool mifareClassicDisableValueBlock(const m5::nfc::a::UID& uid, const uint8_t block,
                                        const m5::nfc::a::mifare::classic::Key& keyA,
                                        const m5::nfc::a::mifare::classic::Key& keyB, const uint8_t permission = 0x00);
    /*!
      @brief Increments the contents of a block and stores the result in the internal Transfer Buffer
      @param uid Device UID
      @param block Block address
      @param delta incremental value
      @return True if successful
      @note Note that the value of the block itself has not yet changed after execution
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
      @post Applied to the actual block by mifareTransfer
    */
    bool mifareClassicIncrement(const m5::nfc::a::UID& uid, const uint8_t block, const uint32_t delta);
    /*!
      @brief Decrements the contents of a block and stores the result in the internal Transfer Buffer
      @param uid Device UID
      @param block Block address
      @param delta decremental value
      @return True if successful
      @note Note that the value of the block itself has not yet changed after execution
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
      @post Applied to the actual block by mifareTransfer
    */
    bool mifareClassicDecrement(const m5::nfc::a::UID& uid, const uint8_t block, const uint32_t delta);
    /*!
      @brief Writes the contents of the internal Transfer Buffer to a value block
      @param uid Device UID
      @param block Block address
      @return True if successful
      @pre Requires the sector to which the block belongs authentication
     */
    bool mifareClassicTransfer(const m5::nfc::a::UID& uid, const uint8_t block);
    /*!
      @brief Moves the contents of a block into the internal Transfer Buffer
      @param uid Device UID
      @param block Block address
      @return True if successful
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
     */
    bool mifareClassicRestore(const m5::nfc::a::UID& uid, const uint8_t block);
    /*!
      @brief Read the value assuming the specified block is the value block
      @param uid Device UID
      @param[out] value value
      @param block Block address
      @return True if successful
      @pre Requires block authentication if needed
    */
    bool mifareClassicReadValue(const m5::nfc::a::UID& uid, int32_t& value, const uint8_t block);
    /*!
      @brief Writes as a specified value block
      @param uid Device UID
      @param block Block address
      @param value value
      @return True if successful
      @pre Requires block authentication if needed
     */
    bool mifareClassicWriteValue(const m5::nfc::a::UID& uid, const uint8_t block, const int32_t value);
    ///@}

    ///@warning Executable only on page structure devices
    ///@name NFC

    /*!

     */
    bool ntagGetVersion(uint8_t info[10]);

    /*!
      @brief Write change to NFC-A Type-2 format
      @return True if NTAG or NTAG format Light/C
    */
    bool nfcWriteChangeToNTAGFormat(const m5::nfc::a::UID& uid);

    /*!
      @brief Read and calculation of required size
      @param uid Device UID
      @patam[out] len Required length
      @return True if successful
     */
    bool nfcReadRequiredSize(const m5::nfc::a::UID& uid, uint32_t& len);
    /*!
      @brief Read the NFC NDEF message
      @param uid Device UID
      @param buf[out] Buffer in NDEF Message or NDEF Record format
      @param blen[in,out] in: Buffer length out:Read length
      @return True if successful or NTAG device
     */
    bool nfcReadDevice(const m5::nfc::a::UID& uid, uint8_t* buf, uint32_t& len);
    /*!
      @brief Write the NFC NDEF message
      @param uid Device UID
      @param buf Buffer in NDEF Message or NDEF Record format
      @param blen Buffer length
      @return True if successful or NTAG device
      @warning Already existing data will be overwritten
      @warning When making additions or changes to already existing data, read and edit first
     */
    bool nfcWriteDevice(const m5::nfc::a::UID& uid, const uint8_t* buf, const uint32_t blen);
    ///@}

protected:
    // register operation
    bool set_register_bit(const uint8_t reg, const uint8_t bit);
    bool clear_register_bit(const uint8_t reg, const uint8_t bit);
    bool read_register_with_align(const uint8_t reg, uint8_t* buf, const uint8_t len, const uint8_t align);
    bool write_pcd_command(const mfrc522::Command cmd);

    bool reset_baud_rates();
    bool flush_fifo_buffer();
    bool wait_comm_irq(const uint8_t irq, const uint32_t duration);
    bool wait_div_irq(const uint8_t irq, const uint32_t duration);

    // PICC
    bool picc_send(const mfrc522::Command cmd, const uint8_t* buf, const uint8_t len, const uint8_t txLast = 0,
                   const uint8_t rxAlign = 0);
    bool picc_transceive(uint8_t* rbuf, uint16_t& rlen, const uint8_t* buf, const uint16_t len, uint8_t& validBits,
                         const uint8_t rxAlign = 0, const bool crc = false, uint8_t* err = nullptr);

    bool picc_haltA();

    // Read/Wrte
    bool read_block(uint8_t* rbuf, uint8_t& rlen, const uint8_t addr);
    bool write_block(const uint8_t block, const uint8_t* buf, const uint8_t len);
    bool write_page(const uint8_t page, const uint8_t* buf, const uint8_t len);

    // NFC-A
    bool request_wakeup(uint16_t& atqa, const bool request);
    bool anti_collision(const uint8_t cascadeLevel, uint8_t* buf);

    // MIFARE classic
    bool mifare_classic_authenticate(const m5::nfc::a::Command cmd, const m5::nfc::a::UID& uid, const uint8_t block,
                                     const m5::nfc::a::mifare::classic::Key& key);
    bool mifare_classic_transceive(const m5::nfc::a::Command cmd, const uint8_t block);
    bool mifare_classic_transceive(const uint8_t* buf, const uint8_t len, const bool usingtimeout = false);

    // NTAG
    bool ntag_check_format(const m5::nfc::a::UID& uid);
    bool ntag_fast_read(uint8_t* rbuf, uint16_t& rlen, const uint8_t saddr, const uint8_t eaddr);
    bool ntag_calclate_ndef_message_size(const m5::nfc::a::UID& uid, uint32_t& sz, const uint8_t targetTagBit = 0x06);

    // crc
    inline bool calculate_crc(uint16_t& result, const uint8_t* buf, const uint8_t len)
    {
        return (this->*(_cfg.software_crc ? &UnitMFRC522::calculateSoftwareCRC : &UnitMFRC522::calculateCRC))(result,
                                                                                                              buf, len);
    }

    virtual bool self_test();

protected:
    config_t _cfg{};
};

///@cond
namespace mfrc522 {
namespace command {
// Command and status
constexpr uint8_t COMMAND_REG{0x01};
constexpr uint8_t COM_IEN_REG{0x02};
constexpr uint8_t COM_IRQ_REG{0x04};
constexpr uint8_t DIV_IRQ_REG{0x05};
constexpr uint8_t ERROR_REG{0x06};
constexpr uint8_t STATUS2_REG{0x08};
constexpr uint8_t FIFO_DATA_REG{0x09};
constexpr uint8_t FIFO_LEVEL_REG{0x0A};
constexpr uint8_t WATER_LEVEL_REG{0x0B};
constexpr uint8_t CONTROL_REG{0x0C};
constexpr uint8_t BIT_FRAMING_REG{0x0D};
constexpr uint8_t COLL_REG{0x0E};

// Communication
constexpr uint8_t MODE_REG{0x11};
constexpr uint8_t TX_MODE_REG{0x12};
constexpr uint8_t RX_MODE_REG{0x13};
constexpr uint8_t TX_CONTROL_REG{0x14};
constexpr uint8_t TX_ASK_REG{0x15};
constexpr uint8_t RX_SEL_REG{0x16};
constexpr uint8_t RX_THRESHOULD_REG{0x18};
constexpr uint8_t DEMOD_REG{0x19};
constexpr uint8_t MF_TX_REG{0x1C};
constexpr uint8_t MF_RX_REG{0x1D};

// Configuration
constexpr uint8_t CRC_RESULT_REGH{0x21};
constexpr uint8_t CRC_RESULT_REGL{0x22};
constexpr uint8_t MOD_WIDTH_REG{0x24};
constexpr uint8_t RFC_FG_REG{0x26};
constexpr uint8_t GSN_REG{0x27};
constexpr uint8_t CW_GSP_REG{0x28};
constexpr uint8_t MOD_GSP_REG{0x29};
constexpr uint8_t TMODE_REG{0x2A};
constexpr uint8_t TPRESCALER_REG_L{0x2B};
constexpr uint8_t TRELOAD_REG_H{0x2C};
constexpr uint8_t TRELOAD_REG_L{0x2D};

// Test
constexpr uint8_t AUTO_TEST_REG{0x36};
constexpr uint8_t VERSION_REG{0x37};

}  // namespace command
}  // namespace mfrc522
///@endcond

}  // namespace unit
}  // namespace m5
#endif
