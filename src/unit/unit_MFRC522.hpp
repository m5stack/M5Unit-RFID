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
#include "rfid/rfid.hpp"
#include "rfid/mifare.hpp"
#include <m5_utility/stl/expected.hpp>
#include <array>

namespace m5 {
namespace unit {

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
    OCCUR_COLLISION,     //!< Ccollision occurs
    UID_NOT_COLMPLETED,  //!< UID is not yet complete
    ARGUMENT = 0x80,     //!< Error caused by arguments (0x80)
    COMMUNICATION,       //!< Error in communication (0x81)
    REGISTER,            //!< Error by error register value (0x82)
    TIMEOUT,             //!< Timeout occurs (0x83)
    MIFARE_NACK,         //!< MIFARE NACK detection (0x84)
    CRC,                 //!< CRC error (0x85)
    AUTH,                //!< Authentication error(0x86)
    INTERNAL = 0xFE,     //!< Internal error
    UNKNOWN  = 0xFF      //!< Misc
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
      @brief API return value
      @note The user can treat it as if it were a bool type
    */
    using result_t = m5::stl::expected<void, mfrc522::Error>;

    /*!
      @typedef MifareKey
      @brief MIFARE key
    */
    using MifareKey = m5::rfid::mifare::Key;
    /*!
      @typedef UID
      @brief Device UID
     */
    using UID = m5::rfid::UID;

    //! @brief KEY of factory default
    static const MifareKey DEFAULT_CLASSIC_KEY;

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

    ///@name PICC
    /*!
      @brief Detect IDLE device
      @return True if successful
      @note Send REQA command
      @note Device status changes from IDLE to READY
     */
    result_t detectIdleDevice();
    /*!
      @brief Detect IDLE and HALT device
      @return True if successful
      @note Send WUPA command
      @note Device status changes from IDLE to READY
      @note Device status changes from HALT to READY*
     */
    result_t detectDevice();
    /*!
      @brief Activate device
      @param[out] uid UID of the activated device
      @return True if successful
      @note Resolve collisions and send SEL command
      @note Device status changes from READY to ACTIVE
      @note Device status changes from READY* to ACTIVE*
      @note ISO14443-4 processing can be performed on devices in ACTIVE state
      @warning Whenever an operation on an activated device is no longer required, it must be deactivated
    */
    result_t activateDevice(UID& uid);
    /*!
      @brief Deactivate device
      @param uid Target UID
      @return True if successful
      @note Send HLTA command and stop crypt1
      @note Device status changes from ACTIVE to HALT
      @note Device status changes from ACTIVE* to HALT
    */
    result_t deactivateDevice();
    ///@}

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
    result_t readDevice(const UID& uid, uint8_t* rbuf, uint8_t& rlen, const uint8_t addr);

    /*!
      @brief Write data to the block
      @param uid Device UID
      @param addr block address or page address
      @param buf buffer
      @param len Length of the buffer (-16)
      @param safety Fail to write to our of the user memory area if true (safety measure)
      @return True if successful
      @noteg Sector structure: If the buffer is less than 16 bytes, 0x00 is padded and written
      @note Page structure: If the buffer is less than 4 bytes, 0x00 is padded and written
      @pre Requires block authentication if needed
    */
    result_t writeDevice(const UID& uid, const uint8_t addr, const uint8_t* buf, const uint8_t len,
                         const bool safety = true);

    /*!
      @brief Write data to the block for sector structure device
      @param uid Device UID
      @param block block address
      @param buf buffer
      @param len Length of the buffer (-16)
      @param safety Fail to write to our of the user memory area if true (safety measure)
      @return True if successful
      @note If the buffer is less than 16 bytes, 0x00 is padded and written
      @pre Requires block authentication if needed
    */
    result_t writeDeviceBlock(const UID& uid, const uint8_t block, const uint8_t* buf, const uint8_t len,
                              const bool safety = true);
    /*!
      @brief Write data to the page for page structure device
      @param uid Device UID
      @param page Page address
      @param buf buffer
      @param len Length of the buffer
      @param safety Fail to write to our of the user memory area if true (safety measure)
      @return True if successful
      @note If the buffer is less than 4 bytes, 0x00 is padded and written
      @note If the buffer is greater than 4 bytes, overwrite next pages until maximum user memory area
      @note Using WRITE_UL command
    */
    result_t writeDevicePage(const UID& uid, const uint8_t page, const uint8_t* buf, const uint32_t len,
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
    result_t readDevicePage(const UID& uid, uint8_t* rbuf, uint8_t& rlen, const uint8_t page);
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
    inline result_t mifareAuthenticateA(const UID& uid, const uint8_t block, const MifareKey& key = DEFAULT_CLASSIC_KEY)
    {
        return mifare_authenticate(m5::rfid::Command::AUTH_WITH_KEY_A, uid, block, key);
    }
    /*!
      @brief Authentication by KeyB
      @copydetails authenticateA
    */
    inline result_t mifareAuthenticateB(const UID& uid, const uint8_t block, const MifareKey& key = DEFAULT_CLASSIC_KEY)
    {
        return mifare_authenticate(m5::rfid::Command::AUTH_WITH_KEY_B, uid, block, key);
    }
    /*!
      @brief  Exit from authenticated state
      @return True if successful
     */
    bool mifareStopCrypto1();

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
    result_t mifareEnableValueBlock(const UID& uid, const uint8_t block, const MifareKey& keyA, const MifareKey& keyB,
                                    const bool readOnly = false);
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
    result_t mifareDisableValueBlock(const UID& uid, const uint8_t block, const MifareKey& keyA, const MifareKey& keyB,
                                     const uint8_t permission = 0x00);
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
    result_t mifareIncrement(const UID& uid, const uint8_t block, const uint32_t delta);
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
    result_t mifareDecrement(const UID& uid, const uint8_t block, const uint32_t delta);
    /*!
      @brief Writes the contents of the internal Transfer Buffer to a value block
      @param uid Device UID
      @param block Block address
      @return True if successful
      @pre Requires the sector to which the block belongs authentication
     */
    result_t mifareTransfer(const UID& uid, const uint8_t block);
    /*!
      @brief Moves the contents of a block into the internal Transfer Buffer
      @param uid Device UID
      @param block Block address
      @return True if successful
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
     */
    result_t mifareRestore(const UID& uid, const uint8_t block);
    /*!
      @brief Read the value assuming the specified block is the value block
      @param uid Device UID
      @param[out] value value
      @param block Block address
      @return True if successful
      @pre Requires block authentication if needed
    */
    result_t mifareReadValue(const UID& uid, int32_t& value, const uint8_t block);
    /*!
      @brief Writes as a specified value block
      @param uid Device UID
      @param block Block address
      @param value value
      @return True if successful
      @pre Requires block authentication if needed
     */
    result_t mifareWriteValue(const UID& uid, const uint8_t block, const int32_t value);
    ///@}

    ///@warning Executable only on page structure devices
    ///@name NFC
    /*!
      @brief Write change to NFC-A Type-2 format
      @return True if NTAG or NTAG format Light/C
    */
    result_t nfcWriteChangeToNTAGFormat(const UID& uid);

    /*!
      @brief Read and calculation of required size
      @param uid Device UID
      @patam[out] len Required length
      @return True if successful
     */
    result_t nfcReadRequiredSize(const UID& uid, uint32_t& len);
    /*!
      @brief Read the NFC NDEF message
      @param uid Device UID
      @param buf[out] Buffer in NDEF Message or NDEF Record format
      @param blen[in,out] in: Buffer length out:Read length
      @return True if successful or NTAG device
     */
    result_t nfcReadDevice(const UID& uid, uint8_t* buf, uint32_t& len);
    /*!
      @brief Write the NFC NDEF message
      @param uid Device UID
      @param buf Buffer in NDEF Message or NDEF Record format
      @param blen Buffer length
      @return True if successful or NTAG device
      @warning Already existing data will be overwritten
      @warning When making additions or changes to already existing data, read and edit first
     */
    result_t nfcWriteDevice(const UID& uid, const uint8_t* buf, const uint32_t blen);
    ///@}

    ///@name Dump
    /*!
      @brief Dump to serial
      @param uid Device UID
      @param keyA Key used for authentication A for Classic
      @return True if successful
      @warning All blocks must be readable with the specified key if needed
     */
    result_t dumpDevice(const UID& uid, const MifareKey& keyA = DEFAULT_CLASSIC_KEY);
    /*!
      @brief Dump specific block/page to serial
      @param uid Device UID
      @param block block address(Classic) or page address
      @return True if successful
      @pre Requires block authentication if needed
     */
    result_t dumpDevice(const UID& uid, const uint8_t addr);
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
    result_t picc_send(const mfrc522::Command cmd, const uint8_t* buf, const uint8_t len, const uint8_t txLast = 0,
                       const uint8_t rxAlign = 0);
    result_t picc_transceive(uint8_t* rbuf, uint8_t& rlen, const uint8_t* buf, const uint8_t len, uint8_t& validBits,
                             const uint8_t rxAlign = 0, const bool crc = false);

    result_t picc_requestA(uint16_t& atqa);
    result_t picc_wakeupA(uint16_t& atqa);
    result_t picc_to_ready(const m5::rfid::Command piccCommand, uint16_t& atqa);
    result_t picc_select(UID& uid, const uint8_t cascadeLevel);
    result_t picc_anti_collision(const uint8_t cascadeLevel, uint8_t* buf);
    result_t picc_haltA();

    result_t activate(UID& uid);
    result_t reactivate(UID& uid, const UID& prev);

    // Read/Wrte
    result_t read_block(uint8_t* rbuf, uint8_t& rlen, const uint8_t addr);
    result_t write_block(const uint8_t block, const uint8_t* buf, const uint8_t len);
    result_t write_page(const uint8_t page, const uint8_t* buf, const uint8_t len);

    // MIFARE
    result_t mifare_authenticate(const m5::rfid::Command cmd, const UID& uid, const uint8_t block,
                                 const MifareKey& key);
    result_t mifare_transceive(const m5::rfid::Command cmd, const uint8_t block);
    result_t mifare_transceive(const uint8_t* buf, const uint8_t len, const bool usingtimeout = false);

    // NTAG
    bool ntag_check_format(const UID& uid);
    result_t ntag_get_version(uint8_t* rbuf, uint8_t& rlen);
    result_t ntag_fast_read(uint8_t* rbuf, uint8_t& rlen, const uint8_t saddr, const uint8_t eaddr);
    result_t ntag_calclate_ndef_message_size(const UID& uid, uint32_t& sz, const uint8_t targetTagBit = 0x06);

    // dump
    result_t dump_sector_structure(const UID& uid, const MifareKey& key);
    result_t dump_sector(const uint8_t sector);
    result_t dump_page_structure(const uint8_t maxPage);
    result_t dump_page(const uint8_t page);

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
