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
#include <m5_utility/stl/extension.hpp>
#include <m5_utility/stl/optional.hpp>
#include <array>

namespace m5 {
namespace unit {

namespace mfrc522 {

/*!
  @enum Command
  @brief PCD command
*/
enum class Command : uint8_t {
    Idle,
    Mem,
    GenerateRandomID,
    CalcCRC,
    Transmit,
    NoCmdChange = 0x07,
    Receive,
    Transceive = 0x0C,
    MFAuthent  = 0x0E,
    SoftReset,
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
    ARGUMENT = 0x80,     //!< Error caused by arguments
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
    using result_t = m5::stl::expected<void, mfrc522::Error>;

    /*!
      @typedef MifareKey
      @brief MIFARE key
    */
    using MifareKey = m5::rfid::mifare::Key;
    /*!
      @typedef UID
      @brief MIFARE UID
     */
    using UID = m5::rfid::mifare::UID;

    //! @brief KEY of factory default
    static const MifareKey DEFAULT_CLASSIC_KEY;

    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! mode reg value. See also 9.3.2.2 ModeReg register
        uint8_t mode_reg{0x3D};
        //
        // float timer{};
        //! Enable antenna on begin if true
        bool enable_antenna{true};
        //! the receiver’s signal voltage gain facto
        mfrc522::ReceiverGain receiver_gain{mfrc522::ReceiverGain::dB48};
        // mfrc522::ReceiverGain receiver_gain{mfrc522::ReceiverGain::dB33};
        //! Using sotware CRC
        bool software_crc{true};
        // bool software_crc{false};
    };

    explicit UnitMFRC522(const uint8_t addr = DEFAULT_ADDRESS) : Component(addr)
    {
        auto ccfg  = component_config();
        ccfg.clock = 400 * 1000U;
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

    //    bool enablePowerDownMode();
    //    bool disablePowerDownMode();

    /*!
      @brief Software reset
      @param blocking Blocks until the reset finishes and starts up again if True
      @return True if successful
    */
    bool softReset(const bool blocking = true);

    virtual bool selfTest();

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

    ///@name TPrescale
    ///@{
    bool readTPrescale(uint16_t& tprescale);
    bool readTPrescale(float& tprescale);
    bool writeTPrescale(const uint16_t tprescale);
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
    */
    result_t activateDevice(UID& uid);
    /*!
      @brief Deactivate device
      @param uid target UID
      @return True if successful
      @note Send HLTA command and stop crypt1
      @note Device status changes from ACTIVE to HALT
    */
    result_t deactivateDevice();
    /*!
      @brief Authentication by KeyA
      @param uid PICC UID
      @param block Block address
      @param key Authentication key
      @return True if successful
      @note The scope of certification is the entire sector to which the block belongs
     */
    inline result_t authenticateA(const UID& uid, const uint8_t block, const MifareKey& key = DEFAULT_CLASSIC_KEY)
    {
        return picc_authenticate(m5::rfid::Command::AUTH_WITH_KEY_A, uid, block, key);
    }
    /*!
      @brief Authentication by KeyB
      @copydetails authenticateA
    */
    inline result_t authenticateB(const UID& uid, const uint8_t block, const MifareKey& key = DEFAULT_CLASSIC_KEY)
    {
        return picc_authenticate(m5::rfid::Command::AUTH_WITH_KEY_B, uid, block, key);
    }
    /*!
      @brief Authentication by KeyA and KeyB
      @param uid PICC UID
      @param block Block address
      @param keyA Authentication key A
      @param keyB Authentication key B
      @return True if successful
      @note The scope of certification is the entire sector to which the block belongs
     */
    inline result_t authenticateAB(const UID& uid, const uint8_t block, const MifareKey& keyA = DEFAULT_CLASSIC_KEY,
                                   const MifareKey& keyB = DEFAULT_CLASSIC_KEY)
    {
        result_t result = authenticateA(uid, block, keyA);
        return result ? authenticateB(uid, block, keyB) : result;
    }
    ///@}

    ///@warning Executable only on activated devices
    ///@name MIFARE
    ///@{
    /*!
      @brief Dump all sectors to serial
      @param uid Device UID
      @param keyA Key used for authentication A
      @return True if successful
      @warning All blocks must be readable with the specified key
     */
    result_t mifareDump(const UID& uid, const MifareKey& keyA = DEFAULT_CLASSIC_KEY);
    /*!
      @brief Dump specific block/page to serial
      @param uid Device UID
      @param block block address(Classic) page address(UltraLight/C)
      @return True if successful
      @pre Requires block authentication
     */
    result_t mifareDumpBlock(const UID& uid, const uint8_t block);
    /*!
      @brief Read data from the block
      @param block block address(Classic) or page address(UltraLight/C)
      @param[out] rbuf The buffer to store
      @param[in,out]  rlen in:Length of the rbuf out:Number of bytes stored
      @return True if successful
      @warning buf at least 18 bytes (16 bytes data + CRC16 2 bytes)
      @pre Requires block authentication
     */
    result_t mifareRead(uint8_t* rbuf, uint8_t& rlen, const uint8_t block);
    /*!
      @brief Write data to the block
      @param block block address(Classic)
      @param buf buffer
      @param len Length of the buffer (1-16)
      @param safety  Fail to write to sector trailer or 0 if true (safety measure)
      @return True if successful
      @noteg If the buffer is less than 16 bytes, 0x00 is padded and written
      @pre Requires block authentication
    */
    result_t mifareWrite(const uint8_t block, const uint8_t* buf, const uint8_t len, const bool safety = true);
    /*!
      @brief Write data to the block for UltraLight/C
      @param page Page address
      @param buf buffer
      @param len Length of the buffer(1-4)
      @return True if successful
      @note If the buffer is less than 4 bytes, 0x00 is padded and written
      @warning Page address 0-3 are system pages,
      @warning Page address 40-47 are system pages if UltraLighC
    */
    result_t mifareWriteUL(const uint8_t page, const uint8_t* buf, const uint8_t len);
    /*!
      @brief  Exit from authenticated state
      @return True if successful
     */
    bool mifareStopCrypto1();
    /*!
      @brief Change the specified block to value block
      @param block Block address
      @param keyA Authentication key A
      @param keyB Authentication key B
      @param readOnly read only value block if true
      @return True if successful
      @pre Requires block authentication
      @note Writes to sector trailer, so keyAB of the target sector is required
      @warning 0 or sector trailer as block address is prohibited
    */
    result_t mifareEnableValueBlock(const uint8_t block, const MifareKey& keyA, const MifareKey& keyB,
                                    const bool readOnly = false);
    /*!
      @brief Change the specified block to normal block
      @param block Block address
      @param keyA Authentication key A
      @param keyB Authentication key B
      @param permission Permission values to be migrated
      @return True if successful
      @pre Requires the sector to which the block belongs authentication
      Writes to sector trailer, so keyAB of the target sector is required.
      @warning 0 or sector trailer as block address is prohibited
    */
    result_t mifareDisableValueBlock(const uint8_t block, const MifareKey& keyA, const MifareKey& keyB,
                                     const uint8_t permission = 0x00);
    /*!
      @brief Increments the contents of a block and stores the result in the internal Transfer Buffer
      @param block Block address
      @param delta incremental value
      @return True if successful
      @note Note that the value of the block itself has not yet changed after execution
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
      @post Applied to the actual block by mifareTransfer
    */
    result_t mifareIncrement(const uint8_t block, const uint32_t delta);
    /*!
      @brief Decrements the contents of a block and stores the result in the internal Transfer Buffer
      @param block Block address
      @param delta decremental value
      @return True if successful
      @note Note that the value of the block itself has not yet changed after execution
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
      @post Applied to the actual block by mifareTransfer
    */
    result_t mifareDecrement(const uint8_t block, const uint32_t delta);
    /*!
      @brief Writes the contents of the internal Transfer Buffer to a value block
      @param block Block address
      @return True if successful
      @pre Requires the sector to which the block belongs authentication
     */
    result_t mifareTransfer(const uint8_t block);
    /*!
      @brief Moves the contents of a block into the internal Transfer Buffer
      @param block Block address
      @return True if successful
      @warning Only value block is executable
      @pre Requires the sector to which the block belongs authentication
     */
    result_t mifareRestore(const uint8_t block);
    /*!
      @brief Read the value assuming the specified block is the value block
      @param[out] value value
      @param block Block address
      @return True if successful
      @pre Requires block authentication
    */
    result_t mifareReadValue(int32_t& value, const uint8_t block);
    /*!
      @brief Writes as a specified value block
      @param block Block address
      @param value value
      @return True if successful
      @pre Requires block authentication
     */
    result_t mifareWriteValue(const uint8_t block, const int32_t value);
    ///@}

protected:
    // register operation
    bool set_register_bit(const uint8_t reg, const uint8_t bit);
    bool clear_register_bit(const uint8_t reg, const uint8_t bit);
    bool read_register_with_align(const uint8_t reg, uint8_t* buf, const uint8_t len, const uint8_t align);
    bool write_pcd_command(const mfrc522::Command cmd);

    //
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
    result_t picc_authenticate(const m5::rfid::Command cmd, const UID& uid, const uint8_t block, const MifareKey& key);

    // MIFARE
    result_t mifare_transceive(const m5::rfid::Command cmd, const uint8_t block);
    result_t mifare_transceive(const uint8_t* buf, const uint8_t len, const bool usingtimeout = false);

    // dump
    result_t mifare_dump_classic(const UID& uid, const MifareKey& key);
    result_t mifare_dump_classic_sector(const UID& uid, const uint8_t sector);
    result_t mifare_dump_ultra_light();
    result_t mifare_dump_ultra_light_page(const uint8_t page);

    // crc
    inline bool calculate_crc(uint16_t& result, const uint8_t* buf, const uint8_t len)
    {
        return (this->*(_cfg.software_crc ? &UnitMFRC522::calculateSoftwareCRC : &UnitMFRC522::calculateCRC))(result,
                                                                                                              buf, len);
    }

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

constexpr uint8_t CONTROL_REG{0x0C};
constexpr uint8_t BIT_FRAMING_REG{0x0D};
constexpr uint8_t COLL_REG{0x0E};

// Command
constexpr uint8_t MODE_REG{0x11};
constexpr uint8_t TX_MODE_REG{0x12};
constexpr uint8_t RX_MODE_REG{0x13};

constexpr uint8_t TX_CONTROL_REG{0x14};
constexpr uint8_t TX_ASK_REG{0x15};

constexpr uint8_t MF_RX_REG{0x1D};

// Configuration
constexpr uint8_t CRC_RESULT_REGH{0x21};
constexpr uint8_t CRC_RESULT_REGL{0x22};

constexpr uint8_t MOD_WIDTH_REG{0x24};

constexpr uint8_t RFC_FG_REG{0x26};

constexpr uint8_t TMODE_REG{0x2A};
constexpr uint8_t TPRESCALER_REG_L{0x2B};
constexpr uint8_t TRELOAD_REG_H{0x2C};
constexpr uint8_t TRELOAD_REG_L{0x2D};

// Test register
constexpr uint8_t AUTO_TEST_REG{0x36};
constexpr uint8_t VERSION_REG{0x37};

//
constexpr uint8_t DEMOD_REG{0x4D};

}  // namespace command
}  // namespace mfrc522
///@endcond

}  // namespace unit
}  // namespace m5
#endif
