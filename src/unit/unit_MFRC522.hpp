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
// M5Unit-NFC
#include <nfc/nfc.hpp>
#include <nfc/a/nfca.hpp>

namespace m5 {
namespace unit {

namespace nfc {
struct AdapterMFRC522;  // For layer
}

namespace mfrc522 {

constexpr uint16_t MAX_FIFO_DEPTH{64};  //!< Maximum FIFO depth

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
  @brief The receiver’s signal voltage gain factor
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
    OCCUR_COLLISION,           //!< Collision occurs
    UID_NOT_COMPLETED,         //!< UID is not yet complete
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
        //! Initial NFC mode applied at begin()
        m5::nfc::NFC mode{m5::nfc::NFC::A};
        //! mode reg value. See also 9.3.2.2 ModeReg register
        uint8_t mode_reg{0x3D};
        //! Enable antenna on begin if true
        bool enable_antenna{true};
        //! The receiver’s signal voltage gain factor
        mfrc522::ReceiverGain receiver_gain{mfrc522::ReceiverGain::dB48};
        //! Using software CRC
        bool software_crc{false};
    };

    /*!
      @brief Constructor
      @param addr I2C address
     */
    explicit UnitMFRC522(const uint8_t addr = DEFAULT_ADDRESS) : Component(addr)
    {
        auto ccfg  = component_config();
        ccfg.clock = 400 * 1000U;
        component_config(ccfg);
    }
    /*!
      @brief Destructor
     */
    virtual ~UnitMFRC522()
    {
    }

    /*!
      @brief Begin the unit
      @return True if successful
     */
    virtual bool begin() override;
    /*!
      @brief Update internal state
      @param force Force update if true
     */
    virtual void update(const bool force = false) override;

    ///@name Settings for begin
    ///@{
    /*! @brief Gets the configuration */
    inline config_t config()
    {
        return _cfg;
    }
    //! @brief Set the configuration
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    /*!
      @brief Gets the current operating mode
      @note Only NFC-A
    */
    inline m5::nfc::NFC NFCMode() const
    {
        return m5::nfc::NFC::A;
    }
    /*!
      @brief Configure NFC mode
      @param mode NFC mode
      @return True if successful
      @warning On pure MFRC522 only NFC-A is supported. Derived classes (e.g. UnitWS1850S) override this.
     */
    inline virtual bool configureNFCMode(const m5::nfc::NFC mode)
    {
        // Nop
        return mode == m5::nfc::NFC::A;
    }

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

    ///@name GsN (N-driver conductance) / CWGsP / ModGsP (P-driver conductance)
    ///@{
    /*!
      @brief Read the GsN register (N-driver conductance: CWGsN[7:4] / ModGsN[3:0])
      @param[out] value Raw 8-bit value (high nibble = CWGsN, low nibble = ModGsN)
      @return True if successful
     */
    bool readGsN(uint8_t& value);
    /*!
      @brief Write the GsN register
      @param value Raw 8-bit value (high nibble = CWGsN, low nibble = ModGsN)
      @return True if successful
     */
    bool writeGsN(const uint8_t value);
    /*!
      @brief Read the CWGsP register (P-driver conductance during no modulation)
      @param[out] value 6-bit value (0x00-0x3F)
      @return True if successful
     */
    bool readCWGsP(uint8_t& value);
    /*!
      @brief Write the CWGsP register
      @param value 6-bit value (0x00-0x3F). Upper 2 bits are reserved.
      @return True if successful, false if value > 0x3F or I2C error
     */
    bool writeCWGsP(const uint8_t value);
    /*!
      @brief Read the ModGsP register (P-driver conductance during modulation)
      @param[out] value 6-bit value (0x00-0x3F)
      @return True if successful
     */
    bool readModGsP(uint8_t& value);
    /*!
      @brief Write the ModGsP register
      @param value 6-bit value (0x00-0x3F). Smaller = deeper ASK modulation.
      @return True if successful, false if value > 0x3F or I2C error
     */
    bool writeModGsP(const uint8_t value);
    ///@}

    ///@name RxThreshold (receiver decision levels)
    ///@{
    /*!
      @brief Read the RxThreshold register
      @param[out] value 8-bit raw value (MinLevel[7:4] / CollLevel[2:0])
      @return True if successful
     */
    bool readRxThreshold(uint8_t& value);
    /*!
      @brief Write the RxThreshold register
      @param value 8-bit raw value. bit[3] is reserved (must be 0).
      @return True if successful, false if reserved bit set or I2C error
      @note On WS1850S silicon this write is silently ignored (verified by exhaustive
        0x00-0xFF scan); the chip uses its internal fixed value (reset default 0x84).
        Provided here for genuine MFRC522 silicon compatibility.
     */
    bool writeRxThreshold(const uint8_t value);
    ///@}

    ///@name TPrescaler
    ///@{
    /*!
      @brief Read the TPrescaler
      @param[out] tprescale TPrescaler value
      @return True if successful
     */
    bool readTPrescaler(uint16_t& tprescale);
    /*!
      @brief Write the TPrescaler
      @param tprescale TPrescaler value
      @return True if successful
     */
    bool writeTPrescaler(const uint16_t tprescale);
    ///@}

    ///@name CRC
    ///@{
    /*!
      @brief Calculate CRC by hardware
      @param[out] result CRC value
      @param buf Buffer
      @param len The length of the buffer
      @return True if successful
     */
    bool calculateCRC(uint16_t& result, const uint8_t* buf, const uint8_t len);
    /*!
      @brief Calculate CRC by software
      @param[out] result CRC value
      @param buf Buffer
      @param len The length of the buffer
      @return True if successful
     */
    bool calculateSoftwareCRC(uint16_t& result, const uint8_t* buf, const uint8_t len);
    ///@}

    ///@name NFC-A
    ///@{
    /*!
      @brief Transceive
      @param rx Receive buffer
      @param[in,out] rx_len in:Size of receive buffer out:actual read size
      @param tx Send buffer
      @param tx_len Size of send buffer
      @param timeout_ms Timeout(ms)
      @return True if successful
     */
    bool nfcaTransceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                        const uint32_t timeout_ms);

    /*!
      @brief Request for idle PICC
      @param[out] atqa ATQA
      @return True if successful
     */
    inline bool request(uint16_t& atqa)
    {
        return request_wakeup(atqa, true);
    }
    /*!
      @brief Wakeup for idle/halt PICC
      @param[out] atqa ATQA
      @return True if successful
     */
    inline bool wakeup(uint16_t& atqa)
    {
        return request_wakeup(atqa, false);
    }

    /*!
      @brief Select PICC with anti-collision
      @param[out] completed Completed select?
      @param[out]  picc Selected PICC
      @param lv Cascade level (1-3)
      @return True if successful
     */
    bool selectWithAnticollision(bool& completed, m5::nfc::a::PICC& picc, const uint8_t lv);
    /*!
      @brief Select specific PICC
      @param  picc PICC
      @return True if successful
     */
    bool select(const m5::nfc::a::PICC& picc);

    /*!
      @brief Read the 1 block / 4 page (16 bytes)
      @param rx Receiver buffer (at least 16 bytes)
      @param block Block address
      @return True if successful
     */
    bool readBlock(uint8_t rx[16], const uint8_t block);
    /*!
      @brief Write the 1 block
      @param block Block address
      @param tx Send buffer (at least 16 bytes)
      @return True if successful
     */
    bool writeBlock(const uint8_t block, const uint8_t tx[16]);
    /*!
      @brief Hlt for PICC
      @return True if successful
     */
    bool hlt();
    ///@}

    ///@name MIFARE classic
    ///@{
    /*!
      @brief Authentication using keyA of the specified block
      @param picc PICC
      @param block Block address
      @param key MIFARE classic key
      @return True if successful
     */
    inline bool mifareClassicAuthenticateA(
        const m5::nfc::a::PICC& picc, const uint8_t block,
        const m5::nfc::a::mifare::classic::Key& key = m5::nfc::a::mifare::classic::DEFAULT_KEY)
    {
        return mifare_classic_authenticate(m5::nfc::a::Command::AUTH_WITH_KEY_A, picc, block, key);
    }
    /*!
      @brief Authentication using keyB of the specified block
      @param picc PICC
      @param block Block address
      @param key MIFARE classic key
      @return True if successful
     */
    inline bool mifareClassicAuthenticateB(
        const m5::nfc::a::PICC& picc, const uint8_t block,
        const m5::nfc::a::mifare::classic::Key& key = m5::nfc::a::mifare::classic::DEFAULT_KEY)
    {
        return mifare_classic_authenticate(m5::nfc::a::Command::AUTH_WITH_KEY_B, picc, block, key);
    }

    /*!
      @brief  Exit from authenticated state for MIFARE classic
      @return True if successful
     */
    bool mifareClassicStopCrypto1();

    /*!
      @brief Operation for the value block
      @param cmd Command
      @param block Block address
      @param arg Argument for command if needs
      @return True if successful
     */
    bool mifareClassicValueBlock(const m5::nfc::a::Command cmd, const uint8_t block, const uint32_t arg = 0);
    ///@}

protected:
    // register operation
    bool modify_bit_register8(const uint8_t reg, const uint8_t set_mask, const uint8_t clear_mask);
    bool set_bit_register8(const uint8_t reg, const uint8_t bits);
    bool clear_bit_register8(const uint8_t reg, const uint8_t bits);
    inline bool change_bit_register8(const uint8_t reg, const uint8_t bits, const uint8_t mask)
    {
        return modify_bit_register8(reg, mask & bits, mask);
    }
    bool read_register_with_align(const uint8_t reg, uint8_t* buf, const uint8_t len, const uint8_t align);
    bool write_pcd_command(const mfrc522::Command cmd);

    bool reset_baud_rates();
    bool flush_fifo_buffer();
    bool wait_comm_irq(const uint8_t irq, const uint32_t duration);
    bool wait_div_irq(const uint8_t irq, const uint32_t duration);

    // transceive
    bool transmit_command(const mfrc522::Command cmd, const uint8_t* buf, const uint8_t len, const uint8_t txLast = 0,
                          const uint8_t rxAlign = 0);
    bool transceive(uint8_t* rbuf, uint16_t& rlen, const uint8_t* buf, const uint16_t len, const uint32_t timeout_ms,
                    uint8_t& validBits, const uint8_t rxAlign = 0, const bool crc = false, uint8_t* err = nullptr);

    bool picc_haltA();

    // Read/Write
    bool read_block(uint8_t* rbuf, uint8_t& rlen, const uint8_t addr);
    bool write_block(const uint8_t block, const uint8_t* buf, const uint8_t len);
    bool write_page(const uint8_t page, const uint8_t* buf, const uint8_t len);

    // NFC-A
    bool request_wakeup(uint16_t& atqa, const bool request);
    bool anti_collision(const uint8_t cascadeLevel, uint8_t* buf);

    // MIFARE classic
    bool mifare_classic_authenticate(const m5::nfc::a::Command cmd, const m5::nfc::a::PICC& picc, const uint8_t block,
                                     const m5::nfc::a::mifare::classic::Key& key);
    bool mifare_classic_transceive(const m5::nfc::a::Command cmd, const uint8_t block, const uint32_t timeout_ms);
    bool mifare_classic_transceive(const uint8_t* buf, const uint8_t len, const uint32_t timeout_ms,
                                   const bool usingtimeout = false);

    // crc
    inline bool calculate_crc(uint16_t& result, const uint8_t* buf, const uint8_t len)
    {
        return (this->*(_cfg.software_crc ? &UnitMFRC522::calculateSoftwareCRC : &UnitMFRC522::calculateCRC))(result,
                                                                                                              buf, len);
    }

    virtual bool self_test();

protected:
    config_t _cfg{};
    uint16_t _tprescaler{};
};

///@cond
namespace mfrc522 {
namespace command {
// Command and status
constexpr uint8_t COMMAND_REG{0x01};
constexpr uint8_t COM_IEN_REG{0x02};
constexpr uint8_t DIV_IEN_REG{0x03};
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
constexpr uint8_t TX_SEL_REG{0x16};
constexpr uint8_t RX_SEL_REG{0x17};
constexpr uint8_t RX_THRESHOLD_REG{0x18};
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
