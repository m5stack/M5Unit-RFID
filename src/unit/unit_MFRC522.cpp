/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_MFRC522.cpp
  @brief MFRC522 Unit for M5UnitUnified
*/
#include "unit_MFRC522.hpp"
#include "rfid/nfc/nfc.hpp"
#include <M5Utility.hpp>
#include <cassert>
#include <thread>
#include <cstdio>
#include <cinttypes>

using namespace m5::unit::mfrc522;
using namespace m5::unit::mfrc522::command;
using namespace m5::rfid;
using namespace m5::rfid::mifare;
using namespace m5::rfid::mifare::classic;

namespace {
constexpr float F_CLOCK = 13.56f * 1000000;  // 13.56 Mhz

constexpr uint8_t NFC_MAGIC_NO{0xE1};
constexpr uint8_t NFC_VERSION{0x10};

// ErrorReg
constexpr uint8_t ERROR_BIT_WRITE{0x80};
constexpr uint8_t ERROR_BIT_TEMPERATURE{0x40};
constexpr uint8_t ERROR_BIT_RESERVED{0x20};
constexpr uint8_t ERROR_BIT_OVERFLOW{0x10};
constexpr uint8_t ERROR_BIT_COLLISION{0x08};
constexpr uint8_t ERROR_BIT_CRC{0x04};
constexpr uint8_t ERROR_BIT_PARITY{0x02};
constexpr uint8_t ERROR_BIT_PROTOCOL{0x01};

// ModeReg register bit [1:0] CRC preset
constexpr uint16_t crc_init_table[] = {0x0000,
                                       0xC6C6,  // Reverse of 0x6366
                                       0x8E65,  // Reverse of 0xA671
                                       0xFFFF};

// Firmware data for self-test
// Reference values based on firmware version
// ** Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August
// 2005; 16.1 Sefttest
constexpr std::array<uint8_t, 64> firmware_referenceV0_0{
    0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19, 0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
    0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50, 0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
    0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78, 0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
    0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF, 0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D};

// ** Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 Self test
constexpr std::array<uint8_t, 64> firmware_referenceV1_0{
    0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
    0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
    0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
    0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};

// ** Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 Self test
constexpr std::array<uint8_t, 64> firmware_referenceV2_0{
    0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
    0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
    0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
    0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};

// ** Clone
// Fudan Semiconductor FM17522 (0x88)
constexpr std::array<uint8_t, 64> firmware_referenceClone{
    0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18, 0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
    0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7, 0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
    0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7, 0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
    0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB, 0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62};

constexpr ReceiverGain receiver_gain_table[] = {
    ReceiverGain::dB18,
    ReceiverGain::dB23,
    ReceiverGain::dB18,  // duplicated
    ReceiverGain::dB23,  // duplicated
    //
    ReceiverGain::dB33,
    ReceiverGain::dB38,
    ReceiverGain::dB43,
    ReceiverGain::dB48,
};

constexpr uint8_t TX_CONTROL_TX12REF{0x03};  // Tx1RFEn | Tx2RFEn
constexpr uint8_t CASCADE_TAG{0x88};

constexpr uint8_t MIFARE_ACK{0x0A};
/*
0x00 transfer valid and invalid operation
0x01 transfer valid and parity or CRC error
0x00 transfer invalid and invalid operation
0x05 transfer invalid and parity or CRC error
:beg */

inline bool isTPrescaleEven(const uint8_t v)
{
    return v & (1U << 4);
}

inline float tprescaleToTimer(const uint16_t tpescale, const bool even)
{
    return F_CLOCK / (2 * tpescale + (int)even + 1);
}

inline uint16_t timerToTprescale(const float timer, const bool even)
{
    return std::round((F_CLOCK / timer - ((int)even + 1)) / 2);
}

inline float modulationWidth(const uint8_t tm)
{
    return tm + 1 / F_CLOCK;
}

inline uint8_t bits_to_NVB(const uint8_t bits)
{
    // High nibble:bytes Low nibble:fraction bits
    return (bits >> 3) | (bits & 0x07);
}

void dump_block(const uint8_t* buf, const int16_t block = -1, const int16_t sector = -1, const uint8_t ab = 0xFF,
                const bool aberror = false, const bool valueblock = false)
{
    char tmp[128 + 1] = "   ";
    uint32_t left{};
    // Sector
    if (sector >= 0) {
        left = snprintf(tmp, 4, "%02d)", sector);
    } else {
        left = 3;
    }
    // Block
    if (block >= 0) {
        left += snprintf(tmp + left, 7, "[%03d]:", block);
    } else {
        strcat(tmp, "      ");
        left += 6;
    }
    // Data
    for (uint8_t i = 0; i < 16; ++i) {
        left += snprintf(tmp + left, 4, "%02X ", buf[i]);
    }
    // Access bits
    if (ab != 0xFF) {
        if (!aberror) {
            left += snprintf(tmp + left, 8, "[%d %d %d]", (ab >> 2) & 1, (ab >> 1) & 1, (ab & 1));
        } else {
            strcat(tmp + left, "[ERROR]");
            left += 7;
        }
    }
    if (valueblock) {
        int32_t value{};
        uint8_t addr{};
        if (decode_value_block(value, addr, buf)) {
            snprintf(tmp + left, 26, " Addr:%03u Val:%" PRId32 "", addr, value);  // PRId32 for compile on NanoC6
        } else {
            strcat(tmp + left, "[Illgal value blcok]");
        }
    }
    ::puts(tmp);
}
}  // namespace

using namespace m5::utility::mmh3;

namespace m5 {
namespace unit {
// class UnitMFRC522
const char UnitMFRC522::name[] = "UnitMFRC522";
const types::uid_t UnitMFRC522::uid{"UnitMFRC522"_mmh3};
const types::uid_t UnitMFRC522::attr{0};
const UnitMFRC522::MifareKey UnitMFRC522::DEFAULT_CLASSIC_KEY{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool UnitMFRC522::readTPrescale(uint16_t& tprescale)
{
    uint8_t v[2]{};
    if (readRegister8(TMODE_REG, v[0], 0) && readRegister8(TPRESCALER_REG_L, v[1], 0)) {
        tprescale = ((v[0] & 0x0F) << 8) | v[1];
        return true;
    }
    return false;
}

bool UnitMFRC522::writeTPrescale(const uint16_t tprescale)
{
    uint8_t tm{};
    if (readRegister8(TMODE_REG, tm, 0)) {  // read TMODE_REG [7:4] and Tprescale high 4bits [3:0]
        tm = (tm & 0xF0) | ((tprescale >> 8) & 0x0F);
        return writeRegister8(TMODE_REG, tm) && writeRegister8(TPRESCALER_REG_L, tprescale & 0xFF);
    }
    return false;
}

bool UnitMFRC522::readTPrescale(float& hz)
{
    uint16_t tprescale{};
    uint8_t dr{};
    if (readRegister8(DEMOD_REG, dr, 0) && readTPrescale(tprescale)) {
        hz = tprescaleToTimer(tprescale, isTPrescaleEven(dr));
        // M5_LIB_LOGV(">>> tp:%04X => %f", tprescale, hz);
        return true;
    }
    return false;
}

bool UnitMFRC522::writeTPrescale(const float hz)
{
    uint8_t dr{};
    if (readRegister8(DEMOD_REG, dr, 0)) {  // even?
        uint16_t tprescale = timerToTprescale(hz, isTPrescaleEven(dr));
        // M5_LIB_LOGV(">>>> %f => tp:%04X", hz, tprescale);
        return writeTPrescale(tprescale);
    }
    return false;
}

bool UnitMFRC522::begin()
{
    if (!softReset()) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }

    if (!reset_baud_rates() ||
        // Timer starts automatically at the end of the transmission in all communication modes at all speeds
        // Timer about 40Khz
        // TMODE_REG TAuto [7] | Prescale high [3:0]
        // TPRESCALE_REG_LOW [7:0]
        !writeRegister8(TMODE_REG, 0x80) || !writeRegister8(TPRESCALER_REG_L, 0xA9) ||
        // reload timeer 1000 (0x03e8) (1/40 * 1000) ms
        !writeRegister8(TRELOAD_REG_H, 0x03) || !writeRegister8(TRELOAD_REG_L, 0xE8) ||
        //! writeRegister8(TRELOAD_REG_H, 0x00) || !writeRegister8(TRELOAD_REG_L, 40*5) || // 5ms
        // forces a 100% ASK modulation
        !writeRegister8(TX_ASK_REG, 0x40)) {
        M5_LIB_LOGE("Failed to configuration");
        return false;
    }

    // Mode and anttena
    return writeRegister8(MODE_REG, _cfg.mode_reg) && writeReceiverGain(_cfg.receiver_gain) &&
           (_cfg.enable_antenna ? turnOnAntenna() : true);
}

void UnitMFRC522::update(const bool /* force */)
{
    /* nop */
}

bool UnitMFRC522::softReset(const bool blocking)
{
    if (write_pcd_command(mfrc522::Command::SoftReset)) {
        if (!blocking) {
            return true;
        }

        m5::utility::delay(38);  // about 37.5 ms (datasheet 8.8.2)

        // Wait the power down flag has been cleared
        auto timeout_at = m5::utility::millis() + 250;
        do {
            uint8_t v{};
            if (readRegister8(COMMAND_REG, v, 0) && !(v & 0x10)) {
                return true;
            }
            m5::utility::delay(1);
        } while (m5::utility::millis() <= timeout_at);
    }
    return false;
}

bool UnitMFRC522::readAntennaStatus(bool& status)
{
    uint8_t v{};
    status = false;
    if (readRegister8(TX_CONTROL_REG, v, 0)) {
        status = ((v & TX_CONTROL_TX12REF) == TX_CONTROL_TX12REF);
        return true;
    }
    return false;
}

bool UnitMFRC522::turnOnAntenna()
{
    uint8_t v{};
    if (readRegister8(TX_CONTROL_REG, v, 0)) {
        if ((v & TX_CONTROL_TX12REF) != TX_CONTROL_TX12REF) {
            return writeRegister8(TX_CONTROL_REG, v | TX_CONTROL_TX12REF);
        }
        // Already on
        return true;
    }
    return false;
}

bool UnitMFRC522::turnOffAntenna()
{
    uint8_t v{};
    if (readRegister8(TX_CONTROL_REG, v, 0)) {
        if ((v & TX_CONTROL_TX12REF)) {
            return writeRegister8(TX_CONTROL_REG, (v & ~TX_CONTROL_TX12REF));
        }
        // Already off
        return true;
    }
    return false;
}

bool UnitMFRC522::readReceiverGain(ReceiverGain& gain)
{
    uint8_t v{};
    if (readRegister8(RFC_FG_REG, v, 0)) {
        gain = receiver_gain_table[(v >> 4) & 0x07];
        return true;
    }
    return false;
}

bool UnitMFRC522::writeReceiverGain(const ReceiverGain gain)
{
    uint8_t v{};
    if (readRegister8(RFC_FG_REG, v, 0)) {
        v = (v & ~(0x07 << 4)) | ((m5::stl::to_underlying(gain) & 0x07) << 4);
        return writeRegister8(RFC_FG_REG, v);
    }
    return false;
}

bool UnitMFRC522::calculateCRC(uint16_t& result, const uint8_t* buf, const uint8_t len)
{
    constexpr uint8_t CRC_IRQ{0x04};

    if (!write_pcd_command(mfrc522::Command::Idle) ||
        // Enable CRC IRQ
        !writeRegister8(DIV_IRQ_REG, CRC_IRQ) ||
        // Flush FIFO
        !flush_fifo_buffer() ||
        // Write data
        !writeRegister(FIFO_DATA_REG, buf, len) ||
        // Calculate CRC
        !write_pcd_command(mfrc522::Command::CalcCRC)) {
        return false;
    }

    // MSG/LSB first?
    uint8_t mode{};
    if (!readRegister8(MODE_REG, mode, 0)) {
        return false;
    }
    uint32_t msb_first = (mode & 0x80) ? 1 : 0;

    // Wait calculation to complete
    if (wait_div_irq(CRC_IRQ, 10)) {
        m5::types::big_uint16_t crc16{};
        if (!readRegister8(CRC_RESULT_REGH, crc16.u8[msb_first], 0) ||
            !readRegister8(CRC_RESULT_REGL, crc16.u8[msb_first ^ 1], 0)) {
            return false;
        }
        result = crc16.get();
        return write_pcd_command(mfrc522::Command::Idle);
    }

    return false;
}

bool UnitMFRC522::calculateSoftwareCRC(uint16_t& result, const uint8_t* buf, const uint8_t len)
{
    uint8_t mode{};
    if (!readRegister8(MODE_REG, mode, 0)) {
        return false;
    }
    bool ref{(mode & 0x80) == 0};

    m5::utility::CRC16 crc(crc_init_table[mode & 0x03], 0x1021, ref, ref, 0);
    result = crc.range(buf, len);
    return true;
}

UnitMFRC522::result_t UnitMFRC522::detectIdleDevice()
{
    ATQA tmp{};
    result_t result{};
    // Collision occurrence is also considered as detected.
    return (result = picc_requestA(tmp.value))          ? result      // OK
           : (result.error() == Error::OCCUR_COLLISION) ? result_t{}  // OK
                                                        : result;     // NG
}

UnitMFRC522::result_t UnitMFRC522::detectDevice()
{
    ATQA tmp{};
    result_t result{};
    // Collision occurrence is also considered as detected.
    return (result = picc_wakeupA(tmp.value))           ? result      // OK
           : (result.error() == Error::OCCUR_COLLISION) ? result_t{}  // OK
                                                        : result;     // NG
}

UnitMFRC522::result_t UnitMFRC522::activateDevice(UID& uid)
{
    result_t result = activate(uid);
    // UltraLight according to SAK judgment or activate error occurred
    if (uid.type != Type::MIFARE_UltraLight) {
        if (result) {
            M5_LIB_LOGV("Type:%u %u", uid.type, uid.blocks);
        }
        return result;
    }

    // Identification of UltraLight or others
    M5_LIB_LOGV("Detect Light or others %s", uid.uidAsString().c_str());

    UID copy = uid;

    // If UltraLightC, this command will be processed successfully
    uint8_t cmd[4]{m5::stl::to_underlying(m5::rfid::Command::AUTHENTICATE_1), 0x00};
    uint8_t txLast{0};
    uint16_t crc{};
    if (calculate_crc(crc, cmd, 2)) {
        cmd[2] = crc & 0xFF;
        cmd[3] = (crc >> 8) & 0xFF;
    }
    uint8_t rbuf[8]{};
    uint8_t rlen{8};
    result = picc_transceive(rbuf, rlen, cmd, m5::stl::size(cmd), txLast, 0, true);
    if (result && rbuf[0] == 0xAF) {
        uid.type   = Type::MIFARE_UltraLightC;
        uid.blocks = get_number_of_blocks(Type::MIFARE_UltraLightC);
        M5_LIB_LOGV("Type:%u %u", uid.type, uid.blocks);
        return result;
    }

    // Reactivate same UID
    uint32_t retry{8};
    do {
        result = reactivate(uid, copy);
    } while (!result && retry--);
    if (!result) {
        return result;
    }

    // Check NTAG
    // If NTAG21x, this command will be processed successfully
    uint8_t vbuf[10]{};
    uint8_t vlen{10};
    bool version = (bool)ntag_get_version(vbuf, vlen);
    if (!version) {
        // Reactivate same UID
        retry = 8;
        do {
            result = reactivate(uid, copy);
        } while (!result && retry--);
        if (!result) {
            return result;
        }
    }

    uint8_t buf[18]{};
    uint8_t blen{18};
    result = read_block(buf, blen, 0);
    if (!result) {
        return result;
    }
    // M5_DUMPI(buf + 12, 4);
    if (buf[12] == NFC_MAGIC_NO && buf[13] == NFC_VERSION) {  // NTAG?
        uid.type = (buf[14] == 0x12)   ? Type::NTAG_213
                   : (buf[14] == 0x3E) ? Type::NTAG_215
                   : (buf[14] == 0x6D) ? Type::NTAG_216
                   : (buf[14] == 0x10) ? Type::NTAG_212
                   : (buf[14] == 0x06)
                       ? (version ? ((vbuf[4] == 0x02) ? Type::NTAG_210u : Type::NTAG_210) : Type::MIFARE_UltraLight)
                       : Type::MIFARE_UltraLight;
    }

    uid.blocks = get_number_of_blocks(uid.type);
    M5_LIB_LOGV("Type:%u %u [%02X]", uid.type, uid.blocks, buf[14]);
    return result;
}

UnitMFRC522::result_t UnitMFRC522::activate(UID& uid)
{
    uint8_t lv{1};
    result_t result;
    do {
        result = picc_select(uid, lv++);
    } while (!result && result.error() == Error::UID_NOT_COLMPLETED && lv < 4);
    M5_LIB_LOGV("Sel:%02X", result ? 0 : m5::stl::to_underlying(result.error()));
    return result;
}

UnitMFRC522::result_t UnitMFRC522::reactivate(UID& uid, const UID& prev)
{
    ATQA tmp{};
    auto result = picc_wakeupA(tmp.value);
    if (result) {
        result = activate(uid);
        // Is a different device selected?
        if (std::memcmp(uid.uid, prev.uid, m5::stl::size(uid.uid))) {
            M5_LIB_LOGE("UID is different");
            picc_haltA();
            result = m5::stl::make_unexpected(Error::INTERNAL);
        }
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::deactivateDevice()
{
    auto result = picc_haltA();
    return mifareStopCrypto1() ? result : m5::stl::make_unexpected(Error::COMMUNICATION);
}

UnitMFRC522::result_t UnitMFRC522::dumpDevice(const UID& uid, const MifareKey& key)
{
    result_t result = m5::stl::make_unexpected(Error::ARGUMENT);
    if (uid.isClassic()) {
        result = dump_sector_structure(uid, key);
    } else if (uid.canNFC()) {
        result = dump_page_structure(uid.blocks);
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::dumpDevice(const UID& uid, const uint8_t block)
{
    result_t result = m5::stl::make_unexpected(Error::ARGUMENT);
    if (uid.isClassic()) {
        result = dump_sector(get_sector(block));
    } else if (uid.canNFC()) {
        result = dump_page(block);
    }
    return result;
}

////
// IDLE to READY
UnitMFRC522::result_t UnitMFRC522::picc_requestA(uint16_t& atqa)
{
    return picc_to_ready(m5::rfid::Command::REQA, atqa);
}

// IDLE or HALT to READY
UnitMFRC522::result_t UnitMFRC522::picc_wakeupA(uint16_t& atqa)
{
    return picc_to_ready(m5::rfid::Command::WUPA, atqa);
}

UnitMFRC522::result_t UnitMFRC522::picc_to_ready(const m5::rfid::Command piccCommand, uint16_t& atqa)
{
    if (piccCommand != m5::rfid::Command::REQA && piccCommand != m5::rfid::Command::WUPA) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t rlen = 2;  // ATQA 16bit
    uint8_t cmd  = m5::stl::to_underlying(piccCommand);
    uint8_t valid_bits{0x07};

    //    if (reset_baud_rates() && clear_register_bit(COLL_REG, 0x80)) {
    if (clear_register_bit(COLL_REG, 0x80)) {
        auto result = picc_transceive((uint8_t*)&atqa, rlen, &cmd, 1, valid_bits);
        if (result && (rlen != 2 || valid_bits)) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        return result;
    }
    return m5::stl::make_unexpected(Error::COMMUNICATION);
}

//
bool UnitMFRC522::set_register_bit(const uint8_t reg, const uint8_t bit)
{
    uint8_t v{};
    return readRegister8(reg, v, 0) && writeRegister8(reg, v | bit);
}

bool UnitMFRC522::clear_register_bit(const uint8_t reg, const uint8_t bit)
{
    uint8_t v{};
    return readRegister8(reg, v, 0) && writeRegister8(reg, v & ~bit);
}

// Read values from register with alignment
// Update buf[0] with first data if align is not zero
bool UnitMFRC522::read_register_with_align(const uint8_t reg, uint8_t* rwbuf, const uint8_t len, const uint8_t align)
{
    uint8_t lead = rwbuf[0];
    if (readRegister(reg, rwbuf, len, 0)) {
        if (align) {
            uint8_t mask = 0xFF << align;
            rwbuf[0]     = (lead & ~mask) | (rwbuf[0] & mask);
        }
        return true;
    }
    return false;
}

bool UnitMFRC522::write_pcd_command(const mfrc522::Command cmd)
{
    return writeRegister8(COM_IRQ_REG, 0x7F) &&
           writeRegister8(mfrc522::command::COMMAND_REG, m5::stl::to_underlying(cmd) & 0x0F);
    //    return writeRegister8(mfrc522::command::COMMAND_REG, m5::stl::to_underlying(cmd) & 0x0F);
}

bool UnitMFRC522::reset_baud_rates()
{
    return writeRegister8(TX_MODE_REG, 0x00) &&  // TxSpped 106kBd
           writeRegister8(RX_MODE_REG, 0x00) &&  // RxSpeed 106kBd
           writeRegister8(MOD_WIDTH_REG, 0x26);  //
}

bool UnitMFRC522::flush_fifo_buffer()
{
    return writeRegister8(FIFO_LEVEL_REG, 0x80);
}

bool UnitMFRC522::wait_comm_irq(const uint8_t irq, const uint32_t duration)
{
    uint8_t v{};
    auto timeout_at = m5::utility::millis() + duration;
    do {
        if (readRegister8(COM_IRQ_REG, v, 0)) {
            if (v & irq) {
                return true;
            }
            if (v & 0x01) {  // occurs timeout
                return false;
            }
        }
        std::this_thread::yield();
    } while (m5::utility::millis() <= timeout_at);
    return false;
}

bool UnitMFRC522::wait_div_irq(const uint8_t irq, const uint32_t duration)
{
    uint8_t v{};
    auto timeout_at = m5::utility::millis() + duration;
    do {
        readRegister8(DIV_IRQ_REG, v, 0);
        if (v & irq) {
            return true;
        }
        std::this_thread::yield();
    } while (m5::utility::millis() <= timeout_at);
    return false;
}

UnitMFRC522::result_t UnitMFRC522::picc_anti_collision(const uint8_t cascadeLevel, uint8_t* buf)
{
    uint8_t slen{2}, rlen{5};
    uint8_t* rbuf = buf + 2;
    uint8_t txLast{0}, fixedBits{}, cpos{};

    // First time (0x9n, 0x20)
    if (!clear_register_bit(COLL_REG, 0x80)) {
        return m5::stl::make_unexpected(Error::COMMUNICATION);
    }
    buf[0] = m5::stl::to_underlying(m5::rfid::Command::SELECT_CL1) + (cascadeLevel - 1) * 2;  // 0x93,95,97
    buf[1] = 0x20;                                                                            // first NVB

    auto result = picc_transceive(rbuf, rlen, buf, slen, txLast);
    if (!result) {
        M5_LIB_LOGV("1st:%02x", result.error());
    }

    // Loop for anti coollision
    // Collision avoidance attempts up to 32
    uint32_t count{};
    while (!result && result.error() == Error::OCCUR_COLLISION && count++ < 32) {
        uint8_t col{};
        if (!readRegister8(COLL_REG, col, 0)) {
            return m5::stl::make_unexpected(Error::COMMUNICATION);
        }
        if (col & 0x20) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        // uint8_t cpos = __builtin_ffs(rbuf[0]);           // First collision position
        cpos = (col & 0x1F) ? (col & 0x1F) : 32;
        if (cpos <= fixedBits) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        fixedBits     = cpos;
        uint8_t count = (fixedBits - 1) & 0x07;

        uint8_t index = 1 + (fixedBits / 8) + (count ? 1 : 0);  // First byte is index 0.
        buf[index] |= (1 << count);

        txLast = fixedBits & 0x07;
        count  = fixedBits / 8;
        index  = 2 + count;

        buf[1] = (index << 4) + txLast;  // NVB - Number of Valid Bits
        slen   = index + (txLast ? 1 : 0);

        // M5_DUMPI(buf, slen);

        if (!clear_register_bit(COLL_REG, 0x80)) {
            return m5::stl::make_unexpected(Error::COMMUNICATION);
        }
        result = picc_transceive(rbuf, rlen, buf, slen, txLast, txLast);
        if (!result) {
            M5_LIB_LOGV("  (%u):%02x", count, result.error());
        }
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::picc_select(UID& uid, const uint8_t cascadeLevel)
{
    uint8_t buf[9]{};
    uint8_t* rbuf{};
    uint8_t slen{}, rlen{};
    uint8_t align{}, last_bits{};
    uint16_t crc{};

    if (cascadeLevel < 1 || cascadeLevel > 3) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    if (!clear_register_bit(COLL_REG, 0x80)) {
        return m5::stl::make_unexpected(Error::COMMUNICATION);
    }

    // Anti collision
    auto result = picc_anti_collision(cascadeLevel, buf);
    if (!result) {
        M5_LIB_LOGV("AC:%02X", result.error());
        return result;
    }

    // Select
    buf[1] = 0x70;
    if (!calculate_crc(crc, buf, 7)) {
        return m5::stl::make_unexpected(Error::CRC);
    }
    buf[7]    = crc & 0xFF;
    buf[8]    = (crc >> 8) & 0xFF;
    slen      = 9;
    rbuf      = buf + 6;
    rlen      = 3;
    last_bits = 0;

    result = picc_transceive(rbuf, rlen, buf, slen, last_bits, align);
    if (!result) {
        M5_LIB_LOGV("SEL:%02X", result.error());
        return result;
    }

    // Check SAK CRC
    if (!calculate_crc(crc, rbuf, 1) || crc != *(uint16_t*)(rbuf + 1)) {
        return m5::stl::make_unexpected(Error::CRC);
    }

    // Copy valid uid block
    std::memcpy(uid.uid + (cascadeLevel - 1) * 3, &buf[2 + (buf[2] == CASCADE_TAG)], 4 - (buf[2] == CASCADE_TAG));

    // Completed?
    if ((rbuf[0] & 0x04) == 0) {
        uid.size   = 1 + cascadeLevel * 3;
        uid.sak    = rbuf[0];
        uid.type   = get_type(rbuf[0]);
        uid.blocks = get_number_of_blocks(uid.type);
        return {};
    }
    return m5::stl::make_unexpected(Error::UID_NOT_COLMPLETED);
}

UnitMFRC522::result_t UnitMFRC522::picc_send(const mfrc522::Command cmd, const uint8_t* buf, const uint8_t len,
                                             const uint8_t txLast, const uint8_t rxAlign)
{
    if (!buf || len == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    // bitframeing value RxAlign [6:4] TxLastBits [2:0]
    uint8_t bfvalue = ((rxAlign & 0x07) << 4) | (txLast & 0x07);

    // Execute transceive command
    if (!write_pcd_command(mfrc522::Command::Idle) ||  // Force stop of running commands
        !writeRegister8(COM_IRQ_REG, 0x7F) ||          // Clear interrupt request bits
        !flush_fifo_buffer() ||                        // Flush FIFO
        !writeRegister8(BIT_FRAMING_REG, bfvalue) ||   // Adjustments for bit-oriented frames
        !writeRegister(FIFO_DATA_REG, buf, len) ||     // Write data to FIFO with adjustment (Not started)
        !write_pcd_command(cmd) ||                     // Execute command
        !set_register_bit(BIT_FRAMING_REG, 0x80)       // StartSend:7 Start the transmission
    ) {
        M5_LIB_LOGE("Failed to send");
        return m5::stl::make_unexpected(Error::COMMUNICATION);
    }
    return {};
}

UnitMFRC522::result_t UnitMFRC522::picc_transceive(uint8_t* rbuf, uint8_t& rlen, const uint8_t* buf, const uint8_t len,
                                                   uint8_t& validBits, const uint8_t rxAlign, const bool crc)
{
    if (!rbuf || !rlen || !buf || !len) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    M5_LIB_LOGV(">>txLast:%u rxAlign:%u", validBits, rxAlign);
    auto result = picc_send(mfrc522::Command::Transceive, buf, len, validBits, rxAlign);
    if (!result) {
        return result;
    }

    // Wait for command completion (timeout 36ms)
    if (!wait_comm_irq(0x30 /* RxIRq | IdleIRq */, 36)) {
        M5_LIB_LOGV("Timeout");
        return m5::stl::make_unexpected(Error::TIMEOUT);
    }

    uint8_t err{};
    if (!readRegister8(ERROR_REG, err, 0)) {
        return m5::stl::make_unexpected(Error::COMMUNICATION);
    }

    // Read FIFO
    uint8_t fifo_len{};
    uint8_t valid{};
    if (!readRegister8(FIFO_LEVEL_REG, fifo_len, 0)) {
        return m5::stl::make_unexpected(Error::COMMUNICATION);
    }
    M5_LIB_LOGV("- Recv:%u", fifo_len);
    if (fifo_len > rlen) {
        M5_LIB_LOGE("Not enough rlen %zu : %u", rlen, fifo_len);
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    if (!read_register_with_align(FIFO_DATA_REG, rbuf, fifo_len, rxAlign) ||
        // indicates the number of valid bits in the last received byte if this value is 000b, the whole byte is
        // valid
        !readRegister8(CONTROL_REG, valid, 0)) {
        return m5::stl::make_unexpected(Error::COMMUNICATION);
    }
    valid &= 0x07;
    validBits = valid;  // It's indicates the number of valid bits in the last received

    // Check collision (Priority over other errors)
    if (err & ERROR_BIT_COLLISION) {
        M5_LIB_LOGV("=COLLISION=");
        return m5::stl::make_unexpected(Error::OCCUR_COLLISION);
    }
    // Check error
    if (err & (ERROR_BIT_OVERFLOW | ERROR_BIT_PARITY | ERROR_BIT_PROTOCOL)) {
        M5_LIB_LOGE("Error occurred: %02X", err);
        return m5::stl::make_unexpected(Error::REGISTER);
    }

    // CRC
    if (crc) {
        if (fifo_len == 1 && valid == 4) {
            M5_LIB_LOGE("NG MIFARE Classic NAK %02X", rbuf[0]);
            return m5::stl::make_unexpected(Error::MIFARE_NACK);
        }
        if (fifo_len < 2 || valid) {
            M5_LIB_LOGE("Not in a condition to calculate CRC %u/%u", fifo_len, valid);
            return m5::stl::make_unexpected(Error::CRC);
        }

        uint16_t crc16{};
        if (!calculate_crc(crc16, rbuf, fifo_len - 2) || ((crc16 >> 8) & 0XFF) == rbuf[fifo_len - 2] ||
            (crc16 & 0xFF) == rbuf[fifo_len - 1]) {
            return m5::stl::make_unexpected(Error::CRC);
        }
    }

    rlen = fifo_len;
    M5_LIB_LOGV("<<rxLast:%u", validBits);
    return {};
}

UnitMFRC522::result_t UnitMFRC522::picc_haltA()
{
    uint8_t buf[4]{}, txLast{};
    uint16_t crc{};
    buf[0] = m5::stl::to_underlying(m5::rfid::Command::HLTA);

    if (!calculate_crc(crc, buf, 2)) {
        return m5::stl::make_unexpected(Error::CRC);
    }
    buf[2] = crc & 0xFF;
    buf[3] = (crc >> 8) & 0xFF;

    auto result = picc_send(mfrc522::Command::Transceive, buf, 4, txLast);  // No recv data
    if (result) {
        if (!wait_comm_irq(0x01 /* TimerIRq */, 36)) {
            M5_LIB_LOGV("Timeout");
            return m5::stl::make_unexpected(Error::TIMEOUT);
        }
    }
    return result;
}

// Read/Write

UnitMFRC522::result_t UnitMFRC522::readDevice(const UID& uid, uint8_t* rbuf, uint8_t& rlen, const uint8_t addr)
{
    uint8_t raddr = addr;
    if (uid.canNFC()) {  // page structure
        raddr &= ~0x03;
    }
    return read_block(rbuf, rlen, raddr);
}

UnitMFRC522::result_t UnitMFRC522::read_block(uint8_t* rbuf, uint8_t& rlen, const uint8_t addr)
{
    if (!rbuf || rlen < 18) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t cmd[4]{m5::stl::to_underlying(m5::rfid::Command::READ), addr};
    uint16_t crc{};
    uint8_t txLast{0};
    if (!calculate_crc(crc, cmd, 2)) {
        return m5::stl::make_unexpected(Error::CRC);
    }
    cmd[2] = crc & 0xFF;
    cmd[3] = (crc >> 8) & 0xFF;

    return picc_transceive(rbuf, rlen, cmd, m5::stl::size(cmd), txLast, 0, true /* return with CRC*/);
}

UnitMFRC522::result_t UnitMFRC522::writeDevice(const UID& uid, const uint8_t addr, const uint8_t* buf,
                                               const uint8_t len, const bool safety)
{
    if (uid.canNFC()) {
        return writeDevicePage(uid, addr, buf, len, safety);
    }
    return writeDeviceBlock(uid, addr, buf, len, safety);
}

UnitMFRC522::result_t UnitMFRC522::writeDeviceBlock(const UID& uid, const uint8_t block, const uint8_t* buf,
                                                    const uint8_t len, const bool safety)
{
    if (safety && (is_sector_trailer_block(block) || block < get_first_user_block(uid.type) ||
                   block > get_last_user_block(uid.type))) {
        M5_LIB_LOGW("Write has been rejected due to safety %u", block);
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    if (block >= uid.blocks) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    return write_block(block, buf, len);
}

UnitMFRC522::result_t UnitMFRC522::write_block(const uint8_t block, const uint8_t* buf, const uint8_t len)
{
    if (!buf || !len || len > 16) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    // Write in units of 16 bytes
    std::array<uint8_t, 16> wbuf{};
    std::memcpy(wbuf.data(), buf, len);

    uint8_t cmd[2]{m5::stl::to_underlying(m5::rfid::Command::WRITE), block};
    result_t result{};
    return (result = mifare_transceive(cmd, 2)) ? mifare_transceive(wbuf.data(), wbuf.size()) : result;
}

UnitMFRC522::result_t UnitMFRC522::writeDevicePage(const UID& uid, const uint8_t page, const uint8_t* buf,
                                                   const uint32_t len, const bool safety)
{
    if (!buf || !len) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    if (safety && (page < get_first_user_block(uid.type) || page > get_last_user_block(uid.type))) {
        M5_LIB_LOGW("Write has been rejected due to safety %u", page);
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    if (len <= 4) {
        return write_page(page, buf, len);
    }

    int16_t pages = (len + 3) / 4;
    if (page + pages > uid.blocks) {
        M5_LIB_LOGE("Not enough page");
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    result_t result{};
    uint8_t current = page;
    uint32_t remain = len;
    while (pages--) {
        auto clen = (remain > 4) ? 4 : remain;
        result    = write_page(current, buf, clen);
        if (!result) {
            break;
        }
        ++current;
        buf += clen;
        remain -= clen;
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::write_page(const uint8_t page, const uint8_t* buf, const uint8_t len)
{
    if (!buf || len > 4) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    uint8_t cmd[6]{m5::stl::to_underlying(m5::rfid::Command::WRITE_UL), page};
    std::memcpy(cmd + 2, buf, len);
    return mifare_transceive(cmd, m5::stl::size(cmd));
}

// MIFARE
bool UnitMFRC522::mifareStopCrypto1()
{
    return clear_register_bit(STATUS2_REG, 0x08);
}

UnitMFRC522::result_t UnitMFRC522::mifareEnableValueBlock(const UID& uid, const uint8_t block, const MifareKey& keyA,
                                                          const MifareKey& keyB, const bool readOnly)

{
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t permission = readOnly ? 0x01 : 0x06;
    uint8_t buf[18]{};
    uint8_t len{18};
    uint8_t permissions[4]{};
    uint8_t st_block = get_sector_trailer_block(block);
    uint8_t poff     = get_permission_offset(block);

    // Read sector trailer block
    auto result = read_block(buf, len, st_block);
    // M5_LIB_LOGW("R) %02X", result ? 0 : m5::stl::to_underlying(result.error()));
    if (result) {
        if (!decode_access_bits(permissions, buf + 6)) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        // Already value block?
        if (permissions[poff] == permission) {
            return {};  // OK
        }
        // Update sector trailer
        permissions[poff] = permission;
        permissions[3]    = 0x03;  // 011: never/keyB /keyA|B/keyB never/keyB
        if (!encode_access_bits(buf + 6, permissions)) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        std::memcpy(buf, keyA.data(), 6);
        std::memcpy(buf + 10, keyB.data(), 6);
        result = write_block(st_block, buf, 16);
        // M5_LIB_LOGW("W) %02X", result ? 0 : m5::stl::to_underlying(result.error()));
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::mifareDisableValueBlock(const UID& uid, const uint8_t block, const MifareKey& keyA,
                                                           const MifareKey& keyB, const uint8_t permission)
{
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0 || is_value_block_permission(permission) ||
        (permission & 0xF8)) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t buf[18]{};
    uint8_t len{18};
    uint8_t permissions[4]{};
    uint8_t st_block = get_sector_trailer_block(block);
    uint8_t poff     = get_permission_offset(block);

    // Read sector trailer block
    auto result = read_block(buf, len, st_block);
    // M5_LIB_LOGE("R) %02X", result ? 0 : m5::stl::to_underlying(result.error()));
    if (result) {
        if (!decode_access_bits(permissions, buf + 6)) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        // Update sector trailer
        permissions[poff] = permission;
#if 0
        if (!is_value_block_permission(permissions[0]) && !is_value_block_permission(permissions[1]) &&
            !is_value_block_permission(permissions[2])) {
            // Are all blocks not value block?
            permissions[3] = 0x01; // 001: never/keyA keyA/keyA keyA/keyA
        }
#endif
        if (!encode_access_bits(buf + 6, permissions)) {
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
        std::memcpy(buf, keyA.data(), 6);
        std::memcpy(buf + 10, keyB.data(), 6);
        result = write_block(st_block, buf, 16);  // update sector tailer
        // M5_LIB_LOGE("W) %02X", result ? 0 : m5::stl::to_underlying(result.error()));
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::mifareIncrement(const UID& uid, const uint8_t block, const uint32_t delta)
{
    result_t result{};
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    return (result = mifare_transceive(m5::rfid::Command::INCREMENT, block))
               ? mifare_transceive((const uint8_t*)&delta, 4, true)
               : result;
}

UnitMFRC522::result_t UnitMFRC522::mifareDecrement(const UID& uid, const uint8_t block, const uint32_t delta)
{
    result_t result{};
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    return (result = mifare_transceive(m5::rfid::Command::DECREMENT, block))
               ? mifare_transceive((const uint8_t*)&delta, 4, true)
               : result;
}

UnitMFRC522::result_t UnitMFRC522::mifareRestore(const UID& uid, const uint8_t block)
{
    if (uid.isClassic()) {
        result_t result{};
        uint8_t dummy[4]{};
        return (result = mifare_transceive(m5::rfid::Command::RESTORE, block)) ? mifare_transceive(dummy, 4, true)
                                                                               : result;
    }
    return m5::stl::make_unexpected(Error::ARGUMENT);
}

UnitMFRC522::result_t UnitMFRC522::mifareTransfer(const UID& uid, const uint8_t block)
{
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }
    return mifare_transceive(m5::rfid::Command::TRANSFER, block);
}

UnitMFRC522::result_t UnitMFRC522::mifareReadValue(const UID& uid, int32_t& value, const uint8_t block)
{
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t buf[18]{};
    uint8_t len{18};

    value       = 0;
    auto result = read_block(buf, len, block);
    if (result) {
        uint8_t addr{};
        if (!decode_value_block(value, addr, buf)) {
            M5_LIB_LOGW("Block %u is NOT value block format data", block);
            return m5::stl::make_unexpected(Error::INTERNAL);
        }
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::mifareWriteValue(const UID& uid, const uint8_t block, const int32_t value)
{
    if (!uid.isClassic() || is_sector_trailer_block(block) || block == 0) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t buf[16]{};
    encode_value_block(buf, value, block);
    return write_block(block, buf, 16);
}

// NFC
UnitMFRC522::result_t UnitMFRC522::nfcWriteChangeToNTAGFormat(const UID& uid)
{
    if (uid.type == Type::MIFARE_UltraLight || uid.type == Type::MIFARE_UltraLightC) {
        if (ntag_check_format(uid)) {
            return {};  // Already NFC
        }
        uint8_t buf[4] = {NFC_MAGIC_NO, NFC_VERSION};
        buf[3]         = (uid.type == Type::MIFARE_UltraLight) ? 0x06 : 0x18;
        return write_page(3 /*OTP area*/, buf, 4);
    }
    if (uid.canNFC()) {
        return {};
    }
    M5_LIB_LOGE("Can not change to NFC format. %u", uid.type);
    return m5::stl::make_unexpected(Error::ARGUMENT);
}

/*
  targetBit Bit group  of the tag included in the size calculation
  0x01:Null, 0x02:LockControl 0x04:NDEFMessage, ....
  e.g. 0x06 means LockControl and MemoryControl
  Proprietary and Terminator can not  be excluded
 */
UnitMFRC522::result_t UnitMFRC522::ntag_calclate_ndef_message_size(const UID& uid, uint32_t& sz,
                                                                   const uint8_t targetTagBit)
{
    using m5::rfid::nfc::ndef::is_terminator_tag;
    using m5::rfid::nfc::ndef::is_valid_tag;
    using m5::rfid::nfc::ndef::Tag;

    uint8_t page     = get_first_user_block(uid.type);
    uint8_t max_page = get_last_user_block(uid.type) / 4 * 4;
    uint32_t required{}, idx{};
    uint16_t payload_len{};
    bool done{};

    sz = 0;

    while (page <= max_page) {
        uint8_t rbuf[18]{};
        uint8_t rlen{18};
        // M5_LIB_LOGE("R:%u/%u", page, max_page);
        auto result = read_block(rbuf, rlen, page);
        if (!result) {
            return result;
        }

        idx &= 0x0F;
        while (!done && idx < 16) {
            auto t = rbuf[idx++];
            // M5_LIB_LOGE(" Tag[%02X]", t);
            //  Not tag, or other than targets
            if (!is_valid_tag(t) || (t < 0x04 && !(targetTagBit & (1U << t)))) {
                done = true;
                break;
            }
            ++required;

            // Terminator?
            if (is_terminator_tag(t)) {
                done = true;
                break;
            }

            // Any message
            payload_len = rbuf[idx++];
            ++required;
            if (payload_len == 0xFF) {  // 3 bytes format
                payload_len = ((uint16_t)rbuf[idx++]) << 8;
                payload_len |= ((uint16_t)rbuf[idx++]);
                required += 2;
            }
            required += payload_len;
            // M5_LIB_LOGW("  PL:%u", payload_len);

            if (payload_len >= (16 - idx)) {
                idx += payload_len;
                page += idx / 16 * 4 - 4;  // skip to next message, (*1) add 4 after break
                break;
            }
            idx += payload_len;
        }
        if (done) {
            break;
        }
        page += 4;  // Try read next 16 bytes [*1]
    }

    sz = required;
    // M5_LIB_LOGW("NDEFSZ:%u", sz);
    return {};
}

UnitMFRC522::result_t UnitMFRC522::nfcReadDevice(const UID& uid, uint8_t* buf, uint32_t& len)
{
    result_t result{};
    uint8_t page   = get_first_user_block(uid.type);
    uint8_t page16 = (len - 2) / 16;

    if (!buf || len < page16 * 16 + 2 || !uid.canNFC()) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    len = 0;
    while (page16--) {
        uint8_t rlen{18};
        result = read_block(buf, rlen, page);
        if (!result) {
            break;
        }
        buf += 16;  // Overwrite prev CRC space
        page += 4;
        len += 16;
    }
    if (result) {
        len += 2;  // last CRC
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::nfcReadRequiredSize(const UID& uid, uint32_t& len)
{
    auto result = ntag_calclate_ndef_message_size(uid, len, 0x0F /* all tag */);
    len         = result ? (len + 15) / 16 * 16 + 2 /*CRC*/ : 0;
    return result;
}

UnitMFRC522::result_t UnitMFRC522::nfcWriteDevice(const UID& uid, const uint8_t* buf, const uint32_t blen)
{
    if (!uid.canNFC()) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t page{}, offset{};
    page = get_first_user_block(uid.type);

    // UltraLight/C
    if (!uid.isNTAG()) {
        return writeDevicePage(uid, page, buf, blen);
    }

    // NTAG 2xx
    // There may be information at the begin of the user area that should not be deleted
    // e.g. NTAG212, 213 LockCOntrol
    uint32_t sz{};
    auto result = ntag_calclate_ndef_message_size(uid, sz);
    if (!result) {
        return result;
    }
    page += sz / 4;
    offset = sz & 0x03;

    // M5_LIB_LOGW("Writable:%u:%u", page, offset);

    if (offset == 0) {
        return writeDevicePage(uid, page, buf, blen);
    }

    // Write new messages in a concatenated manner while maintaining existing messages
    uint8_t rbuf[18]{};
    uint8_t rlen{18};
    result = read_block(rbuf, rlen, page & ~0x03);
    if (!result) {
        M5_LIB_LOGE("Failed to read");
        return result;
    }

    uint8_t* ptr = (uint8_t*)malloc(16 + blen);
    if (!ptr) {
        return m5::stl::make_unexpected(Error::INTERNAL);
    }
    std::memcpy(ptr, &rbuf[(page & 0x03) * 4], offset);
    std::memcpy(ptr + offset, buf, blen);
    uint32_t nlen = offset + blen;

    // M5_DUMPI(ptr, nlen);

    result = writeDevicePage(uid, page, ptr, nlen);
    free(ptr);

    return result;
}

//
UnitMFRC522::result_t UnitMFRC522::mifare_authenticate(const m5::rfid::Command cmd, const UID& uid, const uint8_t block,
                                                       const MifareKey& key)
{
    if (cmd != m5::rfid::Command::AUTH_WITH_KEY_A && cmd != m5::rfid::Command::AUTH_WITH_KEY_B) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    // MFRC522 10.3.1.9 MFAuthent
    uint8_t buf[12]{m5::stl::to_underlying(cmd), block};
    // uint8_t buf[12]{m5::stl::to_underlying(cmd), get_sector_trailer_block(block)};
    std::memcpy(buf + 2, key.data(), key.size());
    std::memcpy(buf + 8, uid.uid, 4);                                               // UID first 4bytes
    auto result = picc_send(mfrc522::Command::MFAuthent, buf, m5::stl::size(buf));  // No recv data
    if (result) {
        if (!wait_comm_irq(0x10 /* IdleIRq */, 36)) {
            M5_LIB_LOGV("Timeout");
            return m5::stl::make_unexpected(Error::TIMEOUT);
        }
    }
    // Check crypt1
    uint8_t v{};
    return result && readRegister8(STATUS2_REG, v, 0) && (v & 0x08) ? result
                                                                    : m5::stl::make_unexpected(Error::INTERNAL);
}

UnitMFRC522::result_t UnitMFRC522::mifare_transceive(const m5::rfid::Command cmd, const uint8_t block)
{
    uint8_t buf[2]{m5::stl::to_underlying(cmd), block};
    return mifare_transceive(buf, 2);
}

UnitMFRC522::result_t UnitMFRC522::mifare_transceive(const uint8_t* buf, const uint8_t len, const bool usingTimeout)
{
    if (!buf || !len || len > 16) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    uint8_t buf2[18]{};  // Add CRC 2 bytes to tail
    uint8_t validBits{};
    uint8_t rbuf[1]{};
    uint8_t rlen{1};
    uint16_t crc{};

    std::memcpy(buf2, buf, len);
    if (!calculate_crc(crc, buf2, len)) {
        return m5::stl::make_unexpected(Error::CRC);
    }
    buf2[len]     = crc & 0xFF;
    buf2[len + 1] = (crc >> 8) & 0xFF;

    auto result = picc_transceive(rbuf, rlen, buf2, len + 2, validBits);
    /*
      Remark: The MIFARE Increment, Decrement, and Restore command part 2 does notprovide an acknowledgement, so the
      regular time out has to be used instead
     */
    if (!result && result.error() == Error::TIMEOUT && usingTimeout) {
        return {};
    }
    if (!result) {
        return result;
    }
    if ((rlen != 1 || validBits != 4) || rbuf[0] != MIFARE_ACK) {
        M5_LIB_LOGE("NACK %u:%u %02X", rlen, validBits, rbuf[0]);
        return m5::stl::make_unexpected(Error::MIFARE_NACK);
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::dump_sector_structure(const UID& uid, const MifareKey& key)
{
    uint8_t sectors = get_number_of_sectors(uid.type);
    if (!sectors) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    puts(
        "Sec[Blk]:00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F [Access]\n"
        "-----------------------------------------------------------------");

    result_t result{};
    for (int_fast8_t sector = 0; sector < sectors; ++sector) {
        auto sblock = get_sector_trailer_block_from_sector(sector);
        result      = mifareAuthenticateA(uid, sblock, key);
        result      = result ? dump_sector(sector) : result;
        if (!result) {
            printf("%2d) ERROR %02X\n", sector, m5::stl::to_underlying(result.error()));
        }
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::dump_sector(const uint8_t sector)
{
    // Sector 0~31 has 4 blocks, 32-39 has 16 blocks
    const uint8_t blocks = (sector < 32) ? 4U : 16U;
    const uint8_t base   = (sector < 32) ? sector * blocks : 128U + (sector - 32) * blocks;

    uint8_t sbuf[18]{};
    uint8_t slen{18};
    uint8_t permissions[4]{};                 // [3] is sector trailer
    const uint8_t saddr = base + blocks - 1;  //  sector traler

    // Read sector trailer
    auto result = read_block(sbuf, slen, saddr);
    if (!result) {
        return result;
    }
    bool error = !decode_access_bits(permissions, sbuf + 6 /* Access bits offset */);
    //    M5_LIB_LOGW(">> S:%u => %u [%u,%u,%u,%u]", sector, saddr, permissions[0], permissions[1], permissions[2],
    //                permissions[3]);

    // Data
    for (int_fast8_t i = 0; i < blocks - 1; ++i) {
        uint8_t dbuf[18]{};
        uint8_t dlen{18};
        uint8_t daddr = base + i;
        result        = read_block(dbuf, dlen, daddr);
        if (!result) {
            break;
        }
        const uint8_t poffset      = (blocks == 4) ? i : i / 5;
        const uint8_t permission   = permissions[poffset];
        const bool show_permission = (blocks == 4) ? true : (i % 5) == 0;
        dump_block(dbuf, base + i, (i == 0) ? sector : -1, show_permission ? permission : 0xFF, error,
                   is_value_block_permission(permission));
    }
    // Sector trailer
    dump_block(sbuf, saddr, -1, permissions[3], error);

    return result;
}

UnitMFRC522::result_t UnitMFRC522::dump_page_structure(const uint8_t maxPage)
{
    result_t result{};
    puts(
        "Page    :00 01 02 03\n"
        "--------------------");

    for (uint_fast8_t page = 0; page < maxPage; page += 4) {
        result = dump_page(page);
        if (!result) {
            break;
        }
    }
    return result;
}

UnitMFRC522::result_t UnitMFRC522::dump_page(const uint8_t page)
{
    uint8_t buf[16 + 2 /*CRC*/]{};
    uint8_t blen{18};
    result_t result{};
    uint8_t baddr = page & ~0x03;

    result = read_block(buf, blen, baddr);  // 16bytes(4 pages)
    if (result) {
        for (int_fast8_t off = 0; off < 4; ++off) {
            auto idx = off << 2;
            printf("[%03d/%02X]:%02X %02X %02X %02X\n", baddr + off, baddr + off, buf[idx + 0], buf[idx + 1],
                   buf[idx + 2], buf[idx + 3]);
        }
    } else {
        for (int_fast8_t off = 0; off < 4; ++off) {
            printf("[%3d/%02X] ERROR %02X\n", baddr + off, baddr + off, m5::stl::to_underlying(result.error()));
        }
    }
    return result;
}

bool UnitMFRC522::self_test()
{
    // 16.1.1 Self teest
    // 1) Perform a soft reset.
    if (!softReset()) {
        M5_LIB_LOGE("Failed to reset");
        return false;
    }
    // 2) Clear the internal buffer by writing 25 bytes of 00h and implement the Config command.
    std::array<uint8_t, 25> zero{};
    if (!flush_fifo_buffer() ||                                     // Flush FIFO
        !writeRegister(FIFO_DATA_REG, zero.data(), zero.size()) ||  // Write 25 bytes 0x00 to FIFO
        !write_pcd_command(mfrc522::Command::Mem)) {                // Stores 25 bytes into the internal buffer
        M5_LIB_LOGE("Failed to clear or store");
        return false;
    }

    // 3) Enable the self test by writing 09h to the AutoTestReg register.
    if (!writeRegister8(AUTO_TEST_REG, 0x09)) {
        M5_LIB_LOGE("Failed to autitest");
        return false;
    }

    // 4) Write 00h to the FIFO buffer.
    if (!writeRegister8(FIFO_DATA_REG, 0x00)) {
        M5_LIB_LOGE("Failed to write");
        return false;
    }

    // 5) Start the self test with the CalcCRC command.
    if (!write_pcd_command(mfrc522::Command::CalcCRC)) {
        M5_LIB_LOGE("Failed to calcCRC");
        return false;
    }

    // 6) The self test is initiated.

    // 7) When the self test has completed, the FIFO buffer contains the following 64 bytes.
    if (!wait_div_irq(0x04, 1000)) {
        M5_LIB_LOGE("Timeout");
        return false;
    }

    if (!write_pcd_command(mfrc522::Command::Idle)) {
        M5_LIB_LOGE("Failed to idle");
        return false;
    }

    std::array<uint8_t, 64> buf{};
    if (!readRegister(FIFO_DATA_REG, buf.data(), buf.size(), 1)) {
        M5_LIB_LOGE("Failed to read");
        return false;
    }
    // M5_DUMPI(buf.data(), buf.size());

    if (!writeRegister8(AUTO_TEST_REG, 0x00)) {  // To normal operation
        M5_LIB_LOGE("Failed to end");
        return false;
    }

    uint8_t ver{};
    if (!readRegister8(VERSION_REG, ver, 1)) {
        M5_LIB_LOGE("Failed to read version");
        return false;
    }
    const std::array<uint8_t, 64>* firm{};
    switch (ver) {
        case 0x88:
            firm = &firmware_referenceClone;
            break;
        case 0x90:
            firm = &firmware_referenceV0_0;
            break;
        case 0x91:
            firm = &firmware_referenceV1_0;
            break;
        case 0x92:
            firm = &firmware_referenceV2_0;
            break;
        default:
            M5_LIB_LOGE("Unknown version %x", ver);
            return false;
    }
    return firm && (*firm == buf);
}

UnitMFRC522::result_t UnitMFRC522::ntag_get_version(uint8_t* rbuf, uint8_t& rlen)
{
    uint8_t cmd[3]{m5::stl::to_underlying(m5::rfid::Command::GET_VERSION)};
    uint16_t crc{};

    if (!rbuf || rlen < 10) {
        return m5::stl::make_unexpected(Error::ARGUMENT);
    }

    if (!calculate_crc(crc, cmd, 1)) {
        return m5::stl::make_unexpected(Error::CRC);
    }
    cmd[1] = crc & 0xFF;
    cmd[2] = (crc >> 8) & 0xFF;

    uint8_t validBits{};
    return picc_transceive(rbuf, rlen, cmd, m5::stl::size(cmd), validBits, 0, true);
}

bool UnitMFRC522::ntag_check_format(const UID& uid)
{
    if (uid.canNFC()) {
        uint8_t rbuf[18]{};
        uint8_t rlen{18};
        return read_block(rbuf, rlen, 0) && rbuf[12] == NFC_MAGIC_NO && rbuf[13] == NFC_VERSION;
    }
    return false;
}
UnitMFRC522::result_t UnitMFRC522::ntag_fast_read(uint8_t* rbuf, uint8_t& rlen, const uint8_t saddr,
                                                  const uint8_t eaddr)
{
    // M5_LIB_LOGW(">>>> S:%u E:%u rlen:%u", saddr, eaddr, rlen);

    uint8_t cmd[5]{m5::stl::to_underlying(m5::rfid::Command::FAST_READ), saddr, eaddr};
    uint8_t validBits{};
    uint16_t crc{};
    if (!calculate_crc(crc, cmd, 3)) {
        return m5::stl::make_unexpected(Error::CRC);
    }
    cmd[3] = crc & 0xFF;
    cmd[4] = (crc >> 8) & 0xFF;
    return picc_transceive(rbuf, rlen, cmd, 5, validBits, 0, true);
}

}  // namespace unit
}  // namespace m5
