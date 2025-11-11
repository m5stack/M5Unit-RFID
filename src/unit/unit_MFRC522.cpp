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
#include <M5Utility.hpp>
#include <cassert>
#include <thread>
#include <cstdio>
#include <cinttypes>

using namespace m5::unit::mfrc522;
using namespace m5::unit::mfrc522::command;
using namespace m5::unit::types;
using namespace m5::utility::mmh3;
using namespace m5::nfc::a;
using namespace m5::nfc::a::mifare;
using namespace m5::nfc::a::mifare::classic;

namespace {
constexpr float F_CLOCK = 13.56f * 1000000;  // 13.56 Mhz

constexpr uint8_t NFC_MAGIC_NO{0xE1};
constexpr uint8_t NFC_VERSION{0x10};

// ComIrqReg register
constexpr uint8_t Set1{0x80};
constexpr uint8_t TxIRq{0x40};       // set immediately after the last bit of the transmitted data was sent out
constexpr uint8_t RxIRq{0x20};       // receiver has detected the end of a valid data stream
constexpr uint8_t IdleIRq{0x10};     // If a command terminates
constexpr uint8_t HiAlertIRq{0x08};  // Status1Reg register’s HiAlert bit is set
constexpr uint8_t LoAlertIRq{0x04};  // Status1Reg register’s LoAlert bit is set
constexpr uint8_t ErrIRq{0x02};      // any error bit in the ErrorReg register is set
constexpr uint8_t TimerIRq{0x01};    // the timer decrements the timer value in register TCounterValReg to zero

// ErrorReg
constexpr uint8_t ERROR_BIT_WRITE{0x80};
constexpr uint8_t ERROR_BIT_TEMPERATURE{0x40};
constexpr uint8_t ERROR_BIT_RESERVED{0x20};
constexpr uint8_t ERROR_BIT_OVERFLOW{0x10};
constexpr uint8_t ERROR_BIT_COLLISION{0x08};
constexpr uint8_t ERROR_BIT_CRC{0x04};
constexpr uint8_t ERROR_BIT_PARITY{0x02};
constexpr uint8_t ERROR_BIT_PROTOCOL{0x01};
constexpr uint8_t ERROR_BIT_TIMEOUT{
    0x20};  // Custom Extension: If this bit is used in the future, extend the error to 16 bits

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

inline bool has_collision(const uint8_t err)
{
    return err & ERROR_BIT_COLLISION;
}

inline bool has_timeout(const uint8_t err)
{
    return err & ERROR_BIT_TIMEOUT;
}

#if 0
inline float modulationWidth(const uint8_t tm)
{
    return tm + 1 / F_CLOCK;
}

inline uint8_t bits_to_NVB(const uint8_t bits)
{
    // High nibble:bytes Low nibble:fraction bits
    return (bits >> 3) | (bits & 0x07);
}
#endif

}  // namespace

namespace m5 {
namespace unit {
// class UnitMFRC522
const char UnitMFRC522::name[] = "UnitMFRC522";
const types::uid_t UnitMFRC522::uid{"UnitMFRC522"_mmh3};
const types::attr_t UnitMFRC522::attr{attribute::AccessI2C};
// const UnitMFRC522::MifareKey UnitMFRC522::DEFAULT_CLASSIC_KEY{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

bool UnitMFRC522::request_wakeup(uint16_t& atqa, const bool request)
{
    uint8_t rbuf[2]{};
    uint16_t rlen = 2;  // ATQA 16bit
    uint8_t cmd   = m5::stl::to_underlying(request ? m5::nfc::a::Command::REQA : m5::nfc::a::Command::WUPA);
    uint8_t valid_bits{0x07};
    uint8_t err{};

    atqa = 0;

    //    if (reset_baud_rates() && clear_register_bit(COLL_REG, 0x80)) {
    if (clear_register_bit(COLL_REG, 0x80)) {
        auto result = picc_transceive(rbuf, rlen, &cmd, 1, valid_bits, 0, false, &err);
        if (result && rlen == 2) {
            atqa = ((uint16_t)rbuf[1] << 8) | (uint16_t)rbuf[0];
            M5_LIB_LOGD("ATQA:%04X", atqa);
            return true;
        }
    }
    return has_collision(err);
}

bool UnitMFRC522::anti_collision(const uint8_t lv, uint8_t buf[9])
{
    uint8_t sbytes{2}, sbits{}, cbytes{}, cbits{};
    uint16_t rlen{5};
    uint8_t* rbuf = buf + 2;
    bool collision{};
    uint32_t count{32};  // Max loop count
    uint8_t coll_byte{1};
    // First time (0x9n, 0x20)
    buf[0] = m5::stl::to_underlying(m5::nfc::a::Command::SELECT_CL1) + (lv - 1) * 2;  // 0x93,95,97
    buf[1] = 0x20;                                                                    // first NVB

    do {
        if (!clear_register_bit(COLL_REG, 0x80)) {
            return false;
        }
        rlen = 5;

        uint8_t err{};
        if (!picc_transceive(rbuf, rlen, buf, sbytes + (sbits != 0), sbits, sbits, false, &err)) {
            M5_LIB_LOGD("Failed ANTICOL:%02X", lv);
            return false;
        }
        collision = has_collision(err);

        // M5_LIB_LOGE("RECV %u last:%u>>>>", rlen, sbits);
        // m5::utility::log::dump(rbuf, rlen, false);
        // m5::utility::log::dump(rbuf, rlen + (sbits != 0), false);
        if (collision) {
            M5_LIB_LOGD("Colliion");
            uint8_t col{};
            if (!readRegister8(COLL_REG, col, 0)) {
                return false;
            }
            if (col & 0x20) {
                return false;
            }
            uint8_t coll_offset = col & 0x1F;  // bit offset of collison
            if (!coll_offset) {
                coll_offset = 32;  // 1-32
            }
            cbytes = coll_offset >> 3;
            cbits  = (coll_offset - 1) & 0x07;
            if (rlen) {
                coll_byte = rbuf[cbytes];
                coll_byte |= (1U << cbits);
            }
            M5_LIB_LOGD("   COL:%u bytes, %u bits", cbytes, cbits);
            M5_LIB_LOGD("   coll_byte: %02x", coll_byte);

            sbytes = 2 + cbytes + (cbits == 0x07);
            sbits  = (cbits + 1) & 0x07;
            buf[1] = (sbytes << 4) | sbits;  // NVB

            buf[2 + cbytes] = coll_byte;
            rbuf            = buf + 2 + cbytes;
        }
    } while (collision && count--);

    return !collision;
}

bool UnitMFRC522::selectWithAnticollision(bool& completed, m5::nfc::a::UID& uid, const uint8_t lv)
{
    uint8_t buf[9]{};
    uint8_t* rbuf{};
    uint8_t slen{}, align{}, last_bits{};
    uint16_t rlen{}, crc{};

    if (lv < 1 || lv > 3) {
        return false;
    }
    if (!clear_register_bit(COLL_REG, 0x80)) {
        return false;
    }

    // Anti collision
    if (!anti_collision(lv, buf)) {
        return false;
    }

    // Select
    buf[1] = 0x70;
    if (!calculate_crc(crc, buf, 7)) {
        return false;
    }
    buf[7]    = crc & 0xFF;
    buf[8]    = (crc >> 8) & 0xFF;
    slen      = 9;
    rbuf      = buf + 6;
    rlen      = 3;
    last_bits = 0;

    if (!picc_transceive(rbuf, rlen, buf, slen, last_bits, align)) {
        M5_LIB_LOGD("Failed to select");
        return false;
    }

    // Check SAK CRC
    if (!calculate_crc(crc, rbuf, 1) || crc != *(uint16_t*)(rbuf + 1)) {
        return false;
    }

    // Copy valid uid block
    std::memcpy(uid.uid + (lv - 1) * 3, buf + 2 + (buf[2] == CASCADE_TAG), 4 - (buf[2] == CASCADE_TAG));

    // Completed?
    const uint8_t sak = rbuf[0];
    if (is_sak_completed(sak)) {
        completed = true;
        uid.size  = 1 + lv * 3;
        uid.sak   = sak;
        uid.type = sak_to_type(sak);  // WARNING: This is a preliminary diagnosis; a more accurate diagnosis is required
        uid.blocks = get_number_of_blocks(uid.type);
        // More check for type
        if (uid.type == Type::MIFARE_Ultralight) {
            uint8_t ver[10]{};
            if (ntag_get_version(ver)) {
                uid.type   = version_to_type(ver);
                uid.blocks = get_number_of_blocks(uid.type);
            } else {
                // PICC to IDLE... so need reactivate
                uint16_t discard{};
                completed = request(discard) && select(uid);
            }
        }
    }
    return completed || has_sak_dependent_bit(sak);  // completed or continue
}

bool UnitMFRC522::select(const UID& uid)
{
    if (!uid.valid()) {
        return false;
    }

    bool completed{};
    uint8_t select_frame[9] = {0x93, 0x70};
    uint8_t rbuf[3]{}, slen{9}, last_bits{}, align{}, lv{1}, offset{};
    uint16_t rlen{3};
    do {
        select_frame[0] = 0x91 + lv * 2;
        // Build frame
        if (uid.size > lv * 3 + 1) {
            select_frame[2] = 0x88;
        } else {
            select_frame[2] = uid.uid[offset++];
        }
        select_frame[3] = uid.uid[offset++];
        select_frame[4] = uid.uid[offset++];
        select_frame[5] = uid.uid[offset++];
        select_frame[6] = calculate_bcc8(select_frame + 2, 4);
        uint16_t crc{};
        if (!calculate_crc(crc, select_frame, 7)) {
            return false;
        }
        select_frame[7] = crc & 0xFF;
        select_frame[8] = (crc >> 8) & 0xFF;

        if (!picc_transceive(rbuf, rlen, select_frame, slen, last_bits, align)) {
            return false;
        }
        // m5::utility::log::dump(rbuf, rlen, false);
        completed = is_sak_completed(rbuf[0]);
        ++lv;
    } while (!completed && lv < 4);

    return completed;
}

bool UnitMFRC522::readBlock(uint8_t* rbuf, const uint8_t addr)
{
    if (!rbuf) {
        return false;
    }

    uint8_t cmd[4]{m5::stl::to_underlying(m5::nfc::a::Command::READ), addr};
    uint16_t crc{};
    uint8_t txLast{0};
    if (!calculate_crc(crc, cmd, 2)) {
        return false;
    }
    cmd[2] = crc & 0xFF;
    cmd[3] = (crc >> 8) & 0xFF;

    uint8_t tmp[16 + 2 /*CRC*/]{};
    uint16_t rlen = sizeof(tmp);
    if (picc_transceive(tmp, rlen, cmd, m5::stl::size(cmd), txLast, 0, true /* return with CRC*/) && rlen == 18) {
        memcpy(rbuf, tmp, 16);
        return true;
    }
    return false;
}

bool UnitMFRC522::writeBlock(const uint8_t block, const uint8_t tx[16])
{
    if (!tx) {
        return false;
    }
    uint8_t cmd[2]{m5::stl::to_underlying(m5::nfc::a::Command::WRITE_BLOCK), block};
    return mifare_classic_transceive(cmd, 2) && mifare_classic_transceive(tx, 16);
}

bool UnitMFRC522::writePage(const uint8_t page, const uint8_t tx[4])
{
    if (!tx) {
        return false;
    }
    uint8_t cmd[6]{m5::stl::to_underlying(m5::nfc::a::Command::WRITE_PAGE), page};
    std::memcpy(cmd + 2, tx, 4);
    return mifare_classic_transceive(cmd, m5::stl::size(cmd));
}

bool UnitMFRC522::ntagReadPage(uint8_t* rx, uint16_t& rx_len, const uint8_t spage, const uint8_t epage)
{
    if (!rx || !rx_len) {
        return false;
    }

    uint8_t cmd[5]{m5::stl::to_underlying(m5::nfc::a::Command::FAST_READ), spage, epage};
    uint8_t validBits{};
    uint16_t crc{};
    if (!calculate_crc(crc, cmd, 3)) {
        return false;
    }
    cmd[3] = crc & 0xFF;
    cmd[4] = (crc >> 8) & 0xFF;

    uint8_t tmp[rx_len + 2 /*CRC*/]{};
    uint16_t rlen = sizeof(tmp);
    if (picc_transceive(tmp, rlen, cmd, sizeof(cmd), validBits, 0, true) && rlen == rx_len + 2) {
        memcpy(rx, tmp, std::min<uint16_t>(rx_len, rlen));
        return true;
    }

    M5_LIB_LOGE("ERROR:%u", rlen);

    return false;
}

bool UnitMFRC522::hlt()
{
    uint8_t buf[4]{}, txLast{};
    uint16_t crc{};
    buf[0] = m5::stl::to_underlying(m5::nfc::a::Command::HLTA);

    if (!calculate_crc(crc, buf, 2)) {
        return false;
    }
    buf[2] = crc & 0xFF;
    buf[3] = (crc >> 8) & 0xFF;

    if (picc_send(mfrc522::Command::Transceive, buf, 4, txLast)) {  // No recv data
        if (wait_comm_irq(TimerIRq, 36)) {
            return true;
        }
        M5_LIB_LOGD("Timeout");
    }
    return false;
}

// MIFARE
bool UnitMFRC522::mifareClassicStopCrypto1()
{
    return clear_register_bit(STATUS2_REG, 0x08);
}

bool UnitMFRC522::mifareClassicEnableValueBlock(const UID& uid, const uint8_t block, const Key& keyA, const Key& keyB,
                                                const bool readOnly)

{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0) {
        return false;
    }

    uint8_t permission = readOnly ? 0x01 : 0x06;
    uint8_t buf[18]{};
    uint8_t len{18};
    uint8_t permissions[4]{};
    uint8_t st_block = get_sector_trailer_block(block);
    uint8_t poff     = get_permission_offset(block);

    // Read sector trailer block
    if (!read_block(buf, len, st_block)) {
        return false;
    }
    // M5_LIB_LOGW("R) %02X", result ? 0 : m5::stl::to_underlying(result.error()));
    if (!decode_access_bits(permissions, buf + 6)) {
        return false;
    }
    // Already value block?
    if (permissions[poff] == permission) {
        return true;  // OK
    }
    // Update sector trailer
    permissions[poff] = permission;
    permissions[3]    = 0x03;  // 011: never/keyB /keyA|B/keyB never/keyB
    if (!encode_access_bits(buf + 6, permissions)) {
        return false;
    }
    std::memcpy(buf, keyA.data(), 6);
    std::memcpy(buf + 10, keyB.data(), 6);
    return write_block(st_block, buf, 16);
}

bool UnitMFRC522::mifareClassicDisableValueBlock(const UID& uid, const uint8_t block, const Key& keyA, const Key& keyB,
                                                 const uint8_t permission)
{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0 ||
        is_value_block_permission(permission) || (permission & 0xF8)) {
        return false;
    }

    uint8_t buf[18]{};
    uint8_t len{18};
    uint8_t permissions[4]{};
    uint8_t st_block = get_sector_trailer_block(block);
    uint8_t poff     = get_permission_offset(block);

    // Read sector trailer block
    if (!read_block(buf, len, st_block)) {
        return false;
    }
    // M5_LIB_LOGE("R) %02X", result ? 0 : m5::stl::to_underlying(result.error()));
    if (!decode_access_bits(permissions, buf + 6)) {
        return false;
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
        return false;
    }
    std::memcpy(buf, keyA.data(), 6);
    std::memcpy(buf + 10, keyB.data(), 6);
    return write_block(st_block, buf, 16);  // update sector tailer
}

bool UnitMFRC522::mifareClassicIncrement(const UID& uid, const uint8_t block, const uint32_t delta)
{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0) {
        return false;
    }
    return mifare_classic_transceive(m5::nfc::a::Command::INCREMENT, block) &&
           mifare_classic_transceive((const uint8_t*)&delta, 4, true);
}

bool UnitMFRC522::mifareClassicDecrement(const UID& uid, const uint8_t block, const uint32_t delta)
{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0) {
        return false;
    }
    return mifare_classic_transceive(m5::nfc::a::Command::DECREMENT, block) &&
           mifare_classic_transceive((const uint8_t*)&delta, 4, true);
}

bool UnitMFRC522::mifareClassicRestore(const UID& uid, const uint8_t block)
{
    if (uid.isMifareClassic()) {
        uint8_t dummy[4]{};
        return mifare_classic_transceive(m5::nfc::a::Command::RESTORE, block) &&
               mifare_classic_transceive(dummy, 4, true);
    }
    return false;
}

bool UnitMFRC522::mifareClassicTransfer(const UID& uid, const uint8_t block)
{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0) {
        return false;
    }
    return mifare_classic_transceive(m5::nfc::a::Command::TRANSFER, block);
}

bool UnitMFRC522::mifareClassicReadValue(const UID& uid, int32_t& value, const uint8_t block)
{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0) {
        return false;
    }

    uint8_t buf[18]{};
    uint8_t len{18};

    value = 0;
    if (!read_block(buf, len, block)) {
        return false;
    }
    uint8_t addr{};
    if (!decode_value_block(value, addr, buf)) {
        M5_LIB_LOGW("Block %u is NOT value block format data", block);
        return false;
    }
    return true;
}

bool UnitMFRC522::mifareClassicWriteValue(const UID& uid, const uint8_t block, const int32_t value)
{
    if (!uid.isMifareClassic() || is_sector_trailer_block(block) || block == 0) {
        return false;
    }

    uint8_t buf[16]{};
    encode_value_block(buf, value, block);
    return write_block(block, buf, 16);
}

// NFC
bool UnitMFRC522::nfcWriteChangeToNTAGFormat(const UID& uid)
{
    if (uid.type == Type::MIFARE_Ultralight || uid.type == Type::MIFARE_UltralightC) {
        if (ntag_check_format(uid)) {
            return true;  // Already NFC-A compatible
        }
        uint8_t buf[4] = {NFC_MAGIC_NO, NFC_VERSION};
        buf[3]         = (uid.type == Type::MIFARE_Ultralight) ? 0x06 : 0x18;
        return write_page(3 /*OTP area*/, buf, 4);
    }
    if (uid.supportsNFC()) {
        return true;
    }
    M5_LIB_LOGE("Can not change to NFC format. %u", uid.type);
    return false;
}

/*
  targetBit Bit group  of the tag included in the size calculation
  0x01:Null, 0x02:LockControl 0x04:NDEFMessage, ....
  e.g. 0x06 means LockControl and MemoryControl
  Proprietary and Terminator can not  be excluded
 */
bool UnitMFRC522::ntag_calclate_ndef_message_size(const UID& uid, uint32_t& sz, const uint8_t targetTagBit)
{
    using m5::nfc::ndef::is_terminator_tag;
    using m5::nfc::ndef::is_valid_tag;
    using m5::nfc::ndef::Tag;

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
        if (!read_block(rbuf, rlen, page)) {
            return false;
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

bool UnitMFRC522::nfcReadRequiredSize(const UID& uid, uint32_t& len)
{
    len = 0;
    if (ntag_calclate_ndef_message_size(uid, len, 0x0F /* all tag */)) {
        len = (len + 15) / 16 * 16 + 2 /*CRC*/;
        return true;
    }
    return false;
}

//
bool UnitMFRC522::mifare_classic_authenticate(const m5::nfc::a::Command cmd, const m5::nfc::a::UID& uid,
                                              const uint8_t block, const m5::nfc::a::mifare::classic::Key& key)
{
    if ((cmd != m5::nfc::a::Command::AUTH_WITH_KEY_A && cmd != m5::nfc::a::Command::AUTH_WITH_KEY_B) ||
        !uid.isMifareClassic()) {
        return false;
    }

    // MFRC522 10.3.1.9 MFAuthent
    uint8_t buf[12]{m5::stl::to_underlying(cmd), block};
    std::memcpy(buf + 2, key.data(), key.size());
    uid.tail4(buf + 8);
    if (!picc_send(mfrc522::Command::MFAuthent, buf, m5::stl::size(buf))) {
        return false;
    }
    if (!wait_comm_irq(IdleIRq, 36)) {
        M5_LIB_LOGD("Timeout");
        return false;
    }
    // Check enabled crypt1?
    uint8_t v{};
    return readRegister8(STATUS2_REG, v, 0) && (v & 0x08);
}

bool UnitMFRC522::mifare_classic_transceive(const m5::nfc::a::Command cmd, const uint8_t block)
{
    uint8_t buf[2]{m5::stl::to_underlying(cmd), block};
    return mifare_classic_transceive(buf, 2);
}

bool UnitMFRC522::mifare_classic_transceive(const uint8_t* buf, const uint8_t len, const bool usingTimeout)
{
    if (!buf || !len || len > 16) {
        return false;
    }

    uint8_t buf2[18]{};  // Add CRC 2 bytes to tail
    uint8_t validBits{}, err{};
    uint8_t rbuf[1]{};
    uint16_t rlen{1}, crc{};

    std::memcpy(buf2, buf, len);
    if (!calculate_crc(crc, buf2, len)) {
        return false;
    }
    buf2[len]     = crc & 0xFF;
    buf2[len + 1] = (crc >> 8) & 0xFF;

    auto result = picc_transceive(rbuf, rlen, buf2, len + 2, validBits, 0, false, &err);
    /*
      Remark: The MIFARE Increment, Decrement, and Restore command part 2 does notprovide an acknowledgement, so the
      regular time out has to be used instead
     */
    if (!result) {
        return usingTimeout ? has_timeout(err) : false;
    }

    if ((rlen != 1 || validBits != 4) || rbuf[0] != MIFARE_ACK) {
        M5_LIB_LOGE("NACK %u:%u %02X", rlen, validBits, rbuf[0]);
        return false;
    }
    return true;
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

bool UnitMFRC522::ntag_get_version(uint8_t info[10])
{
    uint8_t cmd[3]{m5::stl::to_underlying(m5::nfc::a::Command::GET_VERSION)};
    uint16_t crc{};

    if (!info || !calculate_crc(crc, cmd, 1)) {
        return false;
    }
    cmd[1] = crc & 0xFF;
    cmd[2] = (crc >> 8) & 0xFF;

    uint8_t validBits{};
    uint16_t rlen{10};
    return picc_transceive(info, rlen, cmd, m5::stl::size(cmd), validBits, 0, true);
}

bool UnitMFRC522::ntag_check_format(const UID& uid)
{
    if (uid.supportsNFC()) {
        uint8_t rbuf[18]{};
        uint8_t rlen{18};
        return read_block(rbuf, rlen, 0) && rbuf[12] == NFC_MAGIC_NO && rbuf[13] == NFC_VERSION;
    }
    return false;
}

bool UnitMFRC522::ntag_fast_read(uint8_t* rbuf, uint16_t& rlen, const uint8_t saddr, const uint8_t eaddr)
{
    // M5_LIB_LOGW(">>>> S:%u E:%u rlen:%u", saddr, eaddr, rlen);

    uint8_t cmd[5]{m5::stl::to_underlying(m5::nfc::a::Command::FAST_READ), saddr, eaddr};
    uint8_t validBits{};
    uint16_t crc{};
    if (!calculate_crc(crc, cmd, 3)) {
        return false;
    }
    cmd[3] = crc & 0xFF;
    cmd[4] = (crc >> 8) & 0xFF;
    return picc_transceive(rbuf, rlen, cmd, sizeof(cmd), validBits, 0, true);
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
            if (v & 0x01) {  // occurs timeout or error
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

bool UnitMFRC522::picc_send(const mfrc522::Command cmd, const uint8_t* buf, const uint8_t len, const uint8_t txLast,
                            const uint8_t rxAlign)
{
    if (!buf || !len) {
        return false;
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
        return false;
    }
    return true;
}

bool UnitMFRC522::picc_transceive(uint8_t* rbuf, uint16_t& rlen, const uint8_t* buf, const uint16_t len,
                                  uint8_t& validBits, const uint8_t rxAlign, const bool crc, uint8_t* error)
{
    if (!rbuf || !rlen || !buf || !len) {
        return false;
    }
    if (error) {
        *error = 0;
    }

    M5_LIB_LOGV("txLast:%u rxAlign:%u", validBits, rxAlign);
    if (!picc_send(mfrc522::Command::Transceive, buf, len, validBits, rxAlign)) {
        return false;
    }

    // Wait for command completion (timeout 36ms)
    if (!wait_comm_irq(RxIRq | IdleIRq, 36)) {
        M5_LIB_LOGD("Timeout");
        if (error) {
            *error = ERROR_BIT_TIMEOUT;
        }
        return false;
    }

    uint8_t err{};
    if (!readRegister8(ERROR_REG, err, 0)) {
        return false;
    }
    if (error) {
        *error = err;
    }

    // Check error
    if (!has_collision(err) && (err & (ERROR_BIT_OVERFLOW | ERROR_BIT_PARITY | ERROR_BIT_PROTOCOL))) {
        M5_LIB_LOGE("ERROR %02X", err);
        return false;
    }

    // Read FIFO
    uint8_t fifo_len{};
    uint8_t valid{};
    if (!readRegister8(FIFO_LEVEL_REG, fifo_len, 0)) {
        return false;
    }
    M5_LIB_LOGV("- Recv:%u", fifo_len);
    if (fifo_len > rlen) {
        M5_LIB_LOGE("Not enough rlen %zu : %u", rlen, fifo_len);
        return false;
    }
    if (!read_register_with_align(FIFO_DATA_REG, rbuf, fifo_len, rxAlign) ||
        // indicates the number of valid bits in the last received byte if this value is 000b, the whole byte is
        // valid
        !readRegister8(CONTROL_REG, valid, 0)) {
        return false;
    }
    valid &= 0x07;
    validBits = valid;  // It's indicates the number of valid bits in the last received

    // Check collision
    if (has_collision(err)) {
        return false;
    }

    // CRC
    if (crc) {
        if (fifo_len == 1 && valid == 4) {
            M5_LIB_LOGE("NG MIFARE Classic NAK %02X", rbuf[0]);
            return false;
        }
        if (fifo_len < 2 || valid) {
            M5_LIB_LOGE("Not in a condition to calculate CRC %u/%u", fifo_len, valid);
            return false;
        }

        uint16_t crc16{};
        uint16_t rcrc16 = ((uint16_t)rbuf[fifo_len - 1] << 8) | rbuf[fifo_len - 2];
        if (!calculate_crc(crc16, rbuf, fifo_len - 2) || crc16 != rcrc16) {
            M5_LIB_LOGE("CRC ERROR C:%04X R:%04X", crc16, rcrc16);
            return false;
        }
    }

    rlen = fifo_len;
    M5_LIB_LOGV("OK:rxLast:%u", validBits);
    return true;
}

}  // namespace unit
}  // namespace m5
