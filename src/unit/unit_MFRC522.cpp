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

inline bool has_collision(const uint8_t err)
{
    return err & ERROR_BIT_COLLISION;
}

inline bool has_timeout(const uint8_t err)
{
    return err & ERROR_BIT_TIMEOUT;
}

uint16_t calculate_reload(const uint32_t timeout_ms, const uint16_t tprescaler)
{
    const uint32_t denominator = (2 * (uint32_t)tprescaler) + 1;
    const uint32_t reload      = (timeout_ms * 13560) / denominator;
    if (reload > 0xFFFF) {
        return 0xFFFF;
    }
    return (uint16_t)reload;
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
    auto ret = readTPrescaler(_tprescaler) && writeRegister8(MODE_REG, _cfg.mode_reg) &&
               writeReceiverGain(_cfg.receiver_gain) && (_cfg.enable_antenna ? turnOnAntenna() : true);

#if 0
    uint8_t tx_control{}, demod_reg{}, rf_cfg{}, rx_thresh{};
    readRegister8(TX_CONTROL_REG, tx_control, 0);
    readRegister8(DEMOD_REG, demod_reg, 0);
    readRegister8(RFC_FG_REG, rf_cfg, 0);
    readRegister8(RX_THRESHOLD_REG, rx_thresh, 0);
    M5_LIB_LOGE("TxCtrl=%02X Demod=%02X RfCfg=%02X RxTh=%02X", tx_control, demod_reg, rf_cfg, rx_thresh);

    uint8_t rx_sel{}, mod_width{};
    readRegister8(RX_SEL_REG, rx_sel, 0);  // Include RxWait
    readRegister8(MOD_WIDTH_REG, mod_width, 0);
    M5_LIB_LOGE("RxSel=%02X ModWidth=%02X", rx_sel, mod_width);

    uint8_t mf_rx{};
    readRegister8(MF_RX_REG, mf_rx, 0);
    M5_LIB_LOGE("MfRx=%02X (ParityDisable=%d)", mf_rx, (mf_rx >> 4) & 1);
#endif

    return ret;
}

void UnitMFRC522::update(const bool /* force */)
{
    /* nop */
}

bool UnitMFRC522::nfcaTransceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                                 const uint32_t timeout_ms)
{
    const uint16_t rx_len_org = rx_len;
    rx_len                    = 0;

    if (!rx || !rx_len_org || !tx || !tx_len) {
        return false;
    }

    uint8_t frame[tx_len + 2 /*CRC*/]{};
    uint16_t crc{};
    if (!calculate_crc(crc, tx, tx_len)) {
        return false;
    }
    memcpy(frame, tx, tx_len);
    frame[tx_len]     = crc & 0xFF;
    frame[tx_len + 1] = crc >> 8;

    uint8_t discard{};
    uint8_t tmp[rx_len_org + 2 /*CRC*/]{};
    uint16_t rx_tmp = sizeof(tmp);
    // uint8_t error{};

    auto ret = transceive(tmp, rx_tmp, frame, sizeof(frame), timeout_ms, discard, 0, true);
    rx_len   = std::min<uint16_t>(rx_len_org, rx_tmp);
    memcpy(rx, tmp, rx_len);
    return ret && rx_len;
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

bool UnitMFRC522::readTPrescaler(uint16_t& tprescaler)
{
    uint8_t v[2]{};
    if (readRegister8(TMODE_REG, v[0], 0) && readRegister8(TPRESCALER_REG_L, v[1], 0)) {
        tprescaler = ((v[0] & 0x0F) << 8) | v[1];
        return true;
    }
    return false;
}

bool UnitMFRC522::writeTPrescaler(const uint16_t tprescaler)
{
    uint8_t tm{};
    if (readRegister8(TMODE_REG, tm, 0)) {  // read TMODE_REG [7:4] and Tprescale high 4bits [3:0]
        tm = (tm & 0xF0) | ((tprescaler >> 8) & 0x0F);
        if (writeRegister8(TMODE_REG, tm) && writeRegister8(TPRESCALER_REG_L, tprescaler & 0xFF)) {
            _tprescaler = tprescaler;
            return true;
        }
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

    // Shortframe (Without CRC)
    if (clear_bit_register8(COLL_REG, 0x80)) {
        auto result = transceive(rbuf, rlen, &cmd, 1, TIMEOUT_REQ_WUP, valid_bits, 0, false, &err);
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
        if (!clear_bit_register8(COLL_REG, 0x80)) {
            return false;
        }
        rlen = 5;

        uint8_t err{};
        if (!transceive(rbuf, rlen, buf, sbytes + (sbits != 0), TIMEOUT_ANTICOLL, sbits, sbits, false, &err)) {
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

bool UnitMFRC522::selectWithAnticollision(bool& completed, m5::nfc::a::PICC& picc, const uint8_t lv)
{
    uint8_t buf[9]{};
    uint8_t* rbuf{};
    uint8_t slen{}, align{}, last_bits{};
    uint16_t rlen{}, crc{};

    if (lv < 1 || lv > 3) {
        return false;
    }
    if (!clear_bit_register8(COLL_REG, 0x80)) {
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

    if (!transceive(rbuf, rlen, buf, slen, TIMEOUT_SELECT, last_bits, align)) {
        M5_LIB_LOGD("Failed to select");
        return false;
    }

    // Check SAK CRC
    if (!calculate_crc(crc, rbuf, 1) || crc != *(uint16_t*)(rbuf + 1)) {
        return false;
    }

    // Copy valid uid block
    std::memcpy(picc.uid + (lv - 1) * 3, buf + 2 + (buf[2] == CASCADE_TAG), 4 - (buf[2] == CASCADE_TAG));

    // Completed?
    const uint8_t sak = rbuf[0];
    if (is_sak_completed_14443_4(sak)) {
        picc.size = 1 + lv * 3;
        picc.sak  = sak;
        picc.type =
            Type::ISO_14443_4;  // WARNING: This is a preliminary diagnosis; a more accurate diagnosis is required
        picc.blocks = get_number_of_blocks(picc.type);
        completed   = true;
        return true;
    } else if (is_sak_completed(sak)) {
        picc.size = 1 + lv * 3;
        picc.sak  = sak;
        picc.type =
            sak_to_type(sak);  // WARNING: This is a preliminary diagnosis; a more accurate diagnosis is required
        // Only the Plus X SL2 can be confirmed with sak
        if (picc.type == Type::MIFARE_Plus_2K || picc.type == Type::MIFARE_Plus_4K) {
            picc.sub_type_plus  = SubTypePlus::X;
            picc.security_level = 2;
        }
        picc.blocks = get_number_of_blocks(picc.type);
        completed   = true;
    }
    return completed || has_sak_dependent_bit(sak);  // completed or continue
}

bool UnitMFRC522::select(const PICC& picc)
{
    if (!picc.valid()) {
        return false;
    }

    bool completed{};
    uint8_t select_frame[9] = {0x93, 0x70};
    uint8_t rbuf[3]{}, slen{9}, last_bits{}, align{}, lv{1}, offset{};
    uint16_t rlen{3};
    do {
        select_frame[0] = 0x91 + lv * 2;
        // Build frame
        if (picc.size > lv * 3 + 1) {
            select_frame[2] = 0x88;
        } else {
            select_frame[2] = picc.uid[offset++];
        }
        select_frame[3] = picc.uid[offset++];
        select_frame[4] = picc.uid[offset++];
        select_frame[5] = picc.uid[offset++];
        select_frame[6] = calculate_bcc8(select_frame + 2, 4);
        uint16_t crc{};
        if (!calculate_crc(crc, select_frame, 7)) {
            return false;
        }
        select_frame[7] = crc & 0xFF;
        select_frame[8] = (crc >> 8) & 0xFF;

        if (!transceive(rbuf, rlen, select_frame, slen, TIMEOUT_SELECT, last_bits, align)) {
            return false;
        }
        // m5::utility::log::dump(rbuf, rlen, false);
        completed = is_sak_completed(rbuf[0]) || is_sak_completed_14443_4(rbuf[0]);
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
    uint8_t txLast{0};
    uint16_t crc{};
    if (!calculate_crc(crc, cmd, 2)) {
        return false;
    }
    cmd[2] = crc & 0xFF;
    cmd[3] = (crc >> 8) & 0xFF;

    uint8_t tmp[16 + 2 /*CRC*/]{};
    uint16_t rlen = sizeof(tmp);
    if (transceive(tmp, rlen, cmd, m5::stl::size(cmd), TIMEOUT_READ, txLast, 0, true /* return with CRC*/) &&
        rlen == 18) {
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
    return mifare_classic_transceive(cmd, 2, TIMEOUT_WRITE1) && mifare_classic_transceive(tx, 16, TIMEOUT_WRITE2);
}

bool UnitMFRC522::hlt()
{
    uint8_t buf[4]{}, txLast{};
    buf[0] = m5::stl::to_underlying(m5::nfc::a::Command::HLTA);

    uint16_t crc{};
    if (!calculate_crc(crc, buf, 2)) {
        return false;
    }
    buf[2] = crc & 0xFF;
    buf[3] = (crc >> 8) & 0xFF;

    if (transmit_command(mfrc522::Command::Transceive, buf, sizeof(buf), txLast)) {  // No recv data
        if (wait_comm_irq(TimerIRq, TIMEOUT_HALT)) {
            return true;
        }
        M5_LIB_LOGE("Timeout");
    }
    return false;
}

// MIFARE
bool UnitMFRC522::mifareClassicStopCrypto1()
{
    return clear_bit_register8(STATUS2_REG, 0x08);
}

bool UnitMFRC522::mifareClassicValueBlock(const m5::nfc::a::Command cmd, const uint8_t block, const uint32_t arg)
{
    if (cmd != m5::nfc::a::Command::DECREMENT && cmd != m5::nfc::a::Command::INCREMENT &&
        cmd != m5::nfc::a::Command::RESTORE && cmd != m5::nfc::a::Command::TRANSFER) {
        return false;
    }

    M5_LIB_LOGD("ValuBlock:%02X %u %u", cmd, block, arg);

    if (!mifare_classic_transceive(cmd, block, TIMEOUT_VALUE_BLOCK)) {
        M5_LIB_LOGE("Failed to command %02X %u %u", cmd, block, arg);
        return false;
    }
    if (cmd == m5::nfc::a::Command::TRANSFER) {
        return true;
    }

    m5::utility::delayMicroseconds(82);  // Wait for command <-> Send (At least 82 us)

    uint8_t arg8[4]{};
    arg8[0] = arg & 0xFF;
    arg8[1] = arg >> 8;
    arg8[2] = arg >> 16;
    arg8[3] = arg >> 24;
    return mifare_classic_transceive(arg8, sizeof(arg8), TIMEOUT_VALUE_BLOCK, true);
}

//
bool UnitMFRC522::mifare_classic_authenticate(const m5::nfc::a::Command cmd, const m5::nfc::a::PICC& picc,
                                              const uint8_t block, const m5::nfc::a::mifare::classic::Key& key)
{
    if ((cmd != m5::nfc::a::Command::AUTH_WITH_KEY_A && cmd != m5::nfc::a::Command::AUTH_WITH_KEY_B) ||
        !picc.isMifareClassic()) {
        return false;
    }

    // MFRC522 10.3.1.9 MFAuthent
    uint8_t buf[12]{m5::stl::to_underlying(cmd), block};
    std::memcpy(buf + 2, key.data(), key.size());
    picc.tail4(buf + 8);
    if (!transmit_command(mfrc522::Command::MFAuthent, buf, m5::stl::size(buf))) {
        return false;
    }
    if (!wait_comm_irq(IdleIRq, TIMEOUT_AUTH1)) {
        M5_LIB_LOGD("Timeout");
        return false;
    }
    // Check enabled crypt1?
    uint8_t v{};
    return readRegister8(STATUS2_REG, v, 0) && (v & 0x08);
}

bool UnitMFRC522::mifare_classic_transceive(const m5::nfc::a::Command cmd, const uint8_t block,
                                            const uint32_t timeout_ms)
{
    uint8_t buf[2]{m5::stl::to_underlying(cmd), block};
    return mifare_classic_transceive(buf, 2, timeout_ms);
}

bool UnitMFRC522::mifare_classic_transceive(const uint8_t* buf, const uint8_t len, const uint32_t timeout_ms,
                                            const bool timeout_success)
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

    auto result = transceive(rbuf, rlen, buf2, len + 2, timeout_ms, validBits, 0, false, &err);
    /*
      Remark: The MIFARE Increment, Decrement, and Restore command part 2 does notprovide an acknowledgement, so the
      regular time out has to be used instead
     */
    if (!result) {
        return timeout_success ? has_timeout(err) : false;
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

//
bool UnitMFRC522::modify_bit_register8(const uint8_t reg, const uint8_t set_mask, const uint8_t clear_mask)
{
    uint8_t v{};
    if (readRegister8(reg, v, 0)) {
        const uint8_t w = (v & ~clear_mask) | set_mask;
        // M5_LIB_LOGE("[%2u]:%02X %02X/%02X => %02X %08o", reg, v, set_mask, clear_mask, w, OCB(w));
        if (w == v) {
            return true;
        }
        return writeRegister8(reg, w);
    }
    return false;
}

bool UnitMFRC522::set_bit_register8(const uint8_t reg, const uint8_t bits)
{
    uint8_t v{};
    if (readRegister8(reg, v, 0)) {
        if (v == (v | bits)) {
            return true;
        }
        return writeRegister8(reg, v | bits);
    }
    return false;
}

bool UnitMFRC522::clear_bit_register8(const uint8_t reg, const uint8_t bits)
{
    uint8_t v{};
    if (readRegister8(reg, v, 0)) {
        if (v == (v & ~bits)) {
            return true;
        }
        return writeRegister8(reg, (v & ~bits));
    }
    return false;
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
    return change_bit_register8(TX_MODE_REG, 0x00, 0x70)     // 106bBd
           && change_bit_register8(RX_MODE_REG, 0x00, 0x70)  // 106kBd
           && writeRegister8(MOD_WIDTH_REG, 0x26);           //
}

bool UnitMFRC522::flush_fifo_buffer()
{
    return writeRegister8(FIFO_LEVEL_REG, 0x80);
}

bool UnitMFRC522::wait_comm_irq(const uint8_t irq, const uint32_t timeout_ms)
{
    uint8_t v{};
    const uint16_t reload = calculate_reload(timeout_ms, _tprescaler);
    if (!writeRegister8(TRELOAD_REG_H, reload >> 8) || !writeRegister8(TRELOAD_REG_L, reload & 0xFF)) {
        M5_LIB_LOGE("Failed to TReload");
        return false;
    }

    auto timeout_at = m5::utility::millis() + timeout_ms;
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

bool UnitMFRC522::wait_div_irq(const uint8_t irq, const uint32_t timeout_ms)
{
    uint8_t v{};
    auto timeout_at = m5::utility::millis() + timeout_ms;
    do {
        readRegister8(DIV_IRQ_REG, v, 0);
        if (v & irq) {
            return true;
        }
        std::this_thread::yield();
    } while (m5::utility::millis() <= timeout_at);
    return false;
}

bool UnitMFRC522::transmit_command(const mfrc522::Command cmd, const uint8_t* buf, const uint8_t len,
                                   const uint8_t txLast, const uint8_t rxAlign)
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
        !set_bit_register8(BIT_FRAMING_REG, 0x80)      // StartSend:7 Start the transmission
    ) {
        M5_LIB_LOGE("Failed to send");
        return false;
    }
    return true;
}

bool UnitMFRC522::transceive(uint8_t* rbuf, uint16_t& rx_len, const uint8_t* buf, const uint16_t len,
                             const uint32_t timeout_ms, uint8_t& validBits, const uint8_t rxAlign, const bool crc,
                             uint8_t* error)
{
    auto rx_len_org = rx_len;
    rx_len          = 0;

    if (!rbuf || !rx_len_org || !buf || !len) {
        return false;
    }
    if (error) {
        *error = 0;
    }

    M5_LIB_LOGV("txLast:%u rxAlign:%u", validBits, rxAlign);
    if (!transmit_command(mfrc522::Command::Transceive, buf, len, validBits, rxAlign)) {
        return false;
    }

    // Wait for command completion
    if (!wait_comm_irq(RxIRq | IdleIRq, timeout_ms)) {
#if 0
        uint8_t com_irq{}, err_reg{}, fifo_level{};
        readRegister8(COM_IRQ_REG, com_irq, 0);
        readRegister8(ERROR_REG, err_reg, 0);
        readRegister8(FIFO_LEVEL_REG, fifo_level, 0);
        M5_LIB_LOGE("Timeout: COM_IRQ=%02X ERR=%02X FIFO=%u", com_irq, err_reg, fifo_level);
#endif
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
        M5_LIB_LOGD("ERROR %02X", err);
        return false;
    }

    // Read FIFO
    uint8_t fifo_len{};
    uint8_t valid{};
    if (!readRegister8(FIFO_LEVEL_REG, fifo_len, 0)) {
        return false;
    }

    M5_LIB_LOGV("- Recv:%u", fifo_len);
    if (fifo_len > rx_len_org) {
        M5_LIB_LOGD("Not enough %u/%u", fifo_len, rx_len_org);
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

    rx_len = fifo_len;

    // Check collision
    if (has_collision(err)) {
        return false;
    }

    // CRC
    if (crc) {
        if (fifo_len == 1 && valid == 4) {  // 4bit ACK
            if (rbuf[0] != MIFARE_ACK) {
                M5_LIB_LOGE("MIFARE ACK %02X", rbuf[0]);
            }
            return rbuf[0] == MIFARE_ACK;
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

    M5_LIB_LOGV("OK:rxLast:%u", validBits);
    return true;
}

}  // namespace unit
}  // namespace m5
