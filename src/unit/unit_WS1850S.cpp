/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_WS1850S.cpp
  @brief WS1850S Unit for M5UnitUnified
*/
#include "unit_WS1850S.hpp"
#include "pn512_register.hpp"
#include <M5Utility.hpp>
#include <nfc/b/nfcb.hpp>
#include <algorithm>

namespace {
// VERSION_REG value read from actual WS1850S hardware.
// Not documented in the WS1850S datasheet (MFRC522 returns 0x91 or 0x92).
constexpr uint8_t ws1850s_firmware_version{0x15};
}  // namespace

using namespace m5::utility::mmh3;
using namespace m5::unit::types;

namespace m5 {
namespace unit {

using namespace mfrc522;
using namespace mfrc522::command;

// class UnitWS1850S
const char UnitWS1850S::name[] = "UnitWS1850S";
const types::uid_t UnitWS1850S::uid{"UnitWS1850S"_mmh3};
const types::attr_t UnitWS1850S::attr{attribute::AccessI2C};

bool UnitWS1850S::begin()
{
    uint8_t ver{};
    if (!readRegister8(VERSION_REG, ver, 0) || ver != ws1850s_firmware_version) {
        M5_LIB_LOGE("Cannot detect WS1850S %x", ver);
        return false;
    }
    return UnitMFRC522::begin();
}

/*!
  @brief self test
  @return Always false
  @warning It seems to be compatible in function, but may not be compatible in behavior
 */
bool UnitWS1850S::self_test()
{
    M5_LIB_LOGE("Self test is not supported");
    // Is AutoTestReg of the WS1850S read-only register???
    return false;
}

bool UnitWS1850S::configureNFCMode(const m5::nfc::NFC mode)
{
    switch (mode) {
        case m5::nfc::NFC::A:
            return configure_nfca();
        case m5::nfc::NFC::B:
            return configure_nfcb();
        default:
            M5_LIB_LOGE("Unsupported NFC mode");
            return false;
    }
}

bool UnitWS1850S::configure_nfca()
{
    if (!turnOffAntenna()) return false;

    // ModeReg: full write of _cfg_ws1850s.mode_reg (default 0x3D includes CRCPreset[1:0]=01 for CRC_A).
    // Full write ensures NFC-A state regardless of any prior register modifications.
    if (!writeRegister8(MODE_REG, _cfg_ws1850s.mode_reg)) return false;

    // NFC-A framing: TxCRCEn=0, RxCRCEn=0 (M5Unit-NFC appends/validates CRC_A in software).
    // TxFraming/RxFraming=00 (Type A). Hardware CRC enabled would double-CRC the frame.
    if (!writeRegister8(TX_MODE_REG, static_cast<uint8_t>(0x00))) return false;
    if (!writeRegister8(RX_MODE_REG, static_cast<uint8_t>(0x00))) return false;

    // TxASKReg: Force100ASK=1 (0x40) — required for ISO/IEC 14443-3 Type A 100% ASK modulation.
    // Matches MFRC522::begin() default and the previously-working state.
    if (!writeRegister8(TX_ASK_REG, static_cast<uint8_t>(0x40))) return false;

    // Antenna driver conductance (reset defaults)
    if (!writeCWGsP(0x20)) return false;
    if (!writeModGsP(0x20)) return false;
    // RxThreshold: WS1850S ignores writes to 0x18 (verified by 0x00-0xFF scan).
    // Chip uses fixed internal value (reset default 0x84) for the receive decoder.

    // TypeBReg: reset to 0x00 in case NFC-B mode set it (PN512-compatible silicon only).
    // Harmless no-op on pure MFRC522 silicon.
    if (!writeTypeBReg(0x00)) return false;

    if (!turnOnAntenna()) return false;
    _mode = m5::nfc::NFC::A;
    return true;
}

bool UnitWS1850S::configure_nfcb()
{
    // Antenna OFF before reconfiguring
    if (!turnOffAntenna()) return false;

    // ModeReg: full write of _cfg_ws1850s.mode_reg with CRCPreset[1:0] forced to 0b11 (CRC_B 0xFFFF
    // preset). Full write ensures NFC-B state regardless of any prior register modifications.
    if (!writeRegister8(MODE_REG, static_cast<uint8_t>((_cfg_ws1850s.mode_reg & 0xFC) | 0x03))) return false;

    // TxModeReg: TxCRCEn=0 (software CRC), TxSpeed=000 (106 kbit), TxFraming=0b11 (Type B)
    if (!writeRegister8(TX_MODE_REG, static_cast<uint8_t>(0x03))) return false;

    // RxModeReg: RxCRCEn=0, RxSpeed=000, RxFraming=0b11 (Type B)
    if (!writeRegister8(RX_MODE_REG, static_cast<uint8_t>(0x03))) return false;

    // Fixed Type B physical layer constants (per ISO/IEC 14443-3 and PN512 datasheet)
    // 0x10: EOFSOFWidth=1 (regulation-compliant SOF/EOF length). RxSOFReq=0 / RxEOFReq=0
    // are intentionally cleared so frames with or without SOF/EOF are accepted (lenient Rx).
    constexpr uint8_t TYPE_B_REG_VALUE{0x10};
    constexpr uint8_t CW_GSP_VALUE{0x3F};  // Maximum no-modulation conductance

    // ASK depth (0-15) → ModGsP register value lookup
    constexpr uint8_t MOD_GSP_TABLE[16] = {
        0x3F, 0x3A, 0x35, 0x30, 0x2C, 0x28, 0x24, 0x20, 0x1E, 0x1C, 0x1B, 0x1A, 0x18, 0x16, 0x14, 0x10,
    };
    const uint8_t depth   = std::min<uint8_t>(_cfg_ws1850s.nfcb_ask_depth, 15);
    const uint8_t mod_gsp = MOD_GSP_TABLE[depth];

    // PN512 TypeBReg (address 0x1E): RxSOFReq/RxEOFReq/EOFSOFWidth/NoTxSOF/NoTxEOF/TxEGT.
    // Effective only if the silicon is PN512-compatible; ignored otherwise.
    if (!writeTypeBReg(TYPE_B_REG_VALUE)) return false;

    // TxAutoReg/TxASKReg (0x15): release Force100ASK so the silicon emits 8-30% ASK
    if (!writeRegister8(TX_ASK_REG, static_cast<uint8_t>(0x00))) return false;

    // Antenna driver tuning for Type B (~10% ASK depth)
    if (!writeCWGsP(CW_GSP_VALUE)) return false;
    if (!writeModGsP(mod_gsp)) return false;

        // RxThresholdReg (0x18) intentionally not written: WS1850S silently ignores writes
        // (verified by 0x00-0xFF scan). Chip uses fixed internal value (0x84) for receiver.

#if 0    
    // Diagnostic readback: detect if silicon honored NFC-B framing settings.
    // Pure MFRC522 clones reject TxFraming/RxFraming=11 (Type B) and leave 0x80/0x80.
    {
        uint8_t tx_mode{}, rx_mode{}, mode_reg{}, type_b_reg{};
        readRegister8(TX_MODE_REG, tx_mode, 0);
        readRegister8(RX_MODE_REG, rx_mode, 0);
        readRegister8(MODE_REG, mode_reg, 0);
        readRegister8(m5::unit::pn512::TYPE_B_REG, type_b_reg, 0);
        const bool pn512_like = ((tx_mode & 0x03) == 0x03) && ((rx_mode & 0x03) == 0x03);
        M5_LIB_LOGD(
            "WS1850S NFC-B readback: TxMode=%02X RxMode=%02X Mode=%02X TypeB=%02X => %s", tx_mode, rx_mode, mode_reg,
            type_b_reg,
            pn512_like ? "PN512-compatible (Type B framing accepted)" : "MFRC522 clone (Type B framing rejected)");
    }
#endif

    if (!turnOnAntenna()) return false;
    _mode = m5::nfc::NFC::B;
    return true;
}

bool UnitWS1850S::nfcbTransceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                                 const uint32_t timeout_ms)
{
    const uint16_t rx_len_org = rx_len;
    rx_len                    = 0;

    if (!rx || !rx_len_org || !tx || !tx_len || static_cast<uint32_t>(tx_len) + 2 > MAX_FIFO_DEPTH) {
        return false;
    }

    // CRC_B via PN512 hardware CRC coprocessor (CRCPreset=0xFFFF set by configure_nfcb()).
    // PN512 lacks XOR-out; apply CRC_B xorOut=0xFFFF manually.
    uint16_t crc = 0;
    if (!calculateCRC(crc, tx, tx_len)) {
        return false;
    }
    crc ^= 0xFFFF;

    // Build frame: payload | CRC_B (LSB first)
    uint8_t frame[MAX_FIFO_DEPTH]{};
    memcpy(frame, tx, tx_len);
    frame[tx_len]     = crc & 0xFF;
    frame[tx_len + 1] = (crc >> 8) & 0xFF;

    uint8_t discard{};
    uint8_t tmp[MAX_FIFO_DEPTH]{};
    uint16_t rx_tmp = std::min<uint16_t>(rx_len_org, static_cast<uint16_t>(MAX_FIFO_DEPTH));

    // crc=false: keep trailing 2-byte CRC in rx for upper layer to verify
    auto ret = transceive(tmp, rx_tmp, frame, tx_len + 2, timeout_ms, discard, 0, false);

    // Diagnostic: dump WS1850S internal state on failure
    if (!ret || rx_tmp == 0) {
        uint8_t err{}, com_irq{}, fifo{};
        readRegister8(ERROR_REG, err, 0);
        readRegister8(COM_IRQ_REG, com_irq, 0);
        readRegister8(FIFO_LEVEL_REG, fifo, 0);
        M5_LIB_LOGD("Failed: ret:%u rx_tmp:%u tx[%u] ERR=%02X IRQ=%02X FIFO=%u", ret, rx_tmp, tx_len, err, com_irq,
                    fifo);
    }

    rx_len = std::min<uint16_t>(rx_len_org, rx_tmp);
    memcpy(rx, tmp, rx_len);
    return ret && rx_len;
}

bool UnitWS1850S::readTypeBReg(uint8_t& value)
{
    return readRegister8(m5::unit::pn512::TYPE_B_REG, value, 0);
}

bool UnitWS1850S::writeTypeBReg(const uint8_t value)
{
    if (value & 0x20) {  // bit[5] is RFU per PN512 datasheet 8.2.2.15
        M5_LIB_LOGE("TypeBReg reserved bit[5] must be 0 (got 0x%02X)", value);
        return false;
    }
    return writeRegister8(m5::unit::pn512::TYPE_B_REG, value);
}

}  // namespace unit
}  // namespace m5
