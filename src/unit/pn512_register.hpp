/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file pn512_register.hpp
  @brief PN512-specific register addresses (WS1850S is PN512-compatible silicon)
  @details
    NXP PN512 extends the MFRC522 register map with NFC-B / NFC-F support.
    These constants are reserved/undefined on pure MFRC522 silicon and are
    only valid on PN512 (and WS1850S, which is PN512-compatible internally).
    Common registers shared with MFRC522 (CWGsP/ModGsP/RxThreshold) live in
    unit_MFRC522.hpp and are NOT duplicated here.
*/
#ifndef M5_UNIT_RFID_PN512_REGISTER_HPP
#define M5_UNIT_RFID_PN512_REGISTER_HPP

#include <cstdint>

namespace m5 {
namespace unit {
/*!
  @namespace pn512
  @brief PN512-specific register addresses (NFC-B / NFC-F extensions)
 */
namespace pn512 {

//! TypeBReg (address 0x1E): RxSOFReq, RxEOFReq, EOFSOFWidth, NoTxSOF, NoTxEOF, TxEGT
constexpr uint8_t TYPE_B_REG{0x1E};

}  // namespace pn512
}  // namespace unit
}  // namespace m5

#endif
