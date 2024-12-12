/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file M5UnitUnifiedRFID.hpp
  @brief Main header of M5Unit-RFID

  @mainpage M5Unit-RFID
  Library for Unit-RFID using M5UnitUnified.
*/
#ifndef M5_UNIT_UNIFIED_RFID_HPP
#define M5_UNIT_UNIFIED_RFID_HPP

#include "rfid/rfid.hpp"
#include "rfid/mifare.hpp"
#include "rfid/nfc/nfc.hpp"

#include "unit/unit_MFRC522.hpp"
#include "unit/unit_WS1850S.hpp"

/*!
  @namespace m5
  @brief Top level namespace of M5stack
 */
namespace m5 {

/*!
  @namespace unit
  @brief Unit-related namespace
 */
namespace unit {

using UnitRFID  = m5::unit::UnitMFRC522;
using UnitRFID2 = m5::unit::UnitWS1850S;

}  // namespace unit
}  // namespace m5
#endif
