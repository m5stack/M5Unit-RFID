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

#include <M5UnitUnifiedNFC.hpp>  // depends on M5Unit-NFC
#include "unit/unit_MFRC522.hpp"
#include "unit/unit_WS1850S.hpp"

/*!
  @namespace m5
  @brief Top level namespace of M5Stack
 */
namespace m5 {

/*!
  @namespace unit
  @brief Unit-related namespace
 */
namespace unit {

//! @brief Alias for UnitMFRC522
using UnitRFID = m5::unit::UnitMFRC522;
//! @brief Alias for UnitWS1850S
using UnitRFID2 = m5::unit::UnitWS1850S;

}  // namespace unit
}  // namespace m5
#endif
