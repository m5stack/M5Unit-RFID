/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Common functions and definitions for examples
*/
#ifndef M5_UNIT_RFID_EXMAPLES_COMMON_HPP
#define M5_UNIT_RFID_EXMAPLES_COMMON_HPP

#include <rfid/rfid.hpp>
#include <rfid/mifare.hpp>
#include <cstring>

const char* UIDTypeString(const m5::rfid::mifare::UID& uid);
const std::string UIDString(const m5::rfid::mifare::UID& uid);
void printUID(const m5::rfid::mifare::UID& uid);

#endif
