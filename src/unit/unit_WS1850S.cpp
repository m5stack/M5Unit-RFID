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
#include <M5Utility.hpp>

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
    M5_LIB_LOGE("DON'T support it");
    // Is AutoTestReg of the WS1850S read-only register???
    return false;
}

}  // namespace unit
}  // namespace m5
