/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for M5Unit-NFC/RFID
  Value block for MIFARE classic
  This example is shared with M5Unit-RFID
*/
// *************************************************************
// Choose ONE define symbol to match the unit/board you are using
// *************************************************************
#if !defined(USING_UNIT_NFC) && !defined(USING_CAP_CC1101) && !defined(USING_UNIT_RFID2) && \
    !defined(USING_M5DIAL_BUILTIN_WS1850S)
// For UnitNFC (ST25R3916, I2C)
// #define USING_UNIT_NFC
// For CapCC1101NFC (ST25R3916, SPI)
// #define USING_CAP_CC1101
// For UnitRFID2 (WS1850S external, I2C GROVE)
// #define USING_UNIT_RFID2
// For M5Dial Builtin WS1850S (internal I2C)
// #define USING_M5DIAL_BUILTIN_WS1850S
#endif
#include "main/ValueBlock.cpp"
