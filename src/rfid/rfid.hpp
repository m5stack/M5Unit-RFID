/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file rfid.hpp
  @brief Definition for RFID
*/
#ifndef M5_UNIT_RFID_RFID_RFID_HPP
#define M5_UNIT_RFID_RFID_RFID_HPP

#include <cstdint>

namespace m5 {
/*!
  @namespace rfid
  @brief For RFID
 */
namespace rfid {

/*!
  @enum Command
  @brief ISO-14443-3 and MIFARE commands
 */
enum class Command : uint8_t {
    // ISO/IEC 14443-3
    REQA          = 0x26,  //!< Reequest
    WUPA          = 0x52,  //!< Wake-up
    HLTA          = 0x50,  //!< Halt
    SELECT_CL1    = 0x93,  //!< Anticollison/Select CL1
    SELECT_CL2    = 0x95,  //!< Anticollison/Select CL2
    SELECT_CL3    = 0x97,  //!< Anticollison/Select CL3
    SELCT_CL1_OPT = 0x92,  // Select cascade level 1 and swich bit rate to fc/64 after receive SAK
    SELCT_CL2_OPT = 0x94,  // Select cascade level 2 and swich bit rate to fc/64 after receive SAK
    SELCT_CL3_OPT = 0x96,  // Select cascade level 2 and swich bit rate to fc/64 after receive SAK

    // MIFARE
    AUTH_WITH_KEY_A = 0x60,  //!< Authentication with Key A for Classic
    AUTH_WITH_KEY_B = 0x61,  //!< Authentication with Key B for Classic
    AUTHENTICATE_1  = 0x1A,  //!< Authentication 1st for UltraLightC
    AUTHENTICATE_2  = 0xAF,  //!< Authentication 2nd for UltraLightC
    READ            = 0x30,  //!< MIFARE read
    WRITE           = 0xA0,  //!< MIFARE write
    WRITE_UL        = 0xA2,  //!< MIFARE write for UltraLight/C
    DECREMENT       = 0xC0,  //!< MIFARE decrement value block
    INCREMENT       = 0xC1,  //!< MIFARE increment value block
    RESTORE         = 0xC2,  //!< MIFARE reads the contents of a value block into the internal Transfer Buffer
    TRANSFER        = 0xB0,  //!< MIFARE writes the contents of the internal Transfer Buffer to a block

    PERSONALIZE_UID_USAGE = 0x40,  //!< Personalize UID Usage
    SET_MOD_TYPE          = 0x43,  //!< SET_MOD_TYPE

    RATS = 0x0E,
};

/*!
  @struct ATQA
  @brief Answer To Request, Type A
 */
struct ATQA {
    uint8_t RFU() const
    {
        return value & (1U << 5);
    }
    bool HB0() const
    {
        return value & (1U << 8);
    }
    bool HB1() const
    {
        return value & (1U << 9);
    }
    bool HB2() const
    {
        return value & (1U << 10);
    }
    // single:4 double:7 triple:10
    uint8_t uidLength() const
    {
        uint8_t ulen = (value >> 6) & 0x03;
        return (ulen != 0x03) ? 3 + ulen * 3 + 1 : 0;
    };
    uint16_t value{};
};

}  // namespace rfid
}  // namespace m5
#endif
