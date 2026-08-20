/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UHF-RFID
*/
#include <gtest/gtest.h>
#include <unit/jrd4035_frame.hpp>
#include <cstdint>
#include <vector>

using namespace m5::unit::jrd4035;

TEST(UHF, Checksum)
{
    // From the protocol document: BB 00 03 00 01 00 [04] 7E
    // Sum of Type..Parameter = 00+03+00+01+00 = 0x04
    const uint8_t body0[] = {0x00, 0x03, 0x00, 0x01, 0x00};
    EXPECT_EQ(checksum(body0, sizeof(body0)), 0x04);

    // From the R200 document: AA 00 07 00 03 04 02 05 [15] DD
    // Sum of Type..Parameter = 00+07+00+03+04+02+05 = 0x15
    const uint8_t body1[] = {0x00, 0x07, 0x00, 0x03, 0x04, 0x02, 0x05};
    EXPECT_EQ(checksum(body1, sizeof(body1)), 0x15);

    // Wrap-around (only the least significant byte is retained)
    const uint8_t body2[] = {0xFF, 0xFF, 0x03};
    EXPECT_EQ(checksum(body2, sizeof(body2)), 0x01);

    // Empty
    EXPECT_EQ(checksum(nullptr, 0), 0x00);
}
