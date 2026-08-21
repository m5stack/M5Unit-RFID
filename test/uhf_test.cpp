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
#include <uhf/uhf.hpp>
#include <cstdint>
#include <vector>
#include <cstring>

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

TEST(UHF, BuildFrame)
{
    std::vector<uint8_t> buf{};

    // Get module information (hardware version): BB 00 03 00 01 00 04 7E
    const uint8_t param0[] = {0x00};
    ASSERT_TRUE(build_frame(buf, 0x00, 0x03, param0, sizeof(param0)));
    const uint8_t expect0[] = {0xBB, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0x7E};
    ASSERT_EQ(buf.size(), sizeof(expect0));
    EXPECT_EQ(0, memcmp(buf.data(), expect0, sizeof(expect0)));

    // Stop multiple polling: BB 00 28 00 00 28 7E (no parameter)
    ASSERT_TRUE(build_frame(buf, 0x00, 0x28, nullptr, 0));
    const uint8_t expect1[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};
    ASSERT_EQ(buf.size(), sizeof(expect1));
    EXPECT_EQ(0, memcmp(buf.data(), expect1, sizeof(expect1)));

    // Multiple polling 10000 times: BB 00 27 00 03 22 27 10 83 7E
    const uint8_t param2[] = {0x22, 0x27, 0x10};
    ASSERT_TRUE(build_frame(buf, 0x00, 0x27, param2, sizeof(param2)));
    const uint8_t expect2[] = {0xBB, 0x00, 0x27, 0x00, 0x03, 0x22, 0x27, 0x10, 0x83, 0x7E};
    ASSERT_EQ(buf.size(), sizeof(expect2));
    EXPECT_EQ(0, memcmp(buf.data(), expect2, sizeof(expect2)));
}

TEST(UHF, ParseFrame)
{
    Frame f{};

    // Valid response: BB 01 07 00 01 00 09 7E (Set Operating Region succeeded)
    const uint8_t raw0[] = {0xBB, 0x01, 0x07, 0x00, 0x01, 0x00, 0x09, 0x7E};
    ASSERT_TRUE(parse_frame(f, raw0, sizeof(raw0)));
    EXPECT_EQ(f.type, 0x01);
    EXPECT_EQ(f.command, 0x07);
    ASSERT_EQ(f.parameter.size(), 1U);
    EXPECT_EQ(f.parameter[0], 0x00);

    // No parameter
    const uint8_t raw1[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};
    ASSERT_TRUE(parse_frame(f, raw1, sizeof(raw1)));
    EXPECT_EQ(f.command, 0x28);
    EXPECT_TRUE(f.parameter.empty());

    // Bad checksum (0x09 -> 0x0A)
    const uint8_t bad_sum[] = {0xBB, 0x01, 0x07, 0x00, 0x01, 0x00, 0x0A, 0x7E};
    EXPECT_FALSE(parse_frame(f, bad_sum, sizeof(bad_sum)));

    // Bad header
    const uint8_t bad_header[] = {0xAA, 0x01, 0x07, 0x00, 0x01, 0x00, 0x09, 0x7E};
    EXPECT_FALSE(parse_frame(f, bad_header, sizeof(bad_header)));

    // Bad end
    const uint8_t bad_end[] = {0xBB, 0x01, 0x07, 0x00, 0x01, 0x00, 0x09, 0xDD};
    EXPECT_FALSE(parse_frame(f, bad_end, sizeof(bad_end)));

    // Truncated (shorter than the declared parameter length)
    const uint8_t truncated[] = {0xBB, 0x01, 0x07, 0x00, 0x05, 0x00, 0x09, 0x7E};
    EXPECT_FALSE(parse_frame(f, truncated, sizeof(truncated)));

    // Shorter than the minimum frame (7 bytes)
    const uint8_t too_short[] = {0xBB, 0x01, 0x07, 0x00, 0x00, 0x08};
    EXPECT_FALSE(parse_frame(f, too_short, sizeof(too_short)));
}

TEST(UHF, ParseFrameR200Delimiter)
{
    Frame f{};
    // R200 uses 0xAA / 0xDD but the structure is identical
    const uint8_t raw[] = {0xAA, 0x00, 0x07, 0x00, 0x03, 0x04, 0x02, 0x05, 0x15, 0xDD};
    ASSERT_TRUE(parse_frame(f, raw, sizeof(raw), 0xAA, 0xDD));
    EXPECT_EQ(f.type, 0x00);
    EXPECT_EQ(f.command, 0x07);
    ASSERT_EQ(f.parameter.size(), 3U);
    EXPECT_EQ(f.parameter[0], 0x04);
    EXPECT_EQ(f.parameter[2], 0x05);
}

TEST(UHF, ParseTagNotification)
{
    m5::uhf::Tag tag{};

    // From the protocol document (96-bit EPC), PL = 0x0011 = 17 = 1 + 2 + 12 + 2
    const uint8_t p96[] = {0xC9,                                                              // RSSI
                           0x34, 0x00,                                                        // PC
                           0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70,  // EPC
                           0x3A, 0x76};                                                       // CRC
    ASSERT_TRUE(parse_tag_notification(tag, p96, sizeof(p96)));
    EXPECT_EQ(tag.rssi, -55);  // 0xC9 as a signed byte
    EXPECT_EQ(tag.pc, 0x3400);
    ASSERT_EQ(tag.epc.size(), 12U);
    EXPECT_EQ(tag.epc[0], 0x30);
    EXPECT_EQ(tag.epc[11], 0x70);
    EXPECT_EQ(tag.crc, 0x3A76);

    // 32-bit EPC (4 bytes)
    const uint8_t p32[] = {0xB0, 0x18, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0x12, 0x34};
    ASSERT_TRUE(parse_tag_notification(tag, p32, sizeof(p32)));
    EXPECT_EQ(tag.rssi, -80);
    ASSERT_EQ(tag.epc.size(), 4U);
    EXPECT_EQ(tag.epc[3], 0xDD);

    // Positive RSSI
    const uint8_t ppos[] = {0x0A, 0x34, 0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00};
    ASSERT_TRUE(parse_tag_notification(tag, ppos, sizeof(ppos)));
    EXPECT_EQ(tag.rssi, 10);

    // Too short (5 bytes means an empty EPC, which is not a valid tag)
    const uint8_t pshort[] = {0xC9, 0x34, 0x00, 0x3A, 0x76};
    EXPECT_FALSE(parse_tag_notification(tag, pshort, sizeof(pshort)));

    // Shorter than the fixed part
    const uint8_t ptiny[] = {0xC9, 0x34};
    EXPECT_FALSE(parse_tag_notification(tag, ptiny, sizeof(ptiny)));

    EXPECT_FALSE(parse_tag_notification(tag, nullptr, 0));
}
