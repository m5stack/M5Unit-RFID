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
    if (!(build_frame(buf, 0x00, 0x03, param0, sizeof(param0)))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect0[] = {0xBB, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect0));
    EXPECT_EQ(0, memcmp(buf.data(), expect0, sizeof(expect0)));

    // Stop multiple polling: BB 00 28 00 00 28 7E (no parameter)
    if (!(build_frame(buf, 0x00, 0x28, nullptr, 0))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect1[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect1));
    EXPECT_EQ(0, memcmp(buf.data(), expect1, sizeof(expect1)));

    // Multiple polling 10000 times: BB 00 27 00 03 22 27 10 83 7E
    const uint8_t param2[] = {0x22, 0x27, 0x10};
    if (!(build_frame(buf, 0x00, 0x27, param2, sizeof(param2)))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect2[] = {0xBB, 0x00, 0x27, 0x00, 0x03, 0x22, 0x27, 0x10, 0x83, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect2));
    EXPECT_EQ(0, memcmp(buf.data(), expect2, sizeof(expect2)));
}

TEST(UHF, ParseFrame)
{
    Frame f{};

    // Valid response: BB 01 07 00 01 00 09 7E (Set Operating Region succeeded)
    const uint8_t raw0[] = {0xBB, 0x01, 0x07, 0x00, 0x01, 0x00, 0x09, 0x7E};
    if (!(parse_frame(f, raw0, sizeof(raw0)))) {
        EXPECT_TRUE(false);
        return;
    }
    EXPECT_EQ(f.type, 0x01);
    EXPECT_EQ(f.command, 0x07);
    EXPECT_EQ(f.parameter.size(), 1U);
    EXPECT_EQ(f.parameter[0], 0x00);

    // No parameter
    const uint8_t raw1[] = {0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E};
    if (!(parse_frame(f, raw1, sizeof(raw1)))) {
        EXPECT_TRUE(false);
        return;
    }
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
    if (!(parse_frame(f, raw, sizeof(raw), 0xAA, 0xDD))) {
        EXPECT_TRUE(false);
        return;
    }
    EXPECT_EQ(f.type, 0x00);
    EXPECT_EQ(f.command, 0x07);
    EXPECT_EQ(f.parameter.size(), 3U);
    EXPECT_EQ(f.parameter[0], 0x04);
    EXPECT_EQ(f.parameter[2], 0x05);
}

TEST(UHF, ParseTagNotification)
{
    m5::uhf::Tag tag{};

    // From the protocol document (96-bit EPC), PL = 0x0011 = 17 = 1 + 2 + 12 + 2
    const uint8_t p96[] = {0xC9,                                                                    // RSSI
                           0x34, 0x00,                                                              // PC
                           0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70,  // EPC
                           0x3A, 0x76};                                                             // CRC
    if (!(parse_tag_notification(tag, p96, sizeof(p96)))) {
        EXPECT_TRUE(false);
        return;
    }
    EXPECT_EQ(tag.rssi, -55);  // 0xC9 as a signed byte
    EXPECT_EQ(tag.pc, 0x3400);
    EXPECT_EQ(tag.epc.size, 12U);
    EXPECT_EQ(tag.epc[0], 0x30);
    EXPECT_EQ(tag.epc[11], 0x70);
    EXPECT_EQ(tag.crc, 0x3A76);

    // 32-bit EPC (4 bytes)
    const uint8_t p32[] = {0xB0, 0x18, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0x12, 0x34};
    if (!(parse_tag_notification(tag, p32, sizeof(p32)))) {
        EXPECT_TRUE(false);
        return;
    }
    EXPECT_EQ(tag.rssi, -80);
    EXPECT_EQ(tag.epc.size, 4U);
    EXPECT_EQ(tag.epc[3], 0xDD);

    // Positive RSSI
    const uint8_t ppos[] = {0x0A, 0x34, 0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00};
    if (!(parse_tag_notification(tag, ppos, sizeof(ppos)))) {
        EXPECT_TRUE(false);
        return;
    }
    EXPECT_EQ(tag.rssi, 10);

    // Too short (5 bytes means an empty EPC, which is not a valid tag)
    const uint8_t pshort[] = {0xC9, 0x34, 0x00, 0x3A, 0x76};
    EXPECT_FALSE(parse_tag_notification(tag, pshort, sizeof(pshort)));

    // Shorter than the fixed part
    const uint8_t ptiny[] = {0xC9, 0x34};
    EXPECT_FALSE(parse_tag_notification(tag, ptiny, sizeof(ptiny)));

    EXPECT_FALSE(parse_tag_notification(tag, nullptr, 0));
}

TEST(UHF, ErrorClassification)
{
    // 0xFF is the command code used by every failure notification
    EXPECT_TRUE(is_error_frame(0xFF));
    EXPECT_FALSE(is_error_frame(0x27));
    EXPECT_FALSE(is_error_frame(0x22));

    // 0x15 during polling only means "no tag this round"
    EXPECT_TRUE(is_no_tag(0x15));
    EXPECT_FALSE(is_no_tag(0x16));  // Access Fail
    EXPECT_FALSE(is_no_tag(0x17));  // Command Error
    EXPECT_FALSE(is_no_tag(0x20));  // FHSS Fail
    EXPECT_FALSE(is_no_tag(0x09));  // Read Fail
}

namespace {
m5::uhf::Tag make_tag(const std::vector<uint8_t>& epc, const int8_t rssi)
{
    m5::uhf::Tag t{};
    t.pc   = 0x3400;
    t.crc  = 0x1234;
    t.rssi = rssi;
    t.epc.assign(epc.data(), epc.size());
    return t;
}
}  // namespace

TEST(UHF, DeduplicateByEPC)
{
    std::vector<m5::uhf::Tag> dst{};

    // The first tag is appended
    EXPECT_TRUE(m5::uhf::append_unique(dst, make_tag({0x01, 0x02, 0x03, 0x04}, -50)));
    EXPECT_EQ(dst.size(), 1U);

    // The same EPC is rejected even when the RSSI differs
    EXPECT_FALSE(m5::uhf::append_unique(dst, make_tag({0x01, 0x02, 0x03, 0x04}, -60)));
    EXPECT_EQ(dst.size(), 1U);
    EXPECT_EQ(dst[0].rssi, -50);  // The first observation is kept

    // A different EPC is appended
    EXPECT_TRUE(m5::uhf::append_unique(dst, make_tag({0x01, 0x02, 0x03, 0x05}, -55)));
    EXPECT_EQ(dst.size(), 2U);

    // A prefix of an existing EPC is a different tag (length matters)
    EXPECT_TRUE(m5::uhf::append_unique(dst, make_tag({0x01, 0x02, 0x03}, -55)));
    EXPECT_EQ(dst.size(), 3U);

    // An empty EPC is never appended
    EXPECT_FALSE(m5::uhf::append_unique(dst, make_tag({}, -55)));
    EXPECT_EQ(dst.size(), 3U);
}

TEST(UHF, Gen2CRC16)
{
    // From the protocol document: PC=0x3400, EPC=30 75 ... 70, CRC=0x3A76
    const uint8_t pc_epc[] = {0x34, 0x00, 0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70};
    EXPECT_EQ(gen2_crc16(pc_epc, sizeof(pc_epc)), 0x3A76);

    // A single flipped bit must change the result
    uint8_t broken[sizeof(pc_epc)]{};
    memcpy(broken, pc_epc, sizeof(pc_epc));
    broken[5] ^= 0x01;
    EXPECT_NE(gen2_crc16(broken, sizeof(broken)), 0x3A76);
}

TEST(UHF, VerifyTagCRC)
{
    m5::uhf::Tag tag{};
    const uint8_t param[] = {0xC9,                                                                    // RSSI
                             0x34, 0x00,                                                              // PC
                             0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70,  // EPC
                             0x3A, 0x76};                                                             // CRC
    if (!(parse_tag_notification(tag, param, sizeof(param)))) {
        EXPECT_TRUE(false);
        return;
    }
    EXPECT_TRUE(verify_tag_crc(tag));

    // Corrupting the EPC must be detected
    m5::uhf::Tag bad = tag;
    bad.epc[0] ^= 0x01;
    EXPECT_FALSE(verify_tag_crc(bad));

    // Corrupting the PC must be detected
    bad = tag;
    bad.pc ^= 0x0001;
    EXPECT_FALSE(verify_tag_crc(bad));

    // An empty EPC has nothing to verify
    m5::uhf::Tag empty{};
    EXPECT_FALSE(verify_tag_crc(empty));
}

TEST(UHF, DecodeProtocolControl)
{
    // From the protocol document: PC=0x3400 accompanies a 12-byte (6-word) EPC
    EXPECT_EQ(m5::uhf::pcEPCLengthWords(0x3400), 6);
    EXPECT_TRUE(m5::uhf::pcUserMemoryIndicator(0x3400));
    EXPECT_FALSE(m5::uhf::pcXPCIndicator(0x3400));
    EXPECT_EQ(m5::uhf::pcNumberingSystemIdentifier(0x3400), 0x000);

    // 0x3000 is the common PC of a 96-bit EPC without user memory data
    EXPECT_EQ(m5::uhf::pcEPCLengthWords(0x3000), 6);
    EXPECT_FALSE(m5::uhf::pcUserMemoryIndicator(0x3000));

    // 128-bit EPC is 8 words
    EXPECT_EQ(m5::uhf::pcEPCLengthWords(0x4000), 8);

    // XPC indicator and a non-EPCglobal numbering system
    EXPECT_TRUE(m5::uhf::pcXPCIndicator(0x3200));
    EXPECT_EQ(m5::uhf::pcNumberingSystemIdentifier(0x3055), 0x055);
}

TEST(UHF, BuildSelectParameter)
{
    // Datasheet 2.5.1: SelParam=0x01 (Target 000 / Action 000 / MemBank EPC), Ptr=0x20 bits,
    // MaskLength=0x60 (96 bits), Truncate off, Mask = the tag's EPC
    const uint8_t mask[] = {0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70};
    std::vector<uint8_t> param{};
    const uint8_t sel = select_parameter_byte(0, 0, MEMBANK_EPC);
    EXPECT_EQ(sel, 0x01);
    if (!build_select_parameter(param, sel, 0x00000020, 0x60, SELECT_TRUNCATE_OFF, mask, sizeof(mask))) {
        EXPECT_TRUE(false);
        return;
    }

    std::vector<uint8_t> buf{};
    if (!build_frame(buf, 0x00, 0x0C, param.data(), static_cast<uint16_t>(param.size()))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect[] = {0xBB, 0x00, 0x0C, 0x00, 0x13, 0x01, 0x00, 0x00, 0x00, 0x20, 0x60, 0x00, 0x30,
                              0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70, 0xAD, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect));
    EXPECT_EQ(0, memcmp(buf.data(), expect, sizeof(expect)));

    // Reserved memory cannot be selected on
    EXPECT_FALSE(build_select_parameter(param, select_parameter_byte(0, 0, MEMBANK_RESERVED), 0x20, 0x60,
                                        SELECT_TRUNCATE_OFF, mask, sizeof(mask)));
    // A mask shorter than the length claims
    EXPECT_FALSE(build_select_parameter(param, sel, 0x20, 0x60, SELECT_TRUNCATE_OFF, mask, 4));
    // Empty mask
    EXPECT_FALSE(build_select_parameter(param, sel, 0x20, 0, SELECT_TRUNCATE_OFF, mask, sizeof(mask)));
}

TEST(UHF, BuildReadTagMemory)
{
    // Datasheet 2.8.1: AccessPassword=0x0000FFFF, MemBank=User, SA=0, DL=2
    std::vector<uint8_t> param{};
    if (!build_read_tag_memory(param, 0x0000FFFF, MEMBANK_USER, 0x0000, 0x0002)) {
        EXPECT_TRUE(false);
        return;
    }
    std::vector<uint8_t> buf{};
    if (!build_frame(buf, 0x00, 0x39, param.data(), static_cast<uint16_t>(param.size()))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect[] = {0xBB, 0x00, 0x39, 0x00, 0x09, 0x00, 0x00, 0xFF,
                              0xFF, 0x03, 0x00, 0x00, 0x00, 0x02, 0x45, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect));
    EXPECT_EQ(0, memcmp(buf.data(), expect, sizeof(expect)));

    // Reading zero words would mean "to the end of the bank", which is unverified on this module
    EXPECT_FALSE(build_read_tag_memory(param, 0, MEMBANK_USER, 0, 0));
}

TEST(UHF, BuildWriteTagMemory)
{
    // Datasheet 2.9.1: same addressing as the read plus DT = 12345678
    const uint8_t data[] = {0x12, 0x34, 0x56, 0x78};
    std::vector<uint8_t> param{};
    if (!build_write_tag_memory(param, 0x0000FFFF, MEMBANK_USER, 0x0000, data, sizeof(data))) {
        EXPECT_TRUE(false);
        return;
    }
    std::vector<uint8_t> buf{};
    if (!build_frame(buf, 0x00, 0x49, param.data(), static_cast<uint16_t>(param.size()))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect[] = {0xBB, 0x00, 0x49, 0x00, 0x0D, 0x00, 0x00, 0xFF, 0xFF, 0x03,
                              0x00, 0x00, 0x00, 0x02, 0x12, 0x34, 0x56, 0x78, 0x6D, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect));
    EXPECT_EQ(0, memcmp(buf.data(), expect, sizeof(expect)));

    // An odd byte count cannot be expressed in words
    const uint8_t odd[] = {0x12, 0x34, 0x56};
    EXPECT_FALSE(build_write_tag_memory(param, 0, MEMBANK_USER, 0, odd, sizeof(odd)));
    // The module writes at most 32 words
    const std::vector<uint8_t> too_long(WRITE_MAX_WORDS * 2 + 2, 0xAA);
    EXPECT_FALSE(build_write_tag_memory(param, 0, MEMBANK_USER, 0, too_long.data(), too_long.size()));
    // Exactly 32 words is fine
    const std::vector<uint8_t> at_limit(WRITE_MAX_WORDS * 2, 0xAA);
    EXPECT_TRUE(build_write_tag_memory(param, 0, MEMBANK_USER, 0, at_limit.data(), at_limit.size()));
}

TEST(UHF, BuildLockPayload)
{
    using m5::uhf::LockAction;
    using m5::uhf::LockSetting;
    using m5::uhf::LockTarget;

    // Datasheet 2.10.1 locks the access password and expects LD = 0x020080. Only the pwd-write
    // bit is masked in, leaving the permalock bit of that field untouched
    const LockSetting lock_access[] = {{LockTarget::AccessPassword, LockAction::Lock}};
    EXPECT_EQ(m5::uhf::buildLockPayload(lock_access, 1), 0x00020080U);

    std::vector<uint8_t> param{};
    if (!build_lock_tag(param, 0x0000FFFF, 0x00020080U)) {
        EXPECT_TRUE(false);
        return;
    }
    std::vector<uint8_t> buf{};
    if (!build_frame(buf, 0x00, 0x82, param.data(), static_cast<uint16_t>(param.size()))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect[] = {0xBB, 0x00, 0x82, 0x00, 0x07, 0x00, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x80, 0x09, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect));
    EXPECT_EQ(0, memcmp(buf.data(), expect, sizeof(expect)));

    // Kill password sits in the highest slot, User in the lowest
    const LockSetting lock_kill[] = {{LockTarget::KillPassword, LockAction::Lock}};
    EXPECT_EQ(m5::uhf::buildLockPayload(lock_kill, 1), 0x00080200U);
    const LockSetting lock_user[] = {{LockTarget::User, LockAction::Lock}};
    EXPECT_EQ(m5::uhf::buildLockPayload(lock_user, 1), 0x00000802U);

    // A permanent action masks in the permalock bit as well
    const LockSetting perma_user[] = {{LockTarget::User, LockAction::PermanentLock}};
    EXPECT_EQ(m5::uhf::buildLockPayload(perma_user, 1), 0x00000C03U);

    // Targets left out keep their current state, so their mask bits stay zero
    const LockSetting two[] = {{LockTarget::Epc, LockAction::Lock}, {LockTarget::Tid, LockAction::Lock}};
    EXPECT_EQ(m5::uhf::buildLockPayload(two, 2), 0x0000A028U);

    // Nothing asked for, nothing changed
    EXPECT_EQ(m5::uhf::buildLockPayload(nullptr, 0), 0x00000000U);

    EXPECT_TRUE(m5::uhf::isPermanent(LockAction::PermanentOpen));
    EXPECT_TRUE(m5::uhf::isPermanent(LockAction::PermanentLock));
    EXPECT_FALSE(m5::uhf::isPermanent(LockAction::Open));
    EXPECT_FALSE(m5::uhf::isPermanent(LockAction::Lock));
}

TEST(UHF, BuildKillTag)
{
    // Datasheet 2.11.1: KillPassword = 0x0000FFFF
    std::vector<uint8_t> param{};
    if (!build_kill_tag(param, 0x0000FFFF)) {
        EXPECT_TRUE(false);
        return;
    }
    std::vector<uint8_t> buf{};
    if (!build_frame(buf, 0x00, 0x65, param.data(), static_cast<uint16_t>(param.size()))) {
        EXPECT_TRUE(false);
        return;
    }
    const uint8_t expect[] = {0xBB, 0x00, 0x65, 0x00, 0x04, 0x00, 0x00, 0xFF, 0xFF, 0x67, 0x7E};
    EXPECT_EQ(buf.size(), sizeof(expect));
    EXPECT_EQ(0, memcmp(buf.data(), expect, sizeof(expect)));

    // A tag whose kill password is zero refuses to be killed, so the frame is never built
    EXPECT_FALSE(build_kill_tag(param, 0));
}
