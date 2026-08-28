/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UHF-RFID
*/
#include <gtest/gtest.h>
#include <unit/m100_frame.hpp>
#include <uhf/uhf.hpp>
#include <cstdint>
#include <vector>
#include <cstring>

using namespace m5::unit::m100;

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
    EXPECT_EQ(m5::uhf::gen2_crc16(pc_epc, sizeof(pc_epc)), 0x3A76);

    // A single flipped bit must change the result
    uint8_t broken[sizeof(pc_epc)]{};
    memcpy(broken, pc_epc, sizeof(pc_epc));
    broken[5] ^= 0x01;
    EXPECT_NE(m5::uhf::gen2_crc16(broken, sizeof(broken)), 0x3A76);
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
    EXPECT_TRUE(m5::uhf::verify_tag_crc(tag));

    // Corrupting the EPC must be detected
    m5::uhf::Tag bad = tag;
    bad.epc[0] ^= 0x01;
    EXPECT_FALSE(m5::uhf::verify_tag_crc(bad));

    // Corrupting the PC must be detected
    bad = tag;
    bad.pc ^= 0x0001;
    EXPECT_FALSE(m5::uhf::verify_tag_crc(bad));

    // An empty EPC has nothing to verify
    m5::uhf::Tag empty{};
    EXPECT_FALSE(m5::uhf::verify_tag_crc(empty));
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

    // Bit 10h of the PC is the most significant one of the word, so the Toggle at 17h is the
    // eighth down. Every tag this ran against reports it clear, and a tag asserting it is
    // saying the EPC bank holds an ISO UII instead of an EPC
    EXPECT_FALSE(m5::uhf::pcToggle(0x3000));
    EXPECT_FALSE(m5::uhf::pcToggle(0x3400));
    EXPECT_TRUE(m5::uhf::pcToggle(0x3100));

    // The three bits below the length are read one at a time, and setting one does not disturb
    // its neighbours
    EXPECT_TRUE(m5::uhf::pcUserMemoryIndicator(0x3400) && !m5::uhf::pcXPCIndicator(0x3400) &&
                !m5::uhf::pcToggle(0x3400));
    EXPECT_TRUE(!m5::uhf::pcUserMemoryIndicator(0x3200) && m5::uhf::pcXPCIndicator(0x3200) &&
                !m5::uhf::pcToggle(0x3200));
    EXPECT_TRUE(!m5::uhf::pcUserMemoryIndicator(0x3100) && !m5::uhf::pcXPCIndicator(0x3100) &&
                m5::uhf::pcToggle(0x3100));
    EXPECT_TRUE(m5::uhf::pcUserMemoryIndicator(0x3700) && m5::uhf::pcXPCIndicator(0x3700) && m5::uhf::pcToggle(0x3700));

    // The length is untouched whichever of them is set
    EXPECT_EQ(m5::uhf::pcEPCLengthWords(0x3700), 6);
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

TEST(UHF, XtidSerialBits)
{
    // TDS 16.2.2: bits 15-13 hold the serialisation code
    EXPECT_EQ(m5::uhf::xtidSerialBits(0x0000), 0U);  // no serial
    // TDS: "If the tag only contained a 48 bit serial number the XTID header would be 0010000000000000"
    EXPECT_EQ(m5::uhf::xtidSerialBits(0x2000), 48U);
    // TDS: a fully populated XTID header, still a 48-bit serial plus every optional segment
    EXPECT_EQ(m5::uhf::xtidSerialBits(0x3C00), 48U);
    // 48 + (value - 1) * 16 for the rest
    EXPECT_EQ(m5::uhf::xtidSerialBits(0x4000), 64U);
    EXPECT_EQ(m5::uhf::xtidSerialBits(0x6000), 80U);
    EXPECT_EQ(m5::uhf::xtidSerialBits(0x8000), 96U);
    EXPECT_EQ(m5::uhf::xtidSerialBits(0xE000), 144U);  // the longest the encoding allows
}

TEST(UHF, DecodeTid)
{
    m5::uhf::Tag tag{};

    // UCODE G2iM datasheet Rev 3.7: the TID starts E200680Ah
    const uint8_t g2im[] = {0xE2, 0x00, 0x68, 0x0A};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, g2im, sizeof(g2im)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::NXP);
    EXPECT_EQ(tag.mdid, 0x006);
    EXPECT_EQ(tag.model_number, 0x80A);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::NxpUcodeG2iM);
    EXPECT_EQ(tag.chipAsString(), "NXP UCODE G2iM");
    EXPECT_FALSE(tag.has_xtid);
    EXPECT_FALSE(tag.supports_security);
    EXPECT_FALSE(tag.supports_file);
    EXPECT_EQ(tag.serial_bits, 0U);

    // UCODE G2iM+ differs only in the model number
    const uint8_t g2imp[] = {0xE2, 0x00, 0x68, 0x0B};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, g2imp, sizeof(g2imp)));
    EXPECT_EQ(tag.model_number, 0x80B);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::NxpUcodeG2iMPlus);

    // An Impinj tag carrying an XTID with a 48-bit serial
    const uint8_t impinj[] = {0xE2, 0x80, 0x11, 0x05, 0x20, 0x00};
    tag                    = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, impinj, sizeof(impinj)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::Impinj);
    EXPECT_EQ(tag.mdid, 0x001);
    EXPECT_EQ(tag.model_number, 0x105);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::ImpinjMonza4QT);
    EXPECT_TRUE(tag.has_xtid);
    EXPECT_EQ(tag.serial_bits, 48U);

    // The security and file indicators sit next to the XTID indicator
    const uint8_t flags[] = {0xE2, 0xE0, 0x31, 0x23, 0x00, 0x00};
    tag                   = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, flags, sizeof(flags)));
    EXPECT_TRUE(tag.has_xtid);
    EXPECT_TRUE(tag.supports_security);
    EXPECT_TRUE(tag.supports_file);
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::Alien);
    EXPECT_EQ(tag.model_number, 0x123);
    EXPECT_EQ(tag.serial_bits, 0U);  // the header says no serialisation

    // Impinj Monza 4QT, from the Monza 4 datasheet Rev 8.0
    const uint8_t monza4qt[] = {0xE2, 0x80, 0x11, 0x05, 0x20, 0x00};
    tag                      = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, monza4qt, sizeof(monza4qt)));
    EXPECT_EQ(tag.chip, m5::uhf::Chip::ImpinjMonza4QT);
    EXPECT_EQ(tag.chipAsString(), "Impinj Monza 4QT");

    // NXP UCODE 8, the chip on the tag this library was first tried against
    const uint8_t ucode8[] = {0xE2, 0x80, 0x68, 0x94};
    tag                    = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, ucode8, sizeof(ucode8)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::NXP);
    EXPECT_EQ(tag.model_number, 0x894);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::NxpUcode8);

    // NXP moved UCODE 9 from 995h to 915h without renaming the part, so both mean UCODE 9
    const uint8_t ucode9_old[] = {0xE2, 0x80, 0x69, 0x95};
    tag                        = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, ucode9_old, sizeof(ucode9_old)));
    EXPECT_EQ(tag.chip, m5::uhf::Chip::NxpUcode9);
    const uint8_t ucode9_new[] = {0xE2, 0x80, 0x69, 0x15};
    tag                        = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, ucode9_new, sizeof(ucode9_new)));
    EXPECT_EQ(tag.chip, m5::uhf::Chip::NxpUcode9);

    // Alien Higgs-3
    const uint8_t higgs3[] = {0xE2, 0x00, 0x34, 0x12};
    tag                    = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, higgs3, sizeof(higgs3)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::Alien);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::AlienHiggs3);
    EXPECT_FALSE(tag.has_xtid);

    // The same designer with a model number nobody has published stays Unknown
    const uint8_t alien_other[] = {0xE2, 0x00, 0x3F, 0xFF};
    tag                         = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, alien_other, sizeof(alien_other)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::Alien);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::Unknown);

    // A designer we do not list still reports its raw identifier and model number, so an
    // unsupported tag can be described well enough to be added to the table later
    const uint8_t unlisted[] = {0xE2, 0x01, 0x23, 0x45, 0x00, 0x00};
    tag                      = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, unlisted, sizeof(unlisted)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::Unknown);
    EXPECT_EQ(tag.mdid, 0x012);
    EXPECT_EQ(tag.model_number, 0x345);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::Unknown);

    // Every identifier the enum lists resolves back to itself
    EXPECT_EQ(m5::uhf::resolveVendor(0x001), m5::uhf::Vendor::Impinj);
    EXPECT_EQ(m5::uhf::resolveVendor(0x002), m5::uhf::Vendor::TexasInstruments);
    EXPECT_EQ(m5::uhf::resolveVendor(0x003), m5::uhf::Vendor::Alien);
    EXPECT_EQ(m5::uhf::resolveVendor(0x005), m5::uhf::Vendor::Atmel);
    EXPECT_EQ(m5::uhf::resolveVendor(0x006), m5::uhf::Vendor::NXP);
    EXPECT_EQ(m5::uhf::resolveVendor(0x007), m5::uhf::Vendor::STMicroelectronics);
    EXPECT_EQ(m5::uhf::resolveVendor(0x008), m5::uhf::Vendor::EPMicroelectronics);
    EXPECT_EQ(m5::uhf::resolveVendor(0x00B), m5::uhf::Vendor::EMMicroelectronic);
    EXPECT_EQ(m5::uhf::resolveVendor(0x00F), m5::uhf::Vendor::Quanray);
    EXPECT_EQ(m5::uhf::resolveVendor(0x010), m5::uhf::Vendor::Fujitsu);
    EXPECT_EQ(m5::uhf::resolveVendor(0x01B), m5::uhf::Vendor::Nationz);
    EXPECT_EQ(m5::uhf::resolveVendor(0x01C), m5::uhf::Vendor::Invengo);
    EXPECT_EQ(m5::uhf::resolveVendor(0x024), m5::uhf::Vendor::RFMicron);
    EXPECT_EQ(m5::uhf::resolveVendor(0x027), m5::uhf::Vendor::FudanMicroelectronics);
    EXPECT_EQ(m5::uhf::resolveVendor(0x02F), m5::uhf::Vendor::AMS);
    EXPECT_EQ(m5::uhf::resolveVendor(0x032), m5::uhf::Vendor::HuadaSemiconductor);
    EXPECT_EQ(m5::uhf::resolveVendor(0x034), m5::uhf::Vendor::Mikron);
    // A registered designer we do not list, an identifier nobody holds, and the unassigned zero
    EXPECT_EQ(m5::uhf::resolveVendor(0x012), m5::uhf::Vendor::Unknown);
    EXPECT_EQ(m5::uhf::resolveVendor(0x1FF), m5::uhf::Vendor::Unknown);
    EXPECT_EQ(m5::uhf::resolveVendor(0x000), m5::uhf::Vendor::Unknown);

    // A designer that is named but whose model numbers are not listed still reaches Chip::Unknown
    const uint8_t em[] = {0xE2, 0x00, 0xB1, 0x23, 0x00, 0x00};
    tag                = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, em, sizeof(em)));
    EXPECT_EQ(tag.vendor, m5::uhf::Vendor::EMMicroelectronic);
    EXPECT_EQ(tag.mdid, 0x00B);
    EXPECT_EQ(tag.model_number, 0x123);
    EXPECT_EQ(tag.chip, m5::uhf::Chip::Unknown);
    EXPECT_STREQ(tag.vendorAsString().c_str(), "EM Microelectronic");
    EXPECT_STREQ(tag.chipAsString().c_str(), "Unknown");

    // Every named designer prints, and one we do not list falls back
    tag        = m5::uhf::Tag{};
    tag.vendor = m5::uhf::Vendor::Impinj;
    EXPECT_STREQ(tag.vendorAsString().c_str(), "Impinj");
    tag.vendor = m5::uhf::Vendor::FudanMicroelectronics;
    EXPECT_STREQ(tag.vendorAsString().c_str(), "Shanghai Fudan Microelectronics Group");
    tag.vendor = m5::uhf::Vendor::Unknown;
    EXPECT_STREQ(tag.vendorAsString().c_str(), "Unknown");

    // A class identifier other than E2h is not something this decoder understands
    const uint8_t e0[] = {0xE0, 0x00, 0x68, 0x0A};
    EXPECT_FALSE(m5::uhf::decodeTid(tag, e0, sizeof(e0)));
    // Too short to hold even the fixed part
    EXPECT_FALSE(m5::uhf::decodeTid(tag, g2im, 3));
    EXPECT_FALSE(m5::uhf::decodeTid(tag, nullptr, 4));
}

TEST(UHF, XtidTotalWords)
{
    // Two fixed words plus the header, and nothing else
    EXPECT_EQ(m5::uhf::xtidTotalWords(0x0000), 3U);
    // The 48-bit serial of the worked example in TDS Table 16-2
    EXPECT_EQ(m5::uhf::xtidTotalWords(0x2000), 6U);
    // Serialisation 111 is the longest the field can say: 144 bits, nine words
    EXPECT_EQ(m5::uhf::xtidTotalWords(0xE000), 12U);
    // The fully populated XTID of TDS Table 16-2: 48-bit serial and all four segments
    EXPECT_EQ(m5::uhf::xtidTotalWords(0x3E00), 6U + 1U + 4U + 2U + 1U);
    // The largest TID the standard can describe still fits in what identify() keeps
    EXPECT_EQ(m5::uhf::xtidTotalWords(0xFE00) * 2, m5::uhf::TID_MAX_BYTES);

    // Two chips whose datasheets lay out the whole TID, which is what pins the segment widths
    // to something other than the standard's own arithmetic. RAMXEED MB97R8110
    // (DS411-00007-3v1-E Table 4.2.2) reads 3C00h and has thirteen words of TID; the same
    // header is the worked example in TDS 2.3 16.2
    EXPECT_EQ(m5::uhf::xtidTotalWords(0x3C00), 13U);
    // RAMXEED MB97R8050 (DS411-00005-4v1-E) reads 3800h and has eleven
    EXPECT_EQ(m5::uhf::xtidTotalWords(0x3800), 11U);
}

TEST(UHF, DecodeXtidSegments)
{
    m5::uhf::Tag tag{};

    // A tag with a 48-bit serial and the Optional Command Support segment only. The header
    // therefore reads 0011 0000 0000 0000, and the segment sits right after the three serial
    // words. Max EPC Size counts words, so 0x0F means a 240-bit EPC, and bit 12 claims
    // BlockPermaLock support
    const uint8_t ocs_only[] = {
        0xE2, 0x80, 0x11, 0x05,              // Impinj Monza 4QT
        0x30, 0x00,                          // XTID header
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66,  // 48-bit serial
        0x10, 0x0F,                          // Optional Command Support
    };
    EXPECT_TRUE(m5::uhf::decodeTid(tag, ocs_only, sizeof(ocs_only)));
    EXPECT_TRUE(tag.has_xtid);
    EXPECT_EQ(tag.serial_bits, 48);
    // The segment says how large an EPC this tag takes, so that is what is reported rather than
    // the 128 bits the chip is otherwise known for
    EXPECT_EQ(tag.epc_max_bits, 15U * 16U);
    EXPECT_TRUE(tag.supports_block_permalock);
    // Nothing said how much user memory, so what the chip is known to hold stands in
    EXPECT_EQ(tag.user_memory_bits, 512U);

    // The same tag with the User Memory and BlockPermaLock segment as well. The lower address
    // carries the block size and the higher one the user memory size, which is the order that
    // reads backwards from the field numbering (TDS Table 16-2)
    const uint8_t with_user[] = {
        0xE2, 0x80, 0x11, 0x05, 0x34, 0x00,              // header: serial + Optional Command Support + User Memory
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x0F,  // Optional Command Support
        0x00, 0x04,                                      // BlockPermaLock block size: 4 words
        0x00, 0x2B,                                      // User memory size: 43 words
    };
    tag = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, with_user, sizeof(with_user)));
    EXPECT_EQ(tag.permalock_block_bits, 4U * 16U);
    EXPECT_EQ(tag.user_memory_bits, 43U * 16U);

    // The four-word BlockWrite segment in between has to be stepped over, or the two words
    // after it are read as the user memory sizes
    const uint8_t with_blockwrite[] = {
        0xE2, 0x80, 0x11, 0x05, 0x3C, 0x00,              // header: serial + all three of the segments above bit 9
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x10, 0x0F,  // Optional Command Support
        0xAA, 0xAA, 0xBB, 0xBB, 0xCC, 0xCC, 0xDD, 0xDD,  // BlockWrite and BlockErase
        0x00, 0x04,                                      // BlockPermaLock block size
        0x00, 0x2B,                                      // User memory size
    };
    tag = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, with_blockwrite, sizeof(with_blockwrite)));
    EXPECT_EQ(tag.permalock_block_bits, 4U * 16U);
    EXPECT_EQ(tag.user_memory_bits, 43U * 16U);

    // Reading only as far as the header still yields the chip and the serial length. Nothing is
    // invented out of the bytes that were never read: the sizes here are the ones the chip is
    // known for, not the ones the unread segments would have carried
    tag = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, with_blockwrite, 6));
    EXPECT_EQ(tag.chip, m5::uhf::Chip::ImpinjMonza4QT);
    EXPECT_EQ(tag.serial_bits, 48);
    EXPECT_EQ(tag.epc_max_bits, 128U);
    EXPECT_EQ(tag.user_memory_bits, 512U);

    // A tag with no XTID reports no serial and no sizes
    const uint8_t plain[] = {0xE2, 0x00, 0x34, 0x12, 0x30, 0x00};
    tag                   = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, plain, sizeof(plain)));
    EXPECT_FALSE(tag.has_xtid);
    EXPECT_EQ(tag.serial_bits, 0);
    EXPECT_EQ(tag.epc_max_bits, 0U);
}

TEST(UHF, ParseTagOperation)
{
    // The answer to a Read: the count covers PC and EPC, and the words read follow them
    const uint8_t read_answer[] = {
        0x0E,                                                                    // UL
        0x34, 0x00,                                                              // PC
        0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70,  // EPC
        0x12, 0x34, 0x56, 0x78,                                                  // data
    };
    m5::unit::m100::TagOperationResult r{};
    EXPECT_TRUE(m5::unit::m100::parse_tag_operation(r, read_answer, sizeof(read_answer)));
    EXPECT_EQ(r.pc, 0x3400);
    EXPECT_EQ(r.epc.size, 12);
    EXPECT_STREQ(r.epc.toString().c_str(), "30751FEB705C5904E3D50D70");
    EXPECT_EQ(r.data_len, 4U);
    EXPECT_EQ(r.data[0], 0x12);

    // The answer to a Write, Lock or Kill is the same shape with a single status byte
    const uint8_t write_answer[] = {
        0x0E, 0x34, 0x00, 0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70, 0x00,
    };
    r = m5::unit::m100::TagOperationResult{};
    EXPECT_TRUE(m5::unit::m100::parse_tag_operation(r, write_answer, sizeof(write_answer)));
    EXPECT_EQ(r.data_len, 1U);
    EXPECT_EQ(r.data[0], m5::unit::m100::TAG_OPERATION_SUCCESS);

    // A count that runs past the frame, a count too small to hold the PC, and nothing at all
    const uint8_t overrun[] = {0x40, 0x34, 0x00, 0x30};
    r                       = m5::unit::m100::TagOperationResult{};
    EXPECT_FALSE(m5::unit::m100::parse_tag_operation(r, overrun, sizeof(overrun)));
    const uint8_t too_small[] = {0x01, 0x34, 0x00, 0x30};
    EXPECT_FALSE(m5::unit::m100::parse_tag_operation(r, too_small, sizeof(too_small)));
    EXPECT_FALSE(m5::unit::m100::parse_tag_operation(r, nullptr, 16));
    EXPECT_FALSE(m5::unit::m100::parse_tag_operation(r, read_answer, 3));
}

TEST(UHF, ErrorDescription)
{
    // The high nibble says which command failed and the low one says why, so the same Gen2
    // code has to read the same whichever operation provoked it
    EXPECT_TRUE(m5::unit::m100::is_tag_error(m5::unit::m100::TAG_ERROR_READ | 0x04));
    EXPECT_TRUE(m5::unit::m100::is_tag_error(m5::unit::m100::TAG_ERROR_KILL));
    EXPECT_STREQ(m5::unit::m100::error_description(m5::unit::m100::TAG_ERROR_READ | 0x04),
                 m5::unit::m100::error_description(m5::unit::m100::TAG_ERROR_WRITE | 0x04));
    EXPECT_STREQ(m5::unit::m100::error_description(0xA3), "Tag: memory overrun");
    EXPECT_STREQ(m5::unit::m100::error_description(0xD0), "Tag: other error");

    // Module-level failures are not tag errors and keep their own meanings
    EXPECT_FALSE(m5::unit::m100::is_tag_error(0x15));
    EXPECT_FALSE(m5::unit::m100::is_tag_error(0x16));
    // The module's own wording hedges this one, so this does too
    EXPECT_STREQ(m5::unit::m100::error_description(0x16), "Access failed; the password may be wrong");

    // One wording covers all six of these in the module's documentation, and it names two
    // causes. Neither is claimed as the one that happened
    EXPECT_STREQ(m5::unit::m100::error_description(0x09), "Read failed: no answer, or a CRC error");
    EXPECT_STREQ(m5::unit::m100::error_description(0x10), "Write failed: no answer, or a CRC error");
    EXPECT_STREQ(m5::unit::m100::error_description(0x12), "Kill failed: no answer, or a CRC error");
    EXPECT_STREQ(m5::unit::m100::error_description(0x13), "Lock failed: no answer, or a CRC error");
    EXPECT_STREQ(m5::unit::m100::error_description(0x14), "BlockPermalock failed: no answer, or a CRC error");
    EXPECT_STREQ(m5::unit::m100::error_description(0x15), "No tag answered, or a CRC error");
}

TEST(UHF, ExtractFrame)
{
    const uint8_t header = jrd::FRAME_HEADER;
    const uint8_t end    = jrd::FRAME_END;

    // A response to "get module information": BB 01 03 00 01 00 05 7E
    std::vector<uint8_t> one{};
    const uint8_t param[] = {0x00};
    if (!build_frame(one, 0x01, 0x03, param, sizeof(param))) {
        EXPECT_TRUE(false);
        return;
    }

    Frame f{};
    size_t discarded{};

    // A whole frame and nothing else comes out, and the buffer is left empty
    {
        std::vector<uint8_t> buf = one;
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(discarded, 0u);
        EXPECT_EQ(f.command, 0x03);
        EXPECT_TRUE(buf.empty());
    }

    // Whatever sits in front of the header is not part of a frame and is counted as thrown away
    {
        std::vector<uint8_t> buf{0x11, 0x22, 0x33};
        buf.insert(buf.end(), one.begin(), one.end());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(discarded, 3u);
        EXPECT_TRUE(buf.empty());
    }

    // A frame that arrived in pieces is kept as it stands, and finished off once the rest turns
    // up. This is what a truncated read used to throw away
    {
        std::vector<uint8_t> buf(one.begin(), one.begin() + 5);
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::NeedMore);
        EXPECT_EQ(discarded, 0u);
        EXPECT_EQ(buf.size(), 5u);

        buf.insert(buf.end(), one.begin() + 5, one.end());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(f.command, 0x03);
        EXPECT_TRUE(buf.empty());
    }

    // Too few bytes to read a length out of are not enough to judge anything by
    {
        std::vector<uint8_t> buf{header, 0x01};
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::NeedMore);
        EXPECT_EQ(buf.size(), 2u);
    }

    // A length no frame can have says the header byte was data. One byte goes and the search
    // carries on, so the real frame behind it still comes out
    {
        std::vector<uint8_t> buf{header, 0x01, 0x03, 0xFF, 0xFF};
        buf.insert(buf.end(), one.begin(), one.end());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(f.command, 0x03);
        EXPECT_TRUE(buf.empty());
        EXPECT_EQ(discarded, 5u);
    }

    // A checksum that disagrees says the same thing, and is recovered from the same way
    {
        std::vector<uint8_t> bad = one;
        bad[bad.size() - 2] ^= 0xFF;
        std::vector<uint8_t> buf = bad;
        buf.insert(buf.end(), one.begin(), one.end());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(f.command, 0x03);
        EXPECT_TRUE(buf.empty());
        EXPECT_EQ(discarded, one.size());
    }

    // The header byte occurs inside the data a tag notification carries. Taking it for the start
    // of a frame has to cost one byte, not the frame that follows
    {
        std::vector<uint8_t> notification{};
        const uint8_t payload[] = {0xC7, 0x30, 0x00, header, header, 0x02, 0x03, 0x04};
        if (!build_frame(notification, 0x02, 0x22, payload, sizeof(payload))) {
            EXPECT_TRUE(false);
            return;
        }
        std::vector<uint8_t> buf = notification;
        buf.insert(buf.end(), one.begin(), one.end());

        // The notification is whole, so its payload is never mistaken for a frame start
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(discarded, 0u);
        EXPECT_EQ(f.command, 0x22);

        // What follows it is untouched
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(f.command, 0x03);
        EXPECT_TRUE(buf.empty());
    }

    // Losing the front of a notification leaves its payload looking like a frame start. The
    // frame behind it still has to come out
    {
        std::vector<uint8_t> notification{};
        const uint8_t payload[] = {0xC7, 0x30, 0x00, header, header, 0x02, 0x03, 0x04};
        if (!build_frame(notification, 0x02, 0x22, payload, sizeof(payload))) {
            EXPECT_TRUE(false);
            return;
        }
        // Start halfway in, at the first header byte of the payload
        std::vector<uint8_t> buf(notification.begin() + 8, notification.end());
        buf.insert(buf.end(), one.begin(), one.end());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(f.command, 0x03);
        EXPECT_TRUE(buf.empty());
        EXPECT_GT(discarded, 0u);
    }

    // Two frames back to back come out one at a time
    {
        std::vector<uint8_t> buf = one;
        buf.insert(buf.end(), one.begin(), one.end());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_EQ(buf.size(), one.size());
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::Ok);
        EXPECT_TRUE(buf.empty());
    }

    // Nothing at all is not something to wait for a frame in
    {
        std::vector<uint8_t> buf{};
        EXPECT_EQ(extract_frame(f, buf, discarded, header, end), FrameExtract::NeedMore);
        EXPECT_EQ(discarded, 0u);
    }
}

TEST(UHF, RouteFor)
{
    auto error_frame = [](const uint8_t code) {
        Frame f{};
        f.type      = 0x01;
        f.command   = COMMAND_ERROR;
        f.parameter = {code};
        return f;
    };
    auto plain_frame = [](const uint8_t command) {
        Frame f{};
        f.type    = 0x01;
        f.command = command;
        return f;
    };

    // A tag notification is a tag notification whether or not anything is being waited on
    EXPECT_EQ(route_for(plain_frame(COMMAND_SINGLE_POLLING), false, 0x00), FrameRoute::TagNotification);
    EXPECT_EQ(route_for(plain_frame(COMMAND_MULTIPLE_POLLING), true, COMMAND_READ_TAG_MEMORY),
              FrameRoute::TagNotification);

    // An empty round answers a polling command and is a leftover in front of anything else
    EXPECT_EQ(route_for(error_frame(0x15), true, COMMAND_SINGLE_POLLING), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x15), true, COMMAND_MULTIPLE_POLLING), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x15), true, COMMAND_READ_TAG_MEMORY), FrameRoute::Drop);
    EXPECT_EQ(route_for(error_frame(0x15), false, 0x00), FrameRoute::Drop);

    // An error names the operation it came from, so it can only answer that one
    EXPECT_EQ(route_for(error_frame(0xA3), true, COMMAND_READ_TAG_MEMORY), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0xA3), true, COMMAND_WRITE_TAG_MEMORY), FrameRoute::Drop);
    EXPECT_EQ(route_for(error_frame(0x09), true, COMMAND_READ_TAG_MEMORY), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x09), true, COMMAND_LOCK_TAG_MEMORY), FrameRoute::Drop);

    // A vendor command's failure names that command, so it cannot answer any of ours
    EXPECT_EQ(route_for(error_frame(0x2E), true, COMMAND_MONZA_QT), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x2E), true, COMMAND_READ_TAG_MEMORY), FrameRoute::Drop);
    EXPECT_EQ(route_for(error_frame(0x1A), true, COMMAND_NXP_CHANGE_CONFIG), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x1A), true, COMMAND_NXP_CHANGE_EAS), FrameRoute::Drop);
    // ReadProtect and Reset ReadProtect share one command code, so both failures answer it
    EXPECT_EQ(route_for(error_frame(0x2A), true, COMMAND_NXP_READ_PROTECT), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x2B), true, COMMAND_NXP_READ_PROTECT), FrameRoute::Response);

    // A code any command can provoke, and one this table does not name, answer whatever is sent
    EXPECT_EQ(route_for(error_frame(0x0E), true, COMMAND_WRITE_TAG_MEMORY), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x05), true, COMMAND_READ_TAG_MEMORY), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x17), true, COMMAND_LOCK_TAG_MEMORY), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x20), true, COMMAND_READ_TAG_MEMORY), FrameRoute::Response);
    EXPECT_EQ(route_for(error_frame(0x7F), true, COMMAND_KILL_TAG), FrameRoute::Response);

    // An error arriving with nothing pending is left over, whatever it says
    EXPECT_EQ(route_for(error_frame(0x09), false, 0x00), FrameRoute::Drop);
    EXPECT_EQ(route_for(error_frame(0x17), false, 0x00), FrameRoute::Drop);

    // An ordinary response is matched on the command code alone
    EXPECT_EQ(route_for(plain_frame(0x12), true, 0x12), FrameRoute::Response);
    EXPECT_EQ(route_for(plain_frame(0x12), true, 0x0B), FrameRoute::Unexpected);

    // The answer to a command that has already timed out cannot fill the slot the next one is
    // waiting on: nothing is pending between the timeout and the next send
    EXPECT_EQ(route_for(plain_frame(0x03), false, 0x00), FrameRoute::Unexpected);
}

TEST(UHF, BlockPermalockAnswers)
{
    // The tag that replied comes first everywhere: a byte counting the PC and EPC, then those
    // bytes. A lock puts one status byte after them, a read the range and that many words of mask
    const std::vector<uint8_t> tag{0x0E, 0x34, 0x00, 0xE2, 0x80, 0x11, 0x05, 0x20,
                                   0x00, 0x81, 0x1D, 0x67, 0xC1, 0x0B, 0x13};
    auto answer = [&tag](const std::vector<uint8_t>& tail) {
        std::vector<uint8_t> out{tag};
        out.insert(out.end(), tail.begin(), tail.end());
        return out;
    };

    // A lock says how it went, and only one value means it went
    auto locked = answer({0x00});
    EXPECT_TRUE(parse_block_permalock_lock(locked.data(), locked.size()));
    for (const uint8_t status : {0x01, 0x03, 0x04, 0xFF}) {
        auto refused = answer({status});
        EXPECT_FALSE(parse_block_permalock_lock(refused.data(), refused.size())) << "status " << (int)status;
    }
    // Nothing after the tag is not an answer at all
    EXPECT_FALSE(parse_block_permalock_lock(tag.data(), tag.size()));
    EXPECT_FALSE(parse_block_permalock_lock(nullptr, 0));

    // A read carries a range counted in sixteens and one word of mask for each
    std::vector<uint8_t> mask{};
    auto one_word = answer({0x01, 0x10, 0x00});
    EXPECT_TRUE(parse_block_permalock_read(mask, one_word.data(), one_word.size()));
    EXPECT_EQ(mask, (std::vector<uint8_t>{0x10, 0x00}));

    auto two_words = answer({0x02, 0x80, 0x00, 0x00, 0x01});
    EXPECT_TRUE(parse_block_permalock_read(mask, two_words.data(), two_words.size()));
    EXPECT_EQ(mask, (std::vector<uint8_t>{0x80, 0x00, 0x00, 0x01}));

    // A range the mask cannot cover, or none at all, leaves nothing to report
    auto short_mask = answer({0x02, 0x80, 0x00});
    EXPECT_FALSE(parse_block_permalock_read(mask, short_mask.data(), short_mask.size()));
    EXPECT_TRUE(mask.empty());
    auto no_range = answer({0x00});
    EXPECT_FALSE(parse_block_permalock_read(mask, no_range.data(), no_range.size()));
    EXPECT_FALSE(parse_block_permalock_read(mask, tag.data(), tag.size()));

    // A count that overruns the frame is not a tag, however much follows it
    auto overrun = answer({0x01, 0x10, 0x00});
    overrun[0]   = 0x40;
    EXPECT_FALSE(parse_block_permalock_read(mask, overrun.data(), overrun.size()));
    EXPECT_FALSE(parse_block_permalock_lock(overrun.data(), overrun.size()));
}

TEST(UHF, ErrorAnswersCommand)
{
    using namespace m5::unit::m100;

    // A Gen2 error the tag returned carries the operation in its high nibble, so it answers
    // that command and no other. B4 is the "memory locked" a write to a locked bank gets
    EXPECT_TRUE(error_answers_command(0xB4, COMMAND_WRITE_TAG_MEMORY));
    EXPECT_FALSE(error_answers_command(0xB4, COMMAND_READ_TAG_MEMORY));
    EXPECT_FALSE(error_answers_command(0xB4, COMMAND_LOCK_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0xA3, COMMAND_READ_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0xC0, COMMAND_LOCK_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0xD0, COMMAND_KILL_TAG));
    EXPECT_TRUE(error_answers_command(0xE4, COMMAND_BLOCK_PERMALOCK));
    EXPECT_FALSE(error_answers_command(0xD0, COMMAND_LOCK_TAG_MEMORY));

    // The module's own failures are named after the command they belong to
    EXPECT_TRUE(error_answers_command(0x09, COMMAND_READ_TAG_MEMORY));
    EXPECT_FALSE(error_answers_command(0x09, COMMAND_WRITE_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0x10, COMMAND_WRITE_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0x12, COMMAND_KILL_TAG));
    EXPECT_TRUE(error_answers_command(0x13, COMMAND_LOCK_TAG_MEMORY));
    EXPECT_FALSE(error_answers_command(0x13, COMMAND_KILL_TAG));
    EXPECT_TRUE(error_answers_command(0x14, COMMAND_BLOCK_PERMALOCK));

    // Inventory Fail answers a polling command and nothing else
    EXPECT_TRUE(error_answers_command(0x15, COMMAND_MULTIPLE_POLLING));
    EXPECT_TRUE(error_answers_command(0x15, COMMAND_SINGLE_POLLING));
    EXPECT_FALSE(error_answers_command(0x15, COMMAND_READ_TAG_MEMORY));

    // Every tag operation is preceded by an access, so any of them can end in this one
    EXPECT_TRUE(error_answers_command(0x16, COMMAND_READ_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0x16, COMMAND_LOCK_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0x16, COMMAND_KILL_TAG));
    EXPECT_FALSE(error_answers_command(0x16, COMMAND_MULTIPLE_POLLING));

    // A malformed command frame and a failed hop can answer whatever was sent, and so can a
    // code this table does not name. Turning one of those away would cost a timeout
    EXPECT_TRUE(error_answers_command(0x17, COMMAND_MULTIPLE_POLLING));
    EXPECT_TRUE(error_answers_command(0x17, COMMAND_READ_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0x20, COMMAND_LOCK_TAG_MEMORY));
    EXPECT_TRUE(error_answers_command(0x7F, COMMAND_KILL_TAG));
}

TEST(UHF, TidTellsTagsApart)
{
    // Two fixed words and an XTID header, and nothing past them. Every tag of the model holds
    // these same bytes, so a mask built from them would address all of them at once
    const uint8_t chip_only[] = {0xE2, 0x80, 0x69, 0x95, 0x20, 0x00};
    EXPECT_FALSE(m5::uhf::tidTellsTagsApart(chip_only, sizeof(chip_only)));

    // Shorter still, and nothing at all
    EXPECT_FALSE(m5::uhf::tidTellsTagsApart(chip_only, 4));
    EXPECT_FALSE(m5::uhf::tidTellsTagsApart(nullptr, 6));

    // A serial past the header is what tells two tags apart
    const uint8_t serialised[] = {0xE2, 0x80, 0x69, 0x95, 0x20, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    EXPECT_TRUE(m5::uhf::tidTellsTagsApart(serialised, sizeof(serialised)));

    // Room for a serial with nothing in it leaves every tag alike again
    const uint8_t blank_serial[] = {0xE2, 0x80, 0x69, 0x95, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_FALSE(m5::uhf::tidTellsTagsApart(blank_serial, sizeof(blank_serial)));

    // UCODE G2iM keeps a serial without announcing one: its XTID header reads zero, and bit 08h
    // with it, yet the two tags below differ. The rule has to pass these
    const uint8_t g2im_a[] = {0xE2, 0x00, 0x68, 0x0A, 0x00, 0x00, 0x40, 0x00, 0x02, 0xA3, 0xB9, 0x13};
    const uint8_t g2im_b[] = {0xE2, 0x00, 0x68, 0x0A, 0x00, 0x00, 0x40, 0x00, 0x02, 0xA3, 0xB1, 0x13};
    EXPECT_FALSE(m5::uhf::tidHasXtid(g2im_a, sizeof(g2im_a)));
    EXPECT_TRUE(m5::uhf::tidTellsTagsApart(g2im_a, sizeof(g2im_a)));
    EXPECT_TRUE(m5::uhf::tidTellsTagsApart(g2im_b, sizeof(g2im_b)));
    EXPECT_NE(0, memcmp(g2im_a, g2im_b, sizeof(g2im_a)));

    // The two fixed words of the same chip say nothing about the individual, which is what
    // identify() is left holding when the header says there is no serial
    EXPECT_FALSE(m5::uhf::tidTellsTagsApart(g2im_a, 4));
}

TEST(UHF, TidReadPlan)
{
    // Nothing read yet: the two fixed words come first, since they say what follows
    m5::uhf::TidReadPlan plan = m5::uhf::tidReadPlan(nullptr, 0);
    EXPECT_TRUE(plan.fits);
    EXPECT_EQ(plan.word_address, 0);
    EXPECT_EQ(plan.words, 2);

    // Bit 08h clear: no XTID header to say how long the TID is. These bytes name a UCODE G2iM,
    // whose datasheet puts a permalocked serial at 30h to 5Fh under a header that reads zero,
    // so four more words follow the two in hand
    const uint8_t no_xtid[] = {0xE2, 0x00, 0x68, 0x0A};
    EXPECT_EQ(m5::uhf::chipFromTid(no_xtid, sizeof(no_xtid)), m5::uhf::Chip::NxpUcodeG2iM);
    plan = m5::uhf::tidReadPlan(no_xtid, sizeof(no_xtid));
    EXPECT_EQ(plan.word_address, 2);
    EXPECT_EQ(plan.words, 4);

    // Once those six words are in hand there is nothing more to ask for
    const uint8_t g2im_read[] = {0xE2, 0x00, 0x68, 0x0A, 0x00, 0x00, 0x40, 0x00, 0x02, 0xA3, 0xA9, 0x13};
    plan                      = m5::uhf::tidReadPlan(g2im_read, sizeof(g2im_read));
    EXPECT_EQ(plan.words, 0);
    EXPECT_TRUE(m5::uhf::tidTellsTagsApart(g2im_read, sizeof(g2im_read)));

    // The UCODE 9xe datasheet gives both of its TIDs outright, one for each model number the
    // part has carried (SL3S1216 Rev3.3 Table 5 note 1). Both name the same chip
    const uint8_t ucode9xe_old[] = {0xE2, 0x80, 0x6A, 0x16};
    const uint8_t ucode9xe_new[] = {0xE2, 0x80, 0x6A, 0x96};
    EXPECT_EQ(m5::uhf::chipFromTid(ucode9xe_old, sizeof(ucode9xe_old)), m5::uhf::Chip::NxpUcode9xe);
    EXPECT_EQ(m5::uhf::chipFromTid(ucode9xe_new, sizeof(ucode9xe_new)), m5::uhf::Chip::NxpUcode9xe);

    // A Higgs 3 is the same shape as a G2iM: the XTID indicator reads zero, and four words of
    // unique ID follow the two that name the chip (Higgs 3 Supplement, TID Bank rows 0 to 5)
    const uint8_t higgs3[] = {0xE2, 0x00, 0x34, 0x12};
    EXPECT_EQ(m5::uhf::chipFromTid(higgs3, sizeof(higgs3)), m5::uhf::Chip::AlienHiggs3);
    plan = m5::uhf::tidReadPlan(higgs3, sizeof(higgs3));
    EXPECT_EQ(plan.word_address, 2);
    EXPECT_EQ(plan.words, 4);

    // A chip no table names says nothing beyond its fixed words, so the TID ends there
    const uint8_t unlisted[] = {0xE2, 0x00, 0x6F, 0xFF};
    EXPECT_EQ(m5::uhf::chipFromTid(unlisted, sizeof(unlisted)), m5::uhf::Chip::Unknown);
    plan = m5::uhf::tidReadPlan(unlisted, sizeof(unlisted));
    EXPECT_EQ(plan.words, 0);

    // Bit 08h set: the header word follows, and only it says how long the rest is
    const uint8_t xtid[] = {0xE2, 0x80, 0x69, 0x95};
    plan                 = m5::uhf::tidReadPlan(xtid, sizeof(xtid));
    EXPECT_EQ(plan.word_address, 2);
    EXPECT_EQ(plan.words, 1);

    // A header of zero announces no serial and no segments, so the three words are the whole
    const uint8_t bare[] = {0xE2, 0x80, 0x69, 0x95, 0x00, 0x00};
    plan                 = m5::uhf::tidReadPlan(bare, sizeof(bare));
    EXPECT_EQ(plan.words, 0);

    // A 48-bit serial makes six words in all, three of which are already in hand
    const uint8_t serial48[] = {0xE2, 0x80, 0x69, 0x95, 0x20, 0x00};
    plan                     = m5::uhf::tidReadPlan(serial48, sizeof(serial48));
    EXPECT_EQ(plan.word_address, 3);
    EXPECT_EQ(plan.words, 3);

    // Once those six words are in hand there is nothing left to ask for
    const uint8_t serial48_read[] = {0xE2, 0x80, 0x69, 0x95, 0x20, 0x00, 1, 2, 3, 4, 5, 6};
    plan                          = m5::uhf::tidReadPlan(serial48_read, sizeof(serial48_read));
    EXPECT_EQ(plan.words, 0);

    // The longest TID the standard can describe is twenty words, and it still fits in what a
    // Tid holds. That equality is what TID_MAX_BYTES is sized for
    const uint8_t largest[] = {0xE2, 0x80, 0x69, 0x95, 0xFE, 0x00};
    plan                    = m5::uhf::tidReadPlan(largest, sizeof(largest));
    EXPECT_TRUE(plan.fits);
    EXPECT_EQ(plan.word_address, 3);
    EXPECT_EQ(plan.words, m5::uhf::TID_MAX_BYTES / 2 - 3);
}

TEST(UHF, QueryParameters)
{
    // The vendor documents the layout by spelling this word out: DR=8, M=1, TRext=use pilot
    // tone, Sel=00, Session=00, Target=A, Q=4. Only a top-down packing with the padding at the
    // bottom produces those values, which is what the module was found to be set to
    m5::uhf::QueryParameters qp{};
    m5::unit::m100::parse_query_parameters(qp, 0x1020);
    EXPECT_EQ(qp.q, 4);
    EXPECT_EQ(qp.target, m5::uhf::Target::A);
    EXPECT_EQ(qp.session, m5::uhf::Session::S0);
    EXPECT_EQ(qp.filter, m5::uhf::SelectFilter::All);

    // Writing the same values back leaves the word alone, DR/M/TRext included
    EXPECT_EQ(m5::unit::m100::build_query_parameters(qp, 0x1020), 0x1020);

    // Each field lands where the layout says, one at a time from a cleared word
    m5::uhf::QueryParameters zero{};
    EXPECT_EQ(m5::unit::m100::build_query_parameters(zero, 0x0000), 0x0000);
    zero.q = 0x0F;
    EXPECT_EQ(m5::unit::m100::build_query_parameters(zero, 0x0000), 0x0078);
    zero        = m5::uhf::QueryParameters{};
    zero.target = m5::uhf::Target::B;
    EXPECT_EQ(m5::unit::m100::build_query_parameters(zero, 0x0000), 0x0080);
    zero         = m5::uhf::QueryParameters{};
    zero.session = m5::uhf::Session::S3;
    EXPECT_EQ(m5::unit::m100::build_query_parameters(zero, 0x0000), 0x0300);
    zero        = m5::uhf::QueryParameters{};
    zero.filter = m5::uhf::SelectFilter::Selected;
    EXPECT_EQ(m5::unit::m100::build_query_parameters(zero, 0x0000), 0x0C00);

    // DR, M and TRext are carried over untouched, and the unused low bits are left as found
    m5::uhf::QueryParameters all{};
    all.q                  = 0x0F;
    all.target             = m5::uhf::Target::B;
    all.session            = m5::uhf::Session::S3;
    all.filter             = m5::uhf::SelectFilter::Selected;
    const uint16_t written = m5::unit::m100::build_query_parameters(all, 0xF005);
    EXPECT_EQ(written & 0xF000, 0xF000);
    EXPECT_EQ(written & 0x0007, 0x0005);
    // ...and reading it back yields what was asked for
    m5::uhf::QueryParameters back{};
    m5::unit::m100::parse_query_parameters(back, written);
    EXPECT_EQ(back.q, all.q);
    EXPECT_EQ(back.target, all.target);
    EXPECT_EQ(back.session, all.session);
    EXPECT_EQ(back.filter, all.filter);
}

TEST(UHF, WorthRetrying)
{
    using namespace m5::unit::m100;
    // "The tag did not answer" says nothing about the tag being unwilling, so ask again
    EXPECT_TRUE(is_worth_retrying(static_cast<uint8_t>(Error::ReadFail)));
    EXPECT_TRUE(is_worth_retrying(static_cast<uint8_t>(Error::WriteFail)));
    EXPECT_TRUE(is_worth_retrying(static_cast<uint8_t>(Error::LockFail)));
    EXPECT_TRUE(is_worth_retrying(static_cast<uint8_t>(Error::KillFail)));
    EXPECT_TRUE(is_worth_retrying(static_cast<uint8_t>(Error::BlockPermalockFail)));

    // A module that reset itself comes back without the mask it was holding, so the same
    // command sent again would address whichever tag answers rather than the one that was chosen
    EXPECT_FALSE(is_worth_retrying(static_cast<uint8_t>(Error::WatchDogReset)));

    // A tag that gave a reason gives the same reason however often it is asked. The memory
    // overrun a tag without user memory answers with is the one this was measured against
    EXPECT_FALSE(is_worth_retrying(TAG_ERROR_READ | 0x03));
    EXPECT_FALSE(is_worth_retrying(TAG_ERROR_READ | 0x04));
    EXPECT_FALSE(is_worth_retrying(TAG_ERROR_WRITE | 0x04));
    EXPECT_FALSE(is_worth_retrying(TAG_ERROR_LOCK | 0x04));
    EXPECT_FALSE(is_worth_retrying(TAG_ERROR_KILL | 0x0B));
    EXPECT_FALSE(is_worth_retrying(TAG_ERROR_BLOCK_PERMALOCK | 0x03));

    // Repeating a failed access straight away is what starts a security timeout on the tag
    EXPECT_FALSE(is_worth_retrying(static_cast<uint8_t>(Error::AccessFail)));
    // A malformed command frame is ours to fix, not the tag's
    EXPECT_FALSE(is_worth_retrying(static_cast<uint8_t>(Error::CommandError)));
    // Inventory Fail never reaches a tag operation; it belongs to polling
    EXPECT_FALSE(is_worth_retrying(static_cast<uint8_t>(Error::InventoryFail)));
    EXPECT_FALSE(is_worth_retrying(static_cast<uint8_t>(Error::FHSSFail)));
    EXPECT_FALSE(is_worth_retrying(0x00));
}

TEST(UHF, DemodulatorParameters)
{
    using namespace m5::unit::m100;
    // The payload the vendor spells out: mixer 9dB, IF 36dB, threshold 0x01B0
    const uint8_t factory[] = {0x03, 0x06, 0x01, 0xB0};
    m5::unit::m100::DemodulatorParameters dp{};
    EXPECT_TRUE(parse_demodulator_parameters(dp, factory, sizeof(factory)));
    EXPECT_EQ(dp.mixer_gain, m5::unit::m100::MixerGain::dB9);
    EXPECT_EQ(dp.if_gain, m5::unit::m100::IFGain::dB36);
    EXPECT_EQ(dp.threshold, m5::unit::m100::DEMODULATOR_THRESHOLD_DEFAULT);
    // Which is also what the struct defaults to, so a caller that touches nothing writes back
    // exactly what the module already holds
    EXPECT_EQ(dp.mixer_gain, m5::unit::m100::DemodulatorParameters{}.mixer_gain);
    EXPECT_EQ(dp.if_gain, m5::unit::m100::DemodulatorParameters{}.if_gain);
    EXPECT_EQ(dp.threshold, m5::unit::m100::DemodulatorParameters{}.threshold);

    std::vector<uint8_t> built{};
    EXPECT_TRUE(build_demodulator_parameters(built, dp));
    EXPECT_EQ(built, std::vector<uint8_t>(factory, factory + sizeof(factory)));

    // The gain codes are indices into the tables, not the decibels themselves
    EXPECT_EQ(m5::unit::m100::mixerGainDb(m5::unit::m100::MixerGain::dB0), 0);
    EXPECT_EQ(m5::unit::m100::mixerGainDb(m5::unit::m100::MixerGain::dB9), 9);
    EXPECT_EQ(m5::unit::m100::mixerGainDb(m5::unit::m100::MixerGain::dB16), 16);
    EXPECT_EQ(m5::unit::m100::ifGainDb(m5::unit::m100::IFGain::dB12), 12);
    EXPECT_EQ(m5::unit::m100::ifGainDb(m5::unit::m100::IFGain::dB36), 36);
    EXPECT_EQ(m5::unit::m100::ifGainDb(m5::unit::m100::IFGain::dB40), 40);

    // A code past the end of either table is not something the module accepts
    const uint8_t bad_mixer[] = {0x07, 0x06, 0x01, 0xB0};
    EXPECT_FALSE(parse_demodulator_parameters(dp, bad_mixer, sizeof(bad_mixer)));
    const uint8_t bad_if[] = {0x03, 0x08, 0x01, 0xB0};
    EXPECT_FALSE(parse_demodulator_parameters(dp, bad_if, sizeof(bad_if)));
    EXPECT_FALSE(parse_demodulator_parameters(dp, factory, 3));
    EXPECT_FALSE(parse_demodulator_parameters(dp, nullptr, 4));

    m5::unit::m100::DemodulatorParameters out_of_range{};
    out_of_range.mixer_gain = static_cast<m5::unit::m100::MixerGain>(0x07);
    EXPECT_FALSE(build_demodulator_parameters(built, out_of_range));
    out_of_range         = m5::unit::m100::DemodulatorParameters{};
    out_of_range.if_gain = static_cast<m5::unit::m100::IFGain>(0x08);
    EXPECT_FALSE(build_demodulator_parameters(built, out_of_range));
}

TEST(UHF, ParseSelectParameter)
{
    // The answer the vendor spells out: SelParam 0x01, pointer 0x20, mask 96 bits, no truncation
    const uint8_t answer[] = {0x01, 0x00, 0x00, 0x00, 0x20, 0x60, 0x00, 0x30, 0x75, 0x1F,
                              0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70};
    m5::uhf::SelectParameter sp{};
    EXPECT_TRUE(m5::unit::m100::parse_select_parameter(sp, answer, sizeof(answer)));
    EXPECT_EQ(sp.target, 0);
    EXPECT_EQ(sp.action, 0);
    EXPECT_EQ(sp.bank, m5::uhf::Bank::Epc);
    EXPECT_EQ(sp.pointer_bits, 0x20U);
    EXPECT_EQ(sp.mask_length_bits, 96);
    EXPECT_FALSE(sp.truncate);
    EXPECT_EQ(sp.mask_size, 12);
    EXPECT_STREQ(sp.maskAsString().c_str(), "30751FEB705C5904E3D50D70");

    // Target and action come out of the top of the same byte the bank sits in
    const uint8_t sl[] = {0x82, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0xAB, 0xCD};
    sp                 = m5::uhf::SelectParameter{};
    EXPECT_TRUE(m5::unit::m100::parse_select_parameter(sp, sl, sizeof(sl)));
    EXPECT_EQ(sp.target, 4);  // SL
    EXPECT_EQ(sp.action, 0);
    EXPECT_EQ(sp.bank, m5::uhf::Bank::Tid);
    EXPECT_EQ(sp.pointer_bits, 0U);
    EXPECT_EQ(sp.mask_length_bits, 16);
    EXPECT_TRUE(sp.truncate);
    EXPECT_STREQ(sp.maskAsString().c_str(), "ABCD");

    // A frame too short to hold the fixed part, and one whose mask is longer than the module
    // can hold, are both refused rather than read past
    EXPECT_FALSE(m5::unit::m100::parse_select_parameter(sp, answer, 6));
    EXPECT_FALSE(m5::unit::m100::parse_select_parameter(sp, nullptr, 19));
    std::vector<uint8_t> too_long(7 + m5::uhf::SELECT_MASK_MAX_BYTES + 1, 0);
    EXPECT_FALSE(m5::unit::m100::parse_select_parameter(sp, too_long.data(), too_long.size()));
}

TEST(UHF, ResolveChip)
{
    // The TIDs the datasheets give outright, one per chip they name
    struct Case {
        uint8_t tid[4];
        m5::uhf::Chip chip;
    };
    const Case cases[]{
        {{0xE2, 0x00, 0x34, 0x12}, m5::uhf::Chip::AlienHiggs3},     // Higgs 3 Supplement Table 1
        {{0xE2, 0x00, 0x34, 0x14}, m5::uhf::Chip::AlienHiggs4},     // Higgs 4 V1.3.6 TID table
        {{0xE2, 0x00, 0x11, 0x00}, m5::uhf::Chip::ImpinjMonza4D},   // Monza 4 V10.0 Table 4-8
        {{0xE2, 0x00, 0x11, 0x05}, m5::uhf::Chip::ImpinjMonza4QT},  //
        {{0xE2, 0x00, 0x11, 0x0C}, m5::uhf::Chip::ImpinjMonza4E},   //
        {{0xE2, 0x00, 0x11, 0x14}, m5::uhf::Chip::ImpinjMonza4i},   //
        {{0xE2, 0x00, 0x11, 0x91}, m5::uhf::Chip::ImpinjM730},      // M700 Series v6.4 Table 23
        {{0xE2, 0x00, 0x11, 0x90}, m5::uhf::Chip::ImpinjM750},      //
        {{0xE2, 0x00, 0x11, 0xA0}, m5::uhf::Chip::ImpinjM770},      //
        {{0xE2, 0x00, 0x11, 0xC0}, m5::uhf::Chip::ImpinjM780},      // M780 / M781 v1.1 Table 21
        {{0xE2, 0x00, 0x11, 0xC1}, m5::uhf::Chip::ImpinjM781},      //
        {{0xE2, 0x80, 0x68, 0x90}, m5::uhf::Chip::NxpUcode7},       // SL3S1204 Rev4.0 Table 12
        {{0xE2, 0x80, 0x68, 0x94}, m5::uhf::Chip::NxpUcode8},       //
        {{0xE2, 0x80, 0x69, 0x94}, m5::uhf::Chip::NxpUcode8m},      //
    };
    for (const auto& c : cases) {
        EXPECT_EQ(m5::uhf::chipFromTid(c.tid, sizeof(c.tid)), c.chip);
    }

    // A chip with no user bank says so, rather than leaving the size unknown
    EXPECT_TRUE(m5::uhf::chipHasNoUserMemory(m5::uhf::Chip::NxpUcode7));
    EXPECT_TRUE(m5::uhf::chipHasNoUserMemory(m5::uhf::Chip::ImpinjM730));
    EXPECT_EQ(m5::uhf::chipUserMemoryBits(m5::uhf::Chip::ImpinjM730), 0U);

    // The one chip whose datasheet gives the block BlockPermalock works in
    EXPECT_EQ(m5::uhf::chipPermalockBlockBits(m5::uhf::Chip::ImpinjMonza4QT), 128U);
    EXPECT_EQ(m5::uhf::chipPermalockBlockBits(m5::uhf::Chip::AlienHiggs9), 0U);
}

TEST(UHF, SharedUserMemory)
{
    // Both of these chips take their EPC out of the same pool the User bank comes from, so the
    // size of one is what says the size of the other
    const auto pc = [](const uint16_t epc_words) { return static_cast<uint16_t>(epc_words << 11); };

    // Higgs 9, ALC-390: six words of EPC and forty-three of User as it ships
    EXPECT_EQ(m5::uhf::chipSharedUserMemoryBits(m5::uhf::Chip::AlienHiggs9, pc(6)), 43U * 16U);
    EXPECT_EQ(m5::uhf::chipSharedUserMemoryBits(m5::uhf::Chip::AlienHiggs9, pc(31)), 18U * 16U);

    // G2iM+, Rev3.7 Tables 16 and 17: 128 bit of EPC with 640 of User, or 448 with 320
    EXPECT_EQ(m5::uhf::chipSharedUserMemoryBits(m5::uhf::Chip::NxpUcodeG2iMPlus, pc(8)), 640U);
    EXPECT_EQ(m5::uhf::chipSharedUserMemoryBits(m5::uhf::Chip::NxpUcodeG2iMPlus, pc(28)), 320U);

    // A tag selected by its TID arrives with no PC, and a pool sized from that would be whole
    EXPECT_EQ(m5::uhf::chipSharedUserMemoryBits(m5::uhf::Chip::NxpUcodeG2iMPlus, 0), 0U);
    // A chip whose EPC does not come out of the User bank is not sized this way at all
    EXPECT_EQ(m5::uhf::chipSharedUserMemoryBits(m5::uhf::Chip::NxpUcodeG2iM, pc(16)), 0U);
}

TEST(UHF, ChipSizeFallback)
{
    // Every chip met so far carries an XTID with a serial number and nothing else, so the sizes
    // have to come from what the chip is known to hold. These three TIDs were read off tags
    struct Case {
        const char* what;
        uint8_t tid[6];
        m5::uhf::Chip chip;
        uint32_t user_bits;
        uint32_t epc_max_bits;
    };
    const Case cases[] = {
        {"Alien Higgs 9", {0xE2, 0x80, 0x38, 0x21, 0x20, 0x00}, m5::uhf::Chip::AlienHiggs9, 688, 496},
        {"Impinj Monza 4QT", {0xE2, 0x80, 0x11, 0x05, 0x20, 0x00}, m5::uhf::Chip::ImpinjMonza4QT, 512, 128},
        // No user memory at all, and an EPC that stops at 128 bits: a real tag takes a PC of
        // eight words and refuses nine
        {"NXP UCODE 8", {0xE2, 0x80, 0x68, 0x94, 0x20, 0x00}, m5::uhf::Chip::NxpUcode8, 0, 128},
    };
    for (auto&& c : cases) {
        m5::uhf::Tag tag{};
        EXPECT_TRUE(m5::uhf::decodeTid(tag, c.tid, sizeof(c.tid))) << c.what;
        EXPECT_EQ(tag.chip, c.chip) << c.what;
        EXPECT_EQ(tag.serial_bits, 48) << c.what;
        EXPECT_EQ(tag.user_memory_bits, c.user_bits) << c.what;
        EXPECT_EQ(tag.epc_max_bits, c.epc_max_bits) << c.what;
    }

    // What the tag says about itself wins: a chip whose table entry says 512 but whose XTID
    // reports 43 words is taken at its word
    const uint8_t with_segment[] = {
        0xE2, 0x80, 0x11, 0x05,              // Impinj Monza 4QT, table says 512 bits
        0x34, 0x00,                          // serial + Optional Command Support + User Memory
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66,  // 48-bit serial
        0x10, 0x0F,                          // Optional Command Support: max EPC 15 words
        0x00, 0x04,                          // BlockPermaLock block size
        0x00, 0x2B,                          // User memory: 43 words
    };
    m5::uhf::Tag tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, with_segment, sizeof(with_segment)));
    EXPECT_EQ(tag.user_memory_bits, 43U * 16U);
    EXPECT_EQ(tag.epc_max_bits, 15U * 16U);

    // A chip nobody listed reports nothing rather than something wrong
    const uint8_t unlisted[] = {0xE2, 0x01, 0x23, 0x45, 0x20, 0x00};
    tag                      = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, unlisted, sizeof(unlisted)));
    EXPECT_EQ(tag.chip, m5::uhf::Chip::Unknown);
    EXPECT_EQ(tag.user_memory_bits, 0U);
    EXPECT_EQ(tag.epc_max_bits, 0U);
}
