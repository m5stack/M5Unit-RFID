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
    EXPECT_EQ(tag.epc_max_bits, 15U * 16U);
    EXPECT_TRUE(tag.supports_block_permalock);
    EXPECT_EQ(tag.user_memory_bits, 0U);

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

    // Reading only as far as the header still yields the chip and the serial length, and leaves
    // the sizes at zero rather than inventing them out of bytes that were never read
    tag = m5::uhf::Tag{};
    EXPECT_TRUE(m5::uhf::decodeTid(tag, with_blockwrite, 6));
    EXPECT_EQ(tag.chip, m5::uhf::Chip::ImpinjMonza4QT);
    EXPECT_EQ(tag.serial_bits, 48);
    EXPECT_EQ(tag.epc_max_bits, 0U);
    EXPECT_EQ(tag.user_memory_bits, 0U);

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
    m5::unit::jrd4035::TagOperationResult r{};
    EXPECT_TRUE(m5::unit::jrd4035::parse_tag_operation(r, read_answer, sizeof(read_answer)));
    EXPECT_EQ(r.pc, 0x3400);
    EXPECT_EQ(r.epc.size, 12);
    EXPECT_STREQ(r.epc.toString().c_str(), "30751FEB705C5904E3D50D70");
    EXPECT_EQ(r.data_len, 4U);
    EXPECT_EQ(r.data[0], 0x12);

    // The answer to a Write, Lock or Kill is the same shape with a single status byte
    const uint8_t write_answer[] = {
        0x0E, 0x34, 0x00, 0x30, 0x75, 0x1F, 0xEB, 0x70, 0x5C, 0x59, 0x04, 0xE3, 0xD5, 0x0D, 0x70, 0x00,
    };
    r = m5::unit::jrd4035::TagOperationResult{};
    EXPECT_TRUE(m5::unit::jrd4035::parse_tag_operation(r, write_answer, sizeof(write_answer)));
    EXPECT_EQ(r.data_len, 1U);
    EXPECT_EQ(r.data[0], m5::unit::jrd4035::TAG_OPERATION_SUCCESS);

    // A count that runs past the frame, a count too small to hold the PC, and nothing at all
    const uint8_t overrun[] = {0x40, 0x34, 0x00, 0x30};
    r                       = m5::unit::jrd4035::TagOperationResult{};
    EXPECT_FALSE(m5::unit::jrd4035::parse_tag_operation(r, overrun, sizeof(overrun)));
    const uint8_t too_small[] = {0x01, 0x34, 0x00, 0x30};
    EXPECT_FALSE(m5::unit::jrd4035::parse_tag_operation(r, too_small, sizeof(too_small)));
    EXPECT_FALSE(m5::unit::jrd4035::parse_tag_operation(r, nullptr, 16));
    EXPECT_FALSE(m5::unit::jrd4035::parse_tag_operation(r, read_answer, 3));
}

TEST(UHF, ErrorDescription)
{
    // The high nibble says which command failed and the low one says why, so the same Gen2
    // code has to read the same whichever operation provoked it
    EXPECT_TRUE(m5::unit::jrd4035::is_tag_error(m5::unit::jrd4035::TAG_ERROR_READ | 0x04));
    EXPECT_TRUE(m5::unit::jrd4035::is_tag_error(m5::unit::jrd4035::TAG_ERROR_KILL));
    EXPECT_STREQ(m5::unit::jrd4035::error_description(m5::unit::jrd4035::TAG_ERROR_READ | 0x04),
                 m5::unit::jrd4035::error_description(m5::unit::jrd4035::TAG_ERROR_WRITE | 0x04));
    EXPECT_STREQ(m5::unit::jrd4035::error_description(0xA3), "Tag: memory overrun");
    EXPECT_STREQ(m5::unit::jrd4035::error_description(0xD0), "Tag: other error");

    // Module-level failures are not tag errors and keep their own meanings
    EXPECT_FALSE(m5::unit::jrd4035::is_tag_error(0x15));
    EXPECT_FALSE(m5::unit::jrd4035::is_tag_error(0x16));
    EXPECT_STREQ(m5::unit::jrd4035::error_description(0x16), "Access failed: wrong access password");
    EXPECT_STREQ(m5::unit::jrd4035::error_description(0x15), "No tag answered");
}
