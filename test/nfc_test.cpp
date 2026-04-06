/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for NFC
*/
#include <gtest/gtest.h>
#include <M5Unified.h>
#include <nfc/nfc.hpp>
#include <nfc/ndef/ndef.hpp>
#include <nfc/ndef/ndef_record.hpp>
#include <nfc/ndef/ndef_tlv.hpp>
#include <cstring>

using namespace m5::nfc;
using namespace m5::nfc::ndef;

namespace {

constexpr char en_data[] = "Hello M5Stack";       // 13
constexpr char ja_data[] = "こんにちは M5Stack";  // 23
constexpr char zh_data[] = "你好 M5Stack";        // 14

constexpr char en_lang[] = "en";
constexpr char ja_lang[] = "ja";
constexpr char zh_lang[] = "zh";

constexpr char ftp_data[] = "ftp://anonymous:anonymous@example.com/";

}  // namespace

TEST(NDEF, Record)
{
    Record r(TNF::Wellknown);

    EXPECT_EQ(r.tnf(), TNF::Wellknown);
    EXPECT_EQ(r.required(), 3U);  // attr + type len + payload len
    EXPECT_FALSE(r.attribute().idLength());
    EXPECT_TRUE(strcmp(r.type(), "") == 0);
    EXPECT_EQ(r.identifierSize(), 0U);
    EXPECT_EQ(r.identifier(), nullptr);
    EXPECT_EQ(r.payloadSize(), 0U);
    EXPECT_EQ(r.payload(), nullptr);

    // ID
    {
        uint8_t id[7] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

        r.setIdentifier(id, 7);
        EXPECT_EQ(r.required(), 4U + 7U);  // attr + type len + payload len + id len + id[7]
        EXPECT_TRUE(r.attribute().idLength());
        EXPECT_EQ(r.identifierSize(), 7U);
        EXPECT_NE(r.identifier(), nullptr);

        uint8_t id2{0x52};
        r.setIdentifier(&id2, 1);
        EXPECT_EQ(r.required(), 4U + 1U);  // id[1]
        EXPECT_TRUE(r.attribute().idLength());
        EXPECT_EQ(r.identifierSize(), 1U);
        EXPECT_NE(r.identifier(), nullptr);

        r.clearIdentifier();
        EXPECT_EQ(r.required(), 3U);
        EXPECT_FALSE(r.attribute().idLength());
        EXPECT_EQ(r.identifierSize(), 0U);
        EXPECT_EQ(r.identifier(), nullptr);

        r.setIdentifier(id, 7);
        EXPECT_EQ(r.required(), 4U + 7U);
        EXPECT_TRUE(r.attribute().idLength());
        EXPECT_EQ(r.identifierSize(), 7U);
        EXPECT_NE(r.identifier(), nullptr);
    }

    // Payload
    {
        EXPECT_TRUE(strcmp(r.type(), "") == 0);

        r.setTextPayload(en_data, "en");
        EXPECT_TRUE(strcmp(r.type(), "T") == 0);
        EXPECT_EQ(r.required(), 4U + 1U + 7U +       // attr + type len + payload len + id len + type[1] + id[7]
                                    1U + 2U + 13U);  // status + lang[2] + txt[13]

        r.setURIPayload("https://m5stack.com", URIProtocol::HTTPS);
        EXPECT_TRUE(strcmp(r.type(), "U") == 0);

        r.setTextPayload(ja_data, "ja");
        EXPECT_TRUE(strcmp(r.type(), "T") == 0);
        EXPECT_EQ(r.required(), 4U + 1U + 7U +       //
                                    1U + 2U + 23U);  // status + lang[2] + txt[23]

        r.setTextPayload(zh_data, "zh");
        EXPECT_TRUE(strcmp(r.type(), "T") == 0);
        EXPECT_EQ(r.required(), 4U + 1U + 7U +       //
                                    1U + 2U + 14U);  // status + lang[2] + txt[14]
    }

    {
        uint8_t buf[256]{};
        auto encoded = r.encode(buf, 256);
        EXPECT_EQ(encoded, 4U + 1U + 7U +       //
                               1U + 2U + 14U);  // status + lang[2] + txt[14]
        Record r2{};
        auto decoded = r2.decode(buf, encoded);
        EXPECT_EQ(encoded, decoded);

        EXPECT_EQ(r.attribute().value, r2.attribute().value);
        auto tlen  = strlen(r.type());
        auto tlen2 = strlen(r2.type());
        EXPECT_EQ(tlen, tlen2);
        EXPECT_TRUE(std::memcmp(r.type(), r2.type(), tlen) == 0);

        auto ilen  = r.identifierSize();
        auto ilen2 = r2.identifierSize();
        EXPECT_EQ(ilen, ilen2);
        EXPECT_TRUE(std::memcmp(r.identifier(), r2.identifier(), ilen) == 0);

        auto plen  = r.payloadSize();
        auto plen2 = r2.payloadSize();
        EXPECT_EQ(plen, plen2);
        EXPECT_TRUE(std::memcmp(r.payload(), r2.payload(), plen) == 0);
    }
}

TEST(NDEF, TLV)
{
    uint8_t buf[1024]{};
    TLV msg{Tag::Message};
    Record r0{}, r1{}, r2{};

    {
        EXPECT_EQ(msg.tag(), Tag::Message);
        EXPECT_EQ(msg.records().size(), 0U);

        // 0
        r0.setTextPayload(en_data, en_lang);
        EXPECT_TRUE(strcmp(r0.type(), "T") == 0);
        EXPECT_EQ(r0.required(), 20);
        EXPECT_TRUE(strcmp(en_data, r0.payloadAsString().c_str()) == 0);

        msg.push_back(r0);
        EXPECT_EQ(msg.records().size(), 1U);

        auto encoded = msg.encode(buf, 1024);
        EXPECT_GT(encoded, 0U);

        // 1
        r1.setTextPayload(zh_data, zh_lang);
        EXPECT_TRUE(strcmp(r1.type(), "T") == 0);
        EXPECT_EQ(r1.required(), 21);
        EXPECT_TRUE(strcmp(zh_data, r1.payloadAsString().c_str()) == 0);

        msg.push_back(r1);
        EXPECT_EQ(msg.records().size(), 2U);

        encoded = msg.encode(buf, 1024);
        EXPECT_GT(encoded, 0U);

        // 2
        r2.setURIPayload(ftp_data, URIProtocol::FTP_AA);
        EXPECT_TRUE(strcmp(r2.type(), "U") == 0);
        EXPECT_EQ(r2.required(), 17);
        EXPECT_TRUE(strcmp(ftp_data, r2.payloadAsString().c_str()) == 0);

        msg.push_back(r2);
        EXPECT_EQ(msg.records().size(), 3U);

        encoded = msg.encode(buf, 1024);
        EXPECT_GT(encoded, 0U);
    }

    {
        auto encoded = msg.encode(buf, 1024);
        TLV msg2{};
        auto decoded = msg2.decode(buf, encoded);

        EXPECT_EQ(encoded, decoded);
        EXPECT_EQ(msg.tag(), msg2.tag());
        EXPECT_EQ(msg.records().size(), msg2.records().size());
        EXPECT_EQ(msg.records(), msg2.records());
    }
}
