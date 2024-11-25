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
#include <rfid/nfc/nfc.hpp>
#include <cstring>

using namespace m5::rfid::nfc;
using namespace m5::rfid::nfc::ndef;

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
    // r.dump();

    // ID
    {
        uint8_t id[7] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

        r.setIdentifier(id, 7);
        EXPECT_EQ(r.required(), 4U + 7U);  // attr + type len + payload len + id len + id[7]
        EXPECT_TRUE(r.attribute().idLength());
        EXPECT_NE(r.identifierSize(), 0U);
        EXPECT_NE(r.identifier(), nullptr);
        // r.dump();

        uint8_t id2{0x52};
        r.setIdentifier(&id2, 1);
        EXPECT_EQ(r.required(), 4U + 1U);  // id[1]
        EXPECT_TRUE(r.attribute().idLength());
        EXPECT_NE(r.identifierSize(), 0U);
        EXPECT_NE(r.identifier(), nullptr);
        // r.dump();

        r.clearIdentifier();
        EXPECT_EQ(r.required(), 3U);
        EXPECT_FALSE(r.attribute().idLength());
        EXPECT_EQ(r.identifierSize(), 0U);
        EXPECT_EQ(r.identifier(), nullptr);
        // r.dump();

        r.setIdentifier(id, 7);
        EXPECT_EQ(r.required(), 4U + 7U);
        EXPECT_TRUE(r.attribute().idLength());
        EXPECT_NE(r.identifierSize(), 0U);
        EXPECT_NE(r.identifier(), nullptr);
        // r.dump();
    }

    // Payload
    {
        EXPECT_TRUE(strcmp(r.type(), "") == 0);

        r.setTextPayload(en_data, "en");
        EXPECT_TRUE(strcmp(r.type(), "T") == 0);
        EXPECT_EQ(r.required(), 4U + 1U + 7U +       // attr + type len + payload len + id len + type[1] + id[7]
                                    1U + 2U + 13U);  // status + lang[2] + txt[13]
        r.dump();

        r.setURIPayload("https://m5stack.com", URIProtocol::HTTPS);
        EXPECT_TRUE(strcmp(r.type(), "U") == 0);

        r.setTextPayload(ja_data, "ja");
        EXPECT_TRUE(strcmp(r.type(), "T") == 0);
        EXPECT_EQ(r.required(), 4U + 1U + 7U +       //
                                    1U + 2U + 23U);  // status + lang[2] + txt[23]
        // r.dump();

        r.setTextPayload(zh_data, "zh");
        EXPECT_TRUE(strcmp(r.type(), "T") == 0);
        EXPECT_EQ(r.required(), 4U + 1U + 7U +       //
                                    1U + 2U + 14U);  // status + lang[2] + txt[14]
        // r.dump();
    }
}

TEST(NDEF, Message)
{
    constexpr uint8_t empty[3] = {0x03, 0x00, 0xFE};
    uint8_t buf[1024]{};

    {
        Message msg{};

        EXPECT_EQ(msg.tag(), Tag::NDEFMessage);
        EXPECT_EQ(msg.records().size(), 0U);
        EXPECT_EQ(msg.required(), 3);  // tag + record len + terminator
        msg.dump();

        auto encoded = msg.encode(buf, 256);
        EXPECT_EQ(encoded, 3);
        EXPECT_TRUE(std::memcmp(buf, empty, 3) == 0);

        //
        Record r0{}, r1{}, r2{};

        // 0
        r0.setTextPayload(en_data, en_lang);
        EXPECT_TRUE(strcmp(r0.type(), "T") == 0);
        EXPECT_EQ(r0.required(), 20);
        EXPECT_TRUE(strcmp(en_data, r0.payloadAsString().c_str()) == 0);

        msg.push_back(r0);
        EXPECT_EQ(msg.records().size(), 1U);
        EXPECT_EQ(msg.required(), 3 + 20);  // tag + record len + records + terminator
        EXPECT_EQ(msg.required(), 3 + r0.required());

        encoded = msg.encode(buf, 1024);
        EXPECT_EQ(encoded, 23);
        EXPECT_EQ(encoded, msg.required());

        // 1
        r1.setTextPayload(zh_data, zh_lang);
        EXPECT_TRUE(strcmp(r1.type(), "T") == 0);
        EXPECT_EQ(r1.required(), 21);
        EXPECT_TRUE(strcmp(zh_data, r1.payloadAsString().c_str()) == 0);

        msg.push_back(r1);
        EXPECT_EQ(msg.records().size(), 2U);
        EXPECT_EQ(msg.required(), 3 + 20 + 21);  // tag + record len + records + terminator
        EXPECT_EQ(msg.required(), 3 + r0.required() + r1.required());

        encoded = msg.encode(buf, 1024);
        EXPECT_EQ(encoded, 44);
        EXPECT_EQ(encoded, msg.required());

        // 2
        r2.setURIPayload(ftp_data, URIProtocol::FTP_AA);
        EXPECT_TRUE(strcmp(r2.type(), "U") == 0);
        EXPECT_EQ(r2.required(), 17);
        EXPECT_TRUE(strcmp(ftp_data, r2.payloadAsString().c_str()) == 0);

        msg.push_back(r2);
        EXPECT_EQ(msg.records().size(), 3U);
        EXPECT_EQ(msg.required(), 3 + 20 + 21 + 17);  // tag + record len + records + terminator
        EXPECT_EQ(msg.required(), 3 + r0.required() + r1.required() + r2.required());

        encoded = msg.encode(buf, 1024);
        EXPECT_EQ(encoded, 61);
        EXPECT_EQ(encoded, msg.required());

        // M5_LOGI("[%s]", r2.payloadAsString().c_str());
        // msg.dump();
    }
}
