/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file rfid.cpp
  @brief Definition for RFID
*/
#include "rfid.hpp"
#include <M5Utility.hpp>

namespace {
constexpr char type_unknown[]       = "Unknown";
constexpr char type_classic[]       = "MIFARE Classic";
constexpr char type_classic_1K[]    = "MIFARE Classsic 1K";
constexpr char type_classic_2K[]    = "MIFARE Classsic 2K";
constexpr char type_classic_4K[]    = "MIFARE Classsic 4K";
constexpr char type_ultra_light[]   = "MIFARE Ultralight";
constexpr char type_ultra_light_c[] = "MIFARE UltralightC";
constexpr char type_plus_2K[]       = "MIFARE Plus 2K";
constexpr char type_plus_4K[]       = "MIFARE Plus 4K";
constexpr char type_desfire_2K[]    = "MIFARE DESFire 2K";
constexpr char type_desfire_4K[]    = "MIFARE DESFire 4K";
constexpr char type_desfire_8K[]    = "MIFARE DESFire 8K";
constexpr char type_ntag203[]       = "NTAG 203";
constexpr char type_ntag210u[]      = "NTAG 210u";
constexpr char type_ntag210[]       = "NTAG 210";
constexpr char type_ntag212[]       = "NTAG 212";
constexpr char type_ntag213[]       = "NTAG 213";
constexpr char type_ntag215[]       = "NTAG 215";
constexpr char type_ntag216[]       = "NTAG 216";
constexpr char type_iso14443_4[]    = "ISO14443-4";
constexpr char type_iso18092[]      = "ISO18092";
constexpr char type_not_completed[] = "NOT COMPLEDTED";
constexpr const char* type_table[]  = {
    type_unknown,                                                            //
    type_classic,     type_classic_1K,    type_classic_2K, type_classic_4K,  // Classic
    type_ultra_light, type_ultra_light_c,                                    // Light
    type_plus_2K,     type_plus_4K,                                          // Plus
    type_desfire_2K,  type_desfire_4K,    type_desfire_8K,                   // DESFire
    type_ntag203,     type_ntag210u,      type_ntag210,                      // NTAG
    type_ntag212,     type_ntag213,       type_ntag215,                      // NTAG
    type_ntag216,                                                            // NTAG
    type_iso14443_4,  type_iso18092,                                         //
};

// included system area
constexpr uint16_t max_block_table[] = {
    0,                                // Unknown
    20,  64,  128, 256,               // Classic
    16,  48,                          // Light
    128, 256,                         // Plus
    0,   0,   0,                      // DESFire (Not has blocks, File base system)
    42,  19,  16,  40,  44, 134, 230  // NTAG
};

// [min/max]
constexpr uint8_t user_block_table[][2] = {
    {0, 0},
    // Classic
    {1, 19},
    {1, 63},
    {0, 127},
    {0, 255},
    // Light
    {4, 15},
    {4, 39},
    // Plus
    {0, 0},
    {0, 0},
    // DESFire
    {0, 0},
    {0, 0},
    {0, 0},
    // NTAG
    {4, 39},   // 203
    {4, 15},   // 210u
    {4, 15},   // 210
    {4, 35},   // 212
    {4, 39},   // 213
    {4, 129},  // 215
    {4, 225},  // 216
};

constexpr uint8_t max_sector_table[] = {
    0,               //
    5,  16, 32, 40,  // Classic
    0,  0,           // Light
    32, 40,          // Plus
};

}  // namespace

namespace m5 {
namespace rfid {

Type get_type(const uint8_t sak)
{
    if (sak & 0x02) {
        return Type::Unknown;
    }
    if (sak & 0x04) {
        return Type::NotCompleted;
    }
    if (sak & 0x20) {
        return Type::ISO_14443_4;
    }
    if (sak & 0x40) {
        return Type::ISO_18092;
    }
    switch (sak) {
        case 0x00:
            return Type::MIFARE_UltraLight;  // or C or NTAG
        case 0x01:
            return Type::MIFARE_DESFire_2K;  // or 4K or 8K
        case 0x08:
            return Type::MIFARE_Classic_1K;
        case 0x09:
            return Type::MIFARE_Classic;
        case 0x10:
            return Type::MIFARE_Plus_2K;
        case 0x11:
            return Type::MIFARE_Plus_4K;
        case 0x18:
            return Type::MIFARE_Classic_4K;
        case 0x19:
            return Type::MIFARE_Classic_2K;
        default:
            break;
    }
    return Type::Unknown;
}

uint16_t get_number_of_blocks(const Type t)
{
    uint8_t idx = m5::stl::to_underlying(t);
    return max_block_table[idx < m5::stl::size(max_block_table) ? idx : 0];
}

uint8_t get_number_of_sectors(const Type t)
{
    uint8_t idx = m5::stl::to_underlying(t);
    return max_sector_table[idx < m5::stl::size(max_sector_table) ? idx : 0];
}

uint8_t get_first_user_block(const Type t)
{
    uint8_t idx = m5::stl::to_underlying(t);
    return user_block_table[idx < m5::stl::size(user_block_table) ? idx : 0][0];
}

uint8_t get_last_user_block(const Type t)
{
    uint8_t idx = m5::stl::to_underlying(t);
    return user_block_table[idx < m5::stl::size(user_block_table) ? idx : 0][1];
}

uint8_t get_user_block_size(const Type t)
{
    uint8_t idx = m5::stl::to_underlying(t);
    auto p      = user_block_table[idx < m5::stl::size(user_block_table) ? idx : 0];
    uint8_t sz  = p[1] - p[0];
    return sz ? (sz + 1) : 0;
}

std::string UID::uidAsString() const
{
    char buf[2 * 10 + 1]{};
    if (this->size <= 10) {
        uint8_t left{};
        for (uint8_t i = 0; i < this->size; ++i) {
            left += snprintf(buf + left, 3, "%02X", this->uid[i]);
        }
    }
    return std::string(buf);
}

std::string UID::typeAsString() const
{
    return std::string(m5::stl::to_underlying(this->type) <= m5::stl::size(type_table)
                           ? type_table[m5::stl::to_underlying(this->type)]
                           : type_not_completed);
}

}  // namespace rfid
}  // namespace m5
