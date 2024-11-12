/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Common functions and definitions for examples
*/
#include <M5Unified.hpp>
#include <M5Utility.h>
#include "examples_common.hpp"

using namespace m5::rfid::mifare;

const char* picc_type_table[] = {
    "Unknown",           "ISO_14443_4",       "ISO_18092",         "MIFARE_Classic",
    "MIFARE_Classic_1K", "MIFARE_Classic_4K", "MIFARE_Classic_2K", "MIFARE_UltraLight(orC)",
    "MIFARE_Plus",       "MIFARE_DESFire",
};

const char* UIDTypeString(const UID& uid)
{
    auto v = m5::stl::to_underlying(uid.type);
    return (v < m5::stl::size(picc_type_table)) ? picc_type_table[v] : "ERROR";
}

const std::string UIDString(const m5::rfid::mifare::UID& uid) {
    char buf[2 * 10 + 1]{};
    if (uid.size == 0 || uid.size > 10) {
        M5_LOGE("Invalid size:%u", uid.size);
        return std::string();;
    }
    uint8_t left{};
    for (uint8_t i = 0; i < uid.size; ++i) {
        left += snprintf(buf + left, 3, "%02X", uid.uid[i]);
    }
    return std::string(buf);
}



void printUID(const UID& uid)
{
    char buf[2 * 10 + 1]{};
    if (uid.size == 0 || uid.size > 10) {
        M5_LOGE("Invalid size:%u", uid.size);
        return;
    }
    uint8_t left{};
    for (uint8_t i = 0; i < uid.size; ++i) {
        left += snprintf(buf + left, 3, "%02X", uid.uid[i]);
    }
    M5_LOGI("UID:%s [%s] SAK:%02X", buf, UIDTypeString(uid), uid.sak);
}
