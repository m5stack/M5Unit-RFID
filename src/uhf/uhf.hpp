/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file uhf.hpp
  @brief Common vocabulary for UHF-RFID (EPCglobal UHF Class 1 Gen 2 / ISO 18000-6C)
*/
#ifndef M5_UNIT_RFID_UHF_UHF_HPP
#define M5_UNIT_RFID_UHF_UHF_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace m5 {
/*!
  @namespace uhf
  @brief UHF-RFID (EPC Gen2) related namespace
 */
namespace uhf {

/*!
  @enum Bank
  @brief EPC Gen2 memory bank
 */
enum class Bank : uint8_t {
    Reserved,  //!< Kill password and access password
    Epc,       //!< CRC-16, PC and EPC
    Tid,       //!< Tag identification
    User,      //!< User memory
};

/*!
  @enum Region
  @brief Operating region
  @note Japan is not defined by the module; the 920-925MHz band overlaps China900MHz
 */
enum class Region : uint8_t {
    Unspecified,  //!< Keep the module's factory setting
    China900MHz,
    America,
    Europe,
    China800MHz,
    SouthKorea,
};

/*!
  @enum Session
  @brief EPC Gen2 session
 */
enum class Session : uint8_t { S0, S1, S2, S3 };

/*!
  @enum Target
  @brief EPC Gen2 inventoried flag target
 */
enum class Target : uint8_t { A, B };

/*!
  @struct Tag
  @brief Detected tag
 */
struct Tag {
    uint16_t pc{};               //!< Protocol Control
    std::vector<uint8_t> epc{};  //!< EPC (variable length)
    uint16_t crc{};              //!< CRC-16
    int8_t rssi{};               //!< RSSI in dBm (signed)
};

/*!
  @struct QueryParameters
  @brief EPC Gen2 query parameters
 */
struct QueryParameters {
    uint8_t q{};                   //!< Q value
    Session session{Session::S0};  //!< Session
    Target target{Target::A};      //!< Inventoried flag target
};

/*!
  @struct ModuleInformation
  @brief Reader module information
 */
struct ModuleInformation {
    std::string hardware_version{};
    std::string software_version{};
    std::string manufacturer{};
};

/*!
  @brief EPC length in 16-bit words encoded in the PC
  @param pc Protocol Control
  @return Number of 16-bit words comprising the EPC
 */
inline uint8_t pcEPCLengthWords(const uint16_t pc)
{
    return static_cast<uint8_t>((pc >> 11) & 0x1F);
}

/*!
  @brief User Memory Indicator of the PC
  @param pc Protocol Control
  @return True if the tag reports user memory holding data
 */
inline bool pcUserMemoryIndicator(const uint16_t pc)
{
    return (pc & 0x0400) != 0;
}

/*!
  @brief XPC indicator of the PC
  @param pc Protocol Control
  @return True if an XPC word follows
 */
inline bool pcXPCIndicator(const uint16_t pc)
{
    return (pc & 0x0200) != 0;
}

/*!
  @brief Numbering System Identifier of the PC
  @param pc Protocol Control
  @return NSI (0x000 for EPCglobal)
 */
inline uint16_t pcNumberingSystemIdentifier(const uint16_t pc)
{
    return static_cast<uint16_t>(pc & 0x01FF);
}

/*!
  @brief Append a tag unless an identical EPC is already present
  @param[in,out] dst Destination
  @param tag Tag to append
  @return True if the tag was appended
  @note A tag whose EPC is empty is never appended
 */
inline bool append_unique(std::vector<Tag>& dst, const Tag& tag)
{
    if (tag.epc.empty()) {
        return false;
    }
    for (size_t i = 0; i < dst.size(); ++i) {
        if (dst[i].epc == tag.epc) {
            return false;
        }
    }
    dst.push_back(tag);
    return true;
}

}  // namespace uhf
}  // namespace m5
#endif
