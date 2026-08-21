/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file jrd4035_frame.hpp
  @brief Frame codec for the magicRF M100 family (JRD-4035 / JRD-100 / R200)
  @note Intentionally free of M5UnitComponent so that it can be built and tested
  without the ESP32 toolchain
*/
#ifndef M5_UNIT_RFID_UNIT_JRD4035_FRAME_HPP
#define M5_UNIT_RFID_UNIT_JRD4035_FRAME_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include <m5_utility/crc.hpp>

#include "uhf/uhf.hpp"

namespace m5 {
namespace unit {
/*!
  @namespace jrd4035
  @brief JRD-4035 (magicRF M100) specific definitions
 */
namespace jrd4035 {

//! @brief Frame header of the JRD-4035 / JRD-100
constexpr uint8_t FRAME_HEADER{0xBB};
//! @brief Frame end of the JRD-4035 / JRD-100
constexpr uint8_t FRAME_END{0x7E};

/*!
  @brief Calculate the frame checksum
  @param body Pointer to the Type byte (the checksum covers Type through the last Parameter byte)
  @param len Length in bytes
  @return Least significant byte of the sum
 */
inline uint8_t checksum(const uint8_t* body, const size_t len)
{
    uint32_t sum{};
    for (size_t i = 0; i < len; ++i) {
        sum += body[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

//! @brief Maximum parameter length accepted for a frame
constexpr uint16_t MAX_PARAMETER_LENGTH{512};

/*!
  @brief Build a command frame
  @param[out] out Built frame
  @param type Frame type (0x00: command)
  @param command Command code
  @param param Parameter (nullptr if none)
  @param param_len Parameter length
  @param header Frame header (0xBB for JRD-4035, 0xAA for R200)
  @param end Frame end (0x7E for JRD-4035, 0xDD for R200)
  @return True if successful
 */
inline bool build_frame(std::vector<uint8_t>& out, const uint8_t type, const uint8_t command, const uint8_t* param,
                        const uint16_t param_len, const uint8_t header = FRAME_HEADER, const uint8_t end = FRAME_END)
{
    if (param_len > MAX_PARAMETER_LENGTH || (param == nullptr && param_len != 0)) {
        return false;
    }

    out.clear();
    out.reserve(static_cast<size_t>(param_len) + 7);
    out.push_back(header);
    out.push_back(type);
    out.push_back(command);
    out.push_back(static_cast<uint8_t>(param_len >> 8));
    out.push_back(static_cast<uint8_t>(param_len & 0xFF));
    for (uint16_t i = 0; i < param_len; ++i) {
        out.push_back(param[i]);
    }
    // The checksum covers Type through the last Parameter byte (index 1 onwards)
    out.push_back(checksum(out.data() + 1, out.size() - 1));
    out.push_back(end);
    return true;
}

//! @brief Fixed part of a frame (Header, Type, Command, PL_MSB, PL_LSB, Checksum, End)
constexpr size_t FRAME_OVERHEAD{7};
//! @brief Offset of the Type byte
constexpr size_t FRAME_TYPE_OFFSET{1};
//! @brief Offset of the Parameter
constexpr size_t FRAME_PARAMETER_OFFSET{5};

//! @brief Frame type: command (host to module)
constexpr uint8_t TYPE_COMMAND{0x00};
//! @brief Frame type: response (module to host)
constexpr uint8_t TYPE_RESPONSE{0x01};
//! @brief Frame type: notification (module to host)
constexpr uint8_t TYPE_NOTIFICATION{0x02};

/*!
  @struct Frame
  @brief Parsed frame
 */
struct Frame {
    uint8_t type{};                    //!< Frame type
    uint8_t command{};                 //!< Command code
    std::vector<uint8_t> parameter{};  //!< Parameter
};

/*!
  @brief Parse a frame
  @param[out] out Parsed frame
  @param raw Raw bytes (a whole frame including the header and the end)
  @param len Length of raw
  @param header Expected frame header
  @param end Expected frame end
  @return True if the frame is well-formed and the checksum matches
 */
inline bool parse_frame(Frame& out, const uint8_t* raw, const size_t len, const uint8_t header = FRAME_HEADER,
                        const uint8_t end = FRAME_END)
{
    if (raw == nullptr || len < FRAME_OVERHEAD) {
        return false;
    }
    if (raw[0] != header || raw[len - 1] != end) {
        return false;
    }

    const uint16_t param_len = static_cast<uint16_t>((raw[3] << 8) | raw[4]);
    if (param_len > MAX_PARAMETER_LENGTH || len != static_cast<size_t>(param_len) + FRAME_OVERHEAD) {
        return false;
    }
    // The checksum covers Type through the last Parameter byte
    if (raw[len - 2] != checksum(raw + FRAME_TYPE_OFFSET, len - 3)) {
        return false;
    }

    out.type    = raw[1];
    out.command = raw[2];
    out.parameter.assign(raw + FRAME_PARAMETER_OFFSET, raw + FRAME_PARAMETER_OFFSET + param_len);
    return true;
}

//! @brief Command code of the single polling notification
constexpr uint8_t COMMAND_SINGLE_POLLING{0x22};
//! @brief Command code of the multiple polling notification
constexpr uint8_t COMMAND_MULTIPLE_POLLING{0x27};
//! @brief Fixed part of a tag notification (RSSI, PC and CRC)
constexpr size_t TAG_NOTIFICATION_OVERHEAD{5};

/*!
  @brief Parse the parameter of a tag notification
  @param[out] out Parsed tag
  @param param Parameter of the notification frame (RSSI, PC, EPC and CRC)
  @param len Parameter length
  @return True if successful
  @note The EPC length is derived from len, so a variable length EPC is supported
 */
inline bool parse_tag_notification(m5::uhf::Tag& out, const uint8_t* param, const size_t len)
{
    if (param == nullptr || len <= TAG_NOTIFICATION_OVERHEAD) {
        return false;
    }

    const size_t epc_len = len - TAG_NOTIFICATION_OVERHEAD;
    out.rssi             = static_cast<int8_t>(param[0]);
    out.pc               = static_cast<uint16_t>((param[1] << 8) | param[2]);
    out.epc.assign(param + 3, param + 3 + epc_len);
    out.crc = static_cast<uint16_t>((param[3 + epc_len] << 8) | param[4 + epc_len]);
    return true;
}

//! @brief Initial value of the EPC Gen2 CRC-16
constexpr uint16_t GEN2_CRC16_INIT{0xFFFF};
//! @brief Polynomial of the EPC Gen2 CRC-16 (x^16 + x^12 + x^5 + 1)
constexpr uint16_t GEN2_CRC16_POLYNOMIAL{0x1021};
//! @brief Final xor value of the EPC Gen2 CRC-16
constexpr uint16_t GEN2_CRC16_XOROUT{0xFFFF};

/*!
  @brief Calculate the EPC Gen2 CRC-16
  @param data Data to calculate over
  @param len Length in bytes
  @return CRC-16
 */
inline uint16_t gen2_crc16(const uint8_t* data, const size_t len)
{
    m5::utility::CRC16 crc{GEN2_CRC16_INIT, GEN2_CRC16_POLYNOMIAL, false, false, GEN2_CRC16_XOROUT};
    return crc.range(data, len);
}

/*!
  @brief Verify the CRC-16 reported with a detected tag
  @param tag Tag to verify
  @return True if the CRC-16 recalculated from PC and EPC matches the reported one
  @note The Gen2 CRC-16 covers the PC followed by the EPC
 */
inline bool verify_tag_crc(const m5::uhf::Tag& tag)
{
    if (tag.epc.empty()) {
        return false;
    }
    std::vector<uint8_t> buf{};
    buf.reserve(tag.epc.size() + 2);
    buf.push_back(static_cast<uint8_t>(tag.pc >> 8));
    buf.push_back(static_cast<uint8_t>(tag.pc & 0xFF));
    buf.insert(buf.end(), tag.epc.begin(), tag.epc.end());
    return gen2_crc16(buf.data(), buf.size()) == tag.crc;
}

//! @brief Command code used by every failure notification
constexpr uint8_t COMMAND_ERROR{0xFF};

/*!
  @enum Error
  @brief Error codes carried by a failure notification
 */
enum class Error : uint8_t {
    ReadFail           = 0x09,  //!< Failed to read the tag's data memory area
    WriteFail          = 0x10,  //!< Failed to write the tag's data memory area
    KillFail           = 0x12,  //!< Failed to kill the tag
    LockFail           = 0x13,  //!< Failed to lock the tag's data memory area
    BlockPermalockFail = 0x14,  //!< BlockPermalock execution failed
    InventoryFail      = 0x15,  //!< No tag responded or a data CRC check error occurred
    AccessFail         = 0x16,  //!< Failed to access the tag
    CommandError       = 0x17,  //!< Command error in the command frame
    FHSSFail           = 0x20,  //!< Frequency hopping channel search timed out
};

/*!
  @brief Is the command code of a failure notification?
  @param command Command code of the received frame
  @return True if the frame reports a failure
 */
inline bool is_error_frame(const uint8_t command)
{
    return command == COMMAND_ERROR;
}

/*!
  @brief Does the error code only mean "no tag was found this round"?
  @param error_code Error code carried by the failure notification
  @return True if no tag responded
  @note During polling this is an expected condition, not a command failure
 */
inline bool is_no_tag(const uint8_t error_code)
{
    return error_code == static_cast<uint8_t>(Error::InventoryFail);
}

}  // namespace jrd4035
}  // namespace unit
}  // namespace m5
#endif
