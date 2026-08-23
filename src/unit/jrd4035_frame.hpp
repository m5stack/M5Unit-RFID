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
    // An EPC longer than the standard allows means the frame is not what it claims to be
    if (!out.epc.assign(param + 3, epc_len)) {
        return false;
    }
    out.rssi = static_cast<int8_t>(param[0]);
    out.pc   = static_cast<uint16_t>((param[1] << 8) | param[2]);
    out.crc  = static_cast<uint16_t>((param[3 + epc_len] << 8) | param[4 + epc_len]);
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
    // The Gen2 CRC-16 covers the PC followed by the EPC, so they are laid out contiguously
    uint8_t buf[2 + m5::uhf::EPC_MAX_BYTES]{};
    buf[0] = static_cast<uint8_t>(tag.pc >> 8);
    buf[1] = static_cast<uint8_t>(tag.pc & 0xFF);
    for (size_t i = 0; i < tag.epc.size; ++i) {
        buf[2 + i] = tag.epc[i];
    }
    return gen2_crc16(buf, tag.epc.size + 2U) == tag.crc;
}

//! @brief MemBank values of the Select parameter and of the read/write commands
constexpr uint8_t MEMBANK_RESERVED{0x00};
constexpr uint8_t MEMBANK_EPC{0x01};
constexpr uint8_t MEMBANK_TID{0x02};
constexpr uint8_t MEMBANK_USER{0x03};

//! @brief Truncate field of the Select parameter
constexpr uint8_t SELECT_TRUNCATE_OFF{0x00};
constexpr uint8_t SELECT_TRUNCATE_ON{0x80};

//! @brief Select modes of Set Select Mode (0x12)
constexpr uint8_t SELECT_MODE_ALWAYS{0x00};         //!< Select before every operation, inventory included
constexpr uint8_t SELECT_MODE_NEVER{0x01};          //!< Never send Select
constexpr uint8_t SELECT_MODE_NON_INVENTORY{0x02};  //!< Select before everything except inventory

//! @brief Longest write the module accepts, in 16-bit words
constexpr size_t WRITE_MAX_WORDS{32};
//! @brief Longest mask the Select parameter accepts, in bits
constexpr size_t SELECT_MASK_MAX_BITS{255};

/*!
  @brief Build the SelParam byte of Set Select Parameter
  @param target Target (3 bits)
  @param action Action (3 bits)
  @param membank Memory bank (2 bits)
  @return SelParam
 */
inline uint8_t select_parameter_byte(const uint8_t target, const uint8_t action, const uint8_t membank)
{
    return static_cast<uint8_t>(((target & 0x07) << 5) | ((action & 0x07) << 2) | (membank & 0x03));
}

/*!
  @brief Build the parameter of Set Select Parameter (0x0C)
  @param[out] out Parameter
  @param sel_param SelParam byte
  @param pointer_bits Start of the mask as a bit address inside the bank
  @param mask_length_bits Mask length in bits
  @param truncate SELECT_TRUNCATE_OFF or SELECT_TRUNCATE_ON
  @param mask Mask bytes
  @param mask_len Length of mask in bytes
  @return True if successful
  @note Reserved memory cannot be selected on: a tag ignores a Select that names it
 */
inline bool build_select_parameter(std::vector<uint8_t>& out, const uint8_t sel_param, const uint32_t pointer_bits,
                                   const uint8_t mask_length_bits, const uint8_t truncate, const uint8_t* mask,
                                   const size_t mask_len)
{
    if ((sel_param & 0x03) == MEMBANK_RESERVED) {
        return false;
    }
    if (mask_len == 0 || mask == nullptr || mask_length_bits == 0) {
        return false;
    }
    if (mask_length_bits > SELECT_MASK_MAX_BITS || (mask_length_bits + 7U) / 8U > mask_len) {
        return false;
    }

    out.clear();
    out.reserve(7 + mask_len);
    out.push_back(sel_param);
    out.push_back(static_cast<uint8_t>(pointer_bits >> 24));
    out.push_back(static_cast<uint8_t>(pointer_bits >> 16));
    out.push_back(static_cast<uint8_t>(pointer_bits >> 8));
    out.push_back(static_cast<uint8_t>(pointer_bits));
    out.push_back(mask_length_bits);
    out.push_back(truncate);
    out.insert(out.end(), mask, mask + mask_len);
    return true;
}

/*!
  @brief Build the parameter of Read Tag Memory Area (0x39)
  @param[out] out Parameter
  @param access_password Access password, 0 when the tag has none
  @param membank Memory bank
  @param word_address Start address in 16-bit words
  @param word_count Number of 16-bit words
  @return True if successful
  @note A word_count of zero would mean "to the end of the bank" in EPC Gen2, but whether the
  module honours that is unverified, so it is refused here
 */
inline bool build_read_tag_memory(std::vector<uint8_t>& out, const uint32_t access_password, const uint8_t membank,
                                  const uint16_t word_address, const uint16_t word_count)
{
    if (word_count == 0) {
        return false;
    }
    out.clear();
    out.reserve(9);
    out.push_back(static_cast<uint8_t>(access_password >> 24));
    out.push_back(static_cast<uint8_t>(access_password >> 16));
    out.push_back(static_cast<uint8_t>(access_password >> 8));
    out.push_back(static_cast<uint8_t>(access_password));
    out.push_back(membank);
    out.push_back(static_cast<uint8_t>(word_address >> 8));
    out.push_back(static_cast<uint8_t>(word_address));
    out.push_back(static_cast<uint8_t>(word_count >> 8));
    out.push_back(static_cast<uint8_t>(word_count));
    return true;
}

/*!
  @brief Build the parameter of Write Tag Memory Area (0x49)
  @param[out] out Parameter
  @param access_password Access password, 0 when the tag has none
  @param membank Memory bank
  @param word_address Start address in 16-bit words
  @param data Bytes to write
  @param len Length of data, which must be even and at most 64
  @return True if successful
 */
inline bool build_write_tag_memory(std::vector<uint8_t>& out, const uint32_t access_password, const uint8_t membank,
                                   const uint16_t word_address, const uint8_t* data, const size_t len)
{
    if (data == nullptr || len == 0 || (len % 2) != 0 || len / 2 > WRITE_MAX_WORDS) {
        return false;
    }
    const uint16_t word_count = static_cast<uint16_t>(len / 2);
    if (!build_read_tag_memory(out, access_password, membank, word_address, word_count)) {
        return false;
    }
    out.insert(out.end(), data, data + len);
    return true;
}

/*!
  @brief Build the parameter of Lock (0x82)
  @param[out] out Parameter
  @param access_password Access password
  @param payload 20-bit lock payload, see m5::uhf::buildLockPayload
  @return True if successful
 */
inline bool build_lock_tag(std::vector<uint8_t>& out, const uint32_t access_password, const uint32_t payload)
{
    if (payload > 0x000FFFFFU) {
        return false;
    }
    out.clear();
    out.reserve(7);
    out.push_back(static_cast<uint8_t>(access_password >> 24));
    out.push_back(static_cast<uint8_t>(access_password >> 16));
    out.push_back(static_cast<uint8_t>(access_password >> 8));
    out.push_back(static_cast<uint8_t>(access_password));
    out.push_back(static_cast<uint8_t>(payload >> 16));
    out.push_back(static_cast<uint8_t>(payload >> 8));
    out.push_back(static_cast<uint8_t>(payload));
    return true;
}

/*!
  @brief Build the parameter of Kill (0x65)
  @param[out] out Parameter
  @param kill_password Kill password
  @return True if successful
  @note A tag whose kill password is zero refuses to be killed, so zero is refused here
 */
inline bool build_kill_tag(std::vector<uint8_t>& out, const uint32_t kill_password)
{
    if (kill_password == 0) {
        return false;
    }
    out.clear();
    out.reserve(4);
    out.push_back(static_cast<uint8_t>(kill_password >> 24));
    out.push_back(static_cast<uint8_t>(kill_password >> 16));
    out.push_back(static_cast<uint8_t>(kill_password >> 8));
    out.push_back(static_cast<uint8_t>(kill_password));
    return true;
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
