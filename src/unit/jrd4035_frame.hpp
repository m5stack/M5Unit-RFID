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

namespace m5 {
namespace unit {
/*!
  @namespace jrd4035
  @brief JRD-4035 (magicRF M100) specific definitions
 */
namespace jrd4035 {

//! @brief Frame header of the JRD-4035 / JRD-100
constexpr uint8_t FRAME_HEADER = 0xBB;
//! @brief Frame end of the JRD-4035 / JRD-100
constexpr uint8_t FRAME_END = 0x7E;

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
constexpr uint16_t MAX_PARAMETER_LENGTH = 512;

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

}  // namespace jrd4035
}  // namespace unit
}  // namespace m5
#endif
