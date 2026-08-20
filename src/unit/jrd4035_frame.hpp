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

}  // namespace jrd4035
}  // namespace unit
}  // namespace m5
#endif
