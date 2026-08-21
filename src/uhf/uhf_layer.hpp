/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file uhf_layer.hpp
  @brief EPC Gen2 semantics layer for UHF-RFID reader units
*/
#ifndef M5_UNIT_RFID_UHF_UHF_LAYER_HPP
#define M5_UNIT_RFID_UHF_UHF_LAYER_HPP

#include "uhf.hpp"
#include "unit/unit_UHFRFID.hpp"

namespace m5 {
namespace uhf {

/*!
  @class UHFLayer
  @brief EPC Gen2 semantics layer for UHF-RFID reader units
  @details Holds semantics defined by the EPC Gen2 standard, which stay the same across
  reader chips. The frame level belongs to the unit class.
 */
class UHFLayer {
public:
    explicit UHFLayer(m5::unit::UHFRFIDComponent& u) : _u{u}
    {
    }
    UHFLayer(const UHFLayer&)            = delete;
    UHFLayer& operator=(const UHFLayer&) = delete;

    /*!
      @brief Detect tags
      @param[out] tags Detected tags, deduplicated by EPC
      @param timeout_ms Timeout in milliseconds
      @return True if at least one tag was detected
      @note Starts polling internally and restores the previous polling state on return
     */
    bool detect(std::vector<Tag>& tags, const uint32_t timeout_ms = 1000U);

private:
    m5::unit::UHFRFIDComponent& _u;
};

}  // namespace uhf
}  // namespace m5
#endif
