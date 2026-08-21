/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file uhf_layer.cpp
  @brief EPC Gen2 semantics layer for UHF-RFID reader units
*/
#include "uhf_layer.hpp"

#include <M5Utility.hpp>

namespace m5 {
namespace uhf {

bool UHFLayer::detect(std::vector<Tag>& tags, const uint32_t timeout_ms)
{
    tags.clear();

    const bool was_polling = _u.inPolling();
    if (!was_polling && !_u.startPolling(_u.config().polling_count)) {
        M5_LIB_LOGE("Failed to startPolling");
        return false;
    }
    // Discard notifications that arrived before this call
    _u.flush();

    const unsigned long expire_at = m5::utility::millis() + timeout_ms;
    while (m5::utility::millis() < expire_at) {
        _u.update();
        while (_u.available()) {
            append_unique(tags, _u.oldest());
            _u.discard();
        }
        m5::utility::delay(1);
    }

    if (!was_polling) {
        _u.stopPolling();
    }
    return !tags.empty();
}

}  // namespace uhf
}  // namespace m5
