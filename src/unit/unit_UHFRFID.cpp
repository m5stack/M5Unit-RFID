/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_UHFRFID.cpp
  @brief Base class for UHF-RFID reader units
*/
#include "unit_UHFRFID.hpp"

#include <M5Utility.hpp>

namespace m5 {
namespace unit {

bool UHFRFIDComponent::begin()
{
    _tags.reset(new m5::container::CircularBuffer<m5::uhf::Tag>(_cfg.tag_queue_size));
    _dropped = 0;
    _polling = false;
    return true;
}

void UHFRFIDComponent::update(const bool force)
{
    (void)force;
    pump(0);
    reissue_polling_if_needed();
}

bool UHFRFIDComponent::startPolling(const uint16_t count)
{
    if (!start_polling_command(count)) {
        M5_LIB_LOGE("Failed to start polling");
        return false;
    }
    _polling       = true;
    _polling_count = count;
    _last_frame_at = m5::utility::millis();
    return true;
}

bool UHFRFIDComponent::stopPolling()
{
    _polling = false;
    if (!stop_polling_command()) {
        M5_LIB_LOGE("Failed to stop polling");
        return false;
    }
    return true;
}

void UHFRFIDComponent::push_tag(const m5::uhf::Tag& tag)
{
    if (!_tags) {
        return;
    }
    if (_tags->full()) {
        _tags->pop_front();
        ++_dropped;
    }
    _tags->push_back(tag);
}

void UHFRFIDComponent::note_frame_arrival()
{
    _last_frame_at = m5::utility::millis();
}

void UHFRFIDComponent::reissue_polling_if_needed()
{
    if (!_polling) {
        return;
    }
    // While polling the module keeps emitting frames even when no tag is present
    // (Inventory Fail 0x15 per round). Silence therefore means the polling count is exhausted.
    if (m5::utility::millis() - _last_frame_at < _reissue_threshold_ms) {
        return;
    }
    M5_LIB_LOGD("Reissue polling");
    if (start_polling_command(_polling_count)) {
        _last_frame_at = m5::utility::millis();
    }
}

}  // namespace unit
}  // namespace m5
