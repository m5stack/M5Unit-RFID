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

namespace {
// How often the polling command is reissued. Multiple polling runs a finite number of rounds and
// then stops, so the command is renewed on a timer rather than after the stream falls silent:
// waiting for silence would leave a gap in detection, and the module can also go quiet for a
// while with rounds still to run. The interval stays short enough that the count never runs out
// between renewals, whatever the round rate happens to be.
constexpr uint32_t POLLING_REISSUE_INTERVAL_MS{500};
}  // namespace

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
    _polling           = true;
    _polling_count     = count;
    _last_frame_at     = m5::utility::millis();
    _polling_issued_at = _last_frame_at;
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

bool UHFRFIDComponent::reject_while_polling(const char* what) const
{
    if (_polling) {
        M5_LIB_LOGW("%s is unreliable while polling; call stopPolling() first", what);
        return true;
    }
    return false;
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
    if (m5::utility::millis() - _polling_issued_at < POLLING_REISSUE_INTERVAL_MS) {
        return;
    }
    if (start_polling_command(_polling_count)) {
        _polling_issued_at = m5::utility::millis();
    }
}

}  // namespace unit
}  // namespace m5
