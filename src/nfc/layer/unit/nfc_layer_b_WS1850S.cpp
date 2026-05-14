/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file nfc_layer_b_WS1850S.cpp
  @brief WS1850S NFC-B adapter for common layer
*/
#include <nfc/layer/b/nfc_layer_b.hpp>
#include <nfc/layer/ndef_layer.hpp>
#include "unit/unit_WS1850S.hpp"
#include <M5Utility.hpp>

using namespace m5::unit;
using namespace m5::unit::mfrc522;
using namespace m5::nfc::b;

namespace m5 {
namespace nfc {

//
struct AdapterWS1850SForB final : NFCLayerB::Adapter {
    explicit AdapterWS1850SForB(UnitWS1850S& ref) : _u{ref}
    {
    }

    inline virtual uint16_t max_fifo_depth() const override
    {
        return m5::unit::mfrc522::MAX_FIFO_DEPTH;
    }

    bool transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                    const uint32_t timeout_ms) override
    {
        return _u.nfcbTransceive(rx, rx_len, tx, tx_len, timeout_ms);
    }

    bool transmit(const uint8_t* tx, const uint16_t tx_len, const uint32_t timeout_ms) override
    {
        // MFRC522-class chips do not have a standalone transmit; reuse transceive (rx discarded).
        uint8_t dummy[m5::unit::mfrc522::MAX_FIFO_DEPTH]{};
        uint16_t dlen = sizeof(dummy);
        return _u.nfcbTransceive(dummy, dlen, tx, tx_len, timeout_ms);
    }

    bool receive(uint8_t* /*rx*/, uint16_t& /*rx_len*/, const uint32_t /*timeout_ms*/) override
    {
        // Standalone receive is not supported on MFRC522-class chips (Transceive command based).
        return false;
    }

    UnitWS1850S& _u;
};

//
namespace {
std::unique_ptr<NFCLayerB::Adapter> make_ws1850s_b_adapter(UnitWS1850S& u)
{
    return std::unique_ptr<NFCLayerB::Adapter>(new AdapterWS1850SForB(u));
}
}  // namespace

NFCLayerB::NFCLayerB(UnitWS1850S& u) : _ndef{*this}, _isoDEP{*this}, _impl(make_ws1850s_b_adapter(u))
{
}

}  // namespace nfc
}  // namespace m5
