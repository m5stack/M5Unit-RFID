/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file nfc_layer_a_MFRC522.cpp
  @brief MFRC522 adapter for common layer
*/
#include <nfc/layer/nfc_layer_a.hpp>
#include "unit/unit_MFRC522.hpp"
#include "unit/unit_WS1850S.hpp"
#include <M5Utility.hpp>

using namespace m5::unit;
using namespace m5::unit::mfrc522;
using namespace m5::nfc::a;
using namespace m5::nfc::a::mifare;
using namespace m5::nfc::a::mifare::classic;

namespace m5 {
namespace unit {
namespace nfc {

//
struct AdapterMFRC522 final : NFCLayerA::Adapter {
    explicit AdapterMFRC522(UnitMFRC522& ref) : _u{ref}
    {
    }

    virtual bool request(uint16_t& atqa) override;
    virtual bool wakeup(uint16_t& atqa) override;

    virtual bool select(m5::nfc::a::UID& uid) override;
    virtual bool activate(const m5::nfc::a::UID& uid) override;
    virtual bool deactivate() override;

    virtual bool nfca_transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                                 const uint32_t timeout_ms) override;
    virtual bool nfca_read_block(uint8_t* rx, uint16_t& rx_len, const uint16_t addr) override;
    virtual bool nfca_write_block(const uint16_t addr, const uint8_t* tx, const uint16_t tx_len) override;

    virtual bool mifare_classic_authenticate(const bool auth_a, const m5::nfc::a::UID& uid, const uint8_t block,
                                             const m5::nfc::a::mifare::classic::Key& key) override;
    inline virtual bool mifare_classic_read_block(uint8_t* rx, uint16_t& rx_len, const uint16_t addr) override
    {
        return nfca_read_block(rx, rx_len, addr);
    }
    inline virtual bool mifare_classic_write_block(const uint16_t addr, const uint8_t* tx,
                                                   const uint16_t tx_len) override
    {
        return nfca_write_block(addr, tx, tx_len);
    }

    virtual bool ntag_get_version(uint8_t info[10]) override;

    UnitMFRC522& _u;
};

bool AdapterMFRC522::request(uint16_t& atqa)
{
    return _u.request(atqa);
}

bool AdapterMFRC522::wakeup(uint16_t& atqa)
{
    return _u.wakeup(atqa);
}

bool AdapterMFRC522::select(m5::nfc::a::UID& uid)
{
    uint8_t lv{1};  // Cascade level 1-3
    bool completed{};
    uid.clear();
    do {
        if (!_u.selectWithAnticollision(completed, uid, lv)) {
            return false;
        }
    } while (!completed && lv++ < 4);
    return completed;
}

bool AdapterMFRC522::activate(const m5::nfc::a::UID& uid)
{
    uint16_t atqa{};
    return _u.wakeup(atqa) && _u.select(uid);
}

bool AdapterMFRC522::deactivate()
{
    return _u.hlt() && _u.mifareClassicStopCrypto1();
}

bool AdapterMFRC522::nfca_read_block(uint8_t* rx, uint16_t& rx_len, const uint16_t addr)
{
    uint8_t tmp[rx_len + 2 /* CRC */]{};
    uint16_t tmp_len = rx_len + 2;
    if (_u.readBlock(tmp, tmp_len, addr) && tmp_len > 2) {
        memcpy(rx, tmp, std::min<uint16_t>(rx_len, tmp_len - 2));
        rx_len = tmp_len - 2;  // CRC
        return true;
    }
    return false;
}

bool AdapterMFRC522::nfca_write_block(const uint16_t addr, const uint8_t* tx, const uint16_t tx_len)
{
    //    return _u.writeBlock(addr, tx, tx_len);
    return false;
}

bool AdapterMFRC522::nfca_transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                                     const uint32_t timeout_ms)
{
    //    return _u.transceive(rx, rx_len, tx, tx_len, timeout_ms);
    return false;
}

bool AdapterMFRC522::mifare_classic_authenticate(const bool auth_a, const UID& uid, const uint8_t block, const Key& key)
{
    return auth_a ? _u.mifareClassicAuthenticateA(uid, block, key) : _u.mifareClassicAuthenticateB(uid, block, key);
}

bool AdapterMFRC522::ntag_get_version(uint8_t info[10])
{
    return _u.ntagGetVersion(info);
}

//
namespace {
std::unique_ptr<NFCLayerA::Adapter> make_mfrc522_adapter(UnitMFRC522& u)
{
    return std::unique_ptr<NFCLayerA::Adapter>(new AdapterMFRC522(u));
}
}  // namespace

NFCLayerA::NFCLayerA(UnitMFRC522& u) : _impl(make_mfrc522_adapter(u))
{
}

NFCLayerA::NFCLayerA(UnitWS1850S& u) : _impl(make_mfrc522_adapter(static_cast<UnitMFRC522&>(u)))
{
}

}  // namespace nfc
}  // namespace unit
}  // namespace m5
