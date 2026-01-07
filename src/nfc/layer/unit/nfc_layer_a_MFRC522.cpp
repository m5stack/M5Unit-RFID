/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file nfc_layer_a_MFRC522.cpp
  @brief MFRC522 adapter for common layer
*/
#include <nfc/layer/a/nfc_layer_a.hpp>
#include <nfc/layer/ndef_layer.hpp>
#include "unit/unit_MFRC522.hpp"
#include "unit/unit_WS1850S.hpp"
#include <M5Utility.hpp>

using namespace m5::unit;
using namespace m5::unit::mfrc522;
using namespace m5::nfc::a;
using namespace m5::nfc::a::mifare;
using namespace m5::nfc::a::mifare::classic;

namespace m5 {
namespace nfc {

//
struct AdapterMFRC522 final : NFCLayerA::Adapter {
    explicit AdapterMFRC522(UnitMFRC522& ref) : _u{ref}
    {
    }

    inline virtual uint16_t max_fifo_depth() override
    {
        return m5::unit::mfrc522::MAX_FIFO_DEPTH;
    }

    virtual bool transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                            const uint32_t timeout_ms) override;
    //    virtual bool mifare_transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
    //                                   const uint32_t timeout_ms) override;
    //    virtual bool mifare_transmit(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
    //                                 const uint32_t timeout_ms, const bool waitTXE) override;

    virtual bool request(uint16_t& atqa) override;
    virtual bool wakeup(uint16_t& atqa) override;

    virtual bool select(m5::nfc::a::PICC& picc) override;
    virtual bool activate(const m5::nfc::a::PICC& picc) override;
    virtual bool hlt() override;

    virtual bool nfca_read_block(uint8_t rx[16], const uint8_t addr) override;         // READ
    virtual bool nfca_write_block(const uint8_t addr, const uint8_t tx[16]) override;  // WRITE_BLOCK
    virtual bool nfca_write_page(const uint8_t addr, const uint8_t tx[4]) override;    // WRITE_PAGE

    virtual bool mifare_classic_authenticate(const bool auth_a, const m5::nfc::a::PICC& picc, const uint8_t block,
                                             const m5::nfc::a::mifare::classic::Key& key) override;
    virtual bool mifare_classic_value_block(const m5::nfc::a::Command cmd, const uint8_t block,
                                            const uint32_t arg = 0) override;

    virtual bool ntag_read_page(uint8_t* rx, uint16_t& rx_len, const uint8_t spage,
                                const uint8_t epage) override;  // FAST_READ

    UnitMFRC522& _u;
};

bool AdapterMFRC522::transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                                const uint32_t timeout_ms)
{
    return _u.nfcaTransceive(rx, rx_len, tx, tx_len, timeout_ms);
}

#if 0
bool AdapterMFRC522::mifare_transceive(uint8_t* rx, uint16_t& rx_len, const uint8_t* tx, const uint16_t tx_len,
                                       const uint32_t timeout_ms){


    
    

}
#endif

bool AdapterMFRC522::request(uint16_t& atqa)
{
    return _u.request(atqa);
}

bool AdapterMFRC522::wakeup(uint16_t& atqa)
{
    return _u.wakeup(atqa);
}

bool AdapterMFRC522::select(m5::nfc::a::PICC& picc)
{
    uint8_t lv{1};  // Cascade level 1-3
    bool completed{};
    do {
        if (!_u.selectWithAnticollision(completed, picc, lv)) {
            return false;
        }
    } while (!completed && lv++ < 4);
    return completed;
}

bool AdapterMFRC522::activate(const m5::nfc::a::PICC& picc)
{
    return _u.select(picc);
}

bool AdapterMFRC522::hlt()
{
    return _u.hlt() && _u.mifareClassicStopCrypto1();
}

bool AdapterMFRC522::nfca_read_block(uint8_t rx[16], const uint8_t addr)
{
    return _u.readBlock(rx, addr);

#if 0    
    uint8_t tmp[rx_len + 2 /* CRC */]{};
    uint16_t tmp_len = rx_len + 2;
    if (_u.readBlock(tmp, tmp_len, addr) && tmp_len > 2) {
        memcpy(rx, tmp, std::min<uint16_t>(rx_len, tmp_len - 2));
        rx_len = tmp_len - 2;  // CRC
        return true;
    }
    return false;
#endif
}

bool AdapterMFRC522::nfca_write_block(const uint8_t addr, const uint8_t tx[16])
{
    return _u.writeBlock(addr, tx);
}

bool AdapterMFRC522::nfca_write_page(const uint8_t addr, const uint8_t tx[4])
{
    return _u.writePage(addr, tx);
}

bool AdapterMFRC522::mifare_classic_authenticate(const bool auth_a, const PICC& picc, const uint8_t block,
                                                 const Key& key)
{
    return auth_a ? _u.mifareClassicAuthenticateA(picc, block, key) : _u.mifareClassicAuthenticateB(picc, block, key);
}

bool AdapterMFRC522::mifare_classic_value_block(const m5::nfc::a::Command cmd, const uint8_t block, const uint32_t arg)
{
    return _u.mifareClassicValueBlock(cmd, block, arg);
}

bool AdapterMFRC522::ntag_read_page(uint8_t* rx, uint16_t& rx_len, const uint8_t spage, const uint8_t epage)
{
    return _u.ntagReadPage(rx, rx_len, spage, epage);
}

//
namespace {
std::unique_ptr<NFCLayerA::Adapter> make_mfrc522_adapter(UnitMFRC522& u)
{
    return std::unique_ptr<NFCLayerA::Adapter>(new AdapterMFRC522(u));
}
}  // namespace

NFCLayerA::NFCLayerA(UnitMFRC522& u) : _ndef{*this}, _isoDEP{*this}, _impl(make_mfrc522_adapter(u))
{
}

NFCLayerA::NFCLayerA(UnitWS1850S& u)
    : _ndef{*this}, _isoDEP{*this}, _impl(make_mfrc522_adapter(static_cast<UnitMFRC522&>(u)))
{
}

}  // namespace nfc
}  // namespace m5
