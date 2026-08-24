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

#include <algorithm>

#include "unit/m100_frame.hpp"

namespace m5 {
namespace uhf {

namespace {
//! @brief Bit address of the first EPC bit, past the CRC-16 and the PC (EPC Gen2 6.3.2.1.2)
constexpr uint32_t EPC_MASK_POINTER_BITS{0x20};
//! @brief Bit address of the first TID bit
constexpr uint32_t TID_MASK_POINTER_BITS{0x00};
//! @brief Word holding the first half of the EPC, past the CRC-16 and the PC
constexpr uint16_t EPC_FIRST_WORD{2};
//! @brief Words of the TID that sit at a fixed address whatever the tag is
constexpr uint16_t TID_FIXED_WORDS{2};
}  // namespace

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

void UHFLayer::pause_polling()
{
    if (_u.inPolling()) {
        _u.stopPolling();
        _resume_polling = true;
    }
}

void UHFLayer::resume_polling()
{
    if (_resume_polling) {
        _resume_polling = false;
        if (!_u.startPolling(_u.config().polling_count)) {
            M5_LIB_LOGE("Failed to resume polling");
        }
    }
}

bool UHFLayer::apply_selection(const Bank bank, const uint32_t pointer_bits, const uint8_t* mask, const size_t mask_len,
                               const uint32_t access_password, const bool verify)
{
    if (mask == nullptr || mask_len == 0) {
        M5_LIB_LOGE("An empty mask would match every tag, not one");
        return false;
    }
    pause_polling();

    if (!_u.write_select_parameter(bank, pointer_bits, mask, mask_len)) {
        resume_polling();
        return false;
    }
    // Storing the parameter is documented to switch the module over on its own, but saying so
    // explicitly is what makes the state the same whether or not a deselect() came before
    if (!_u.write_select_enabled(true)) {
        resume_polling();
        return false;
    }

    _access_password = access_password;
    _mask_bank       = bank;
    _has_selection   = true;

    if (verify && !verify_selection()) {
        _has_selection = false;
        _u.write_select_enabled(false);
        resume_polling();
        return false;
    }
    return true;
}

bool UHFLayer::verify_selection()
{
    // One word of the EPC bank, which every tag has. Reading it goes down the same path as the
    // Read and Write that follow, so it proves the selection the way the caller will use it
    std::vector<uint8_t> word{};
    if (!_u.read_tag_memory(word, Bank::Epc, EPC_FIRST_WORD, 1, _access_password)) {
        M5_LIB_LOGW("The selected tag did not answer");
        return false;
    }
    return true;
}

bool UHFLayer::select(const Tag& tag, const uint32_t access_password, const bool verify)
{
    if (tag.epc.empty()) {
        M5_LIB_LOGE("The tag carries no EPC to build a mask from");
        return false;
    }
    if (!apply_selection(Bank::Epc, EPC_MASK_POINTER_BITS, tag.epc.begin(), tag.epc.size, access_password, verify)) {
        return false;
    }
    _selected = tag;
    return true;
}

bool UHFLayer::select(const Epc& epc, const uint32_t access_password, const bool verify)
{
    if (epc.empty()) {
        M5_LIB_LOGE("The tag carries no EPC to build a mask from");
        return false;
    }
    if (!apply_selection(Bank::Epc, EPC_MASK_POINTER_BITS, epc.begin(), epc.size, access_password, verify)) {
        return false;
    }
    _selected     = Tag{};
    _selected.epc = epc;
    return true;
}

bool UHFLayer::select(const Tid& tid, const uint32_t access_password, const bool verify)
{
    if (tid.empty()) {
        M5_LIB_LOGE("The tag carries no TID to build a mask from");
        return false;
    }
    if (!apply_selection(Bank::Tid, TID_MASK_POINTER_BITS, tid.begin(), tid.size, access_password, verify)) {
        return false;
    }
    _selected     = Tag{};
    _selected.tid = tid;
    decodeTid(_selected, tid.begin(), tid.size);
    return true;
}

bool UHFLayer::deselect()
{
    _has_selection      = false;
    _selected           = Tag{};
    _access_password    = 0;
    const bool switched = _u.write_select_enabled(false);
    resume_polling();
    return switched;
}

bool UHFLayer::identify(Tag& tag)
{
    if (!_has_selection) {
        M5_LIB_LOGE("identify needs a tag to have been selected");
        return false;
    }
    pause_polling();

    // The two fixed words name the chip and say whether an XTID follows them
    std::vector<uint8_t> tid{};
    if (!_u.read_tag_memory(tid, Bank::Tid, 0, TID_FIXED_WORDS, _access_password)) {
        return false;
    }
    Tag identified = _selected;
    if (!decodeTid(identified, tid.data(), tid.size())) {
        M5_LIB_LOGE("Not an EPCglobal TID: %02X", tid.empty() ? 0 : tid[0]);
        return false;
    }

    if (identified.has_xtid) {
        // Only the header says how long the rest is, so it has to be read before the rest can be
        std::vector<uint8_t> header{};
        if (!_u.read_tag_memory(header, Bank::Tid, TID_FIXED_WORDS, 1, _access_password)) {
            return false;
        }
        tid.insert(tid.end(), header.begin(), header.end());

        const size_t total = xtidTotalWords(static_cast<uint16_t>((header[0] << 8) | header[1]));
        if (total * 2 > TID_MAX_BYTES) {
            M5_LIB_LOGE("XTID of %zu words does not fit; the header reads %02X%02X", total, header[0], header[1]);
            return false;
        }
        if (total > XTID_FIXED_WORDS) {
            std::vector<uint8_t> rest{};
            if (!_u.read_tag_memory(rest, Bank::Tid, XTID_FIXED_WORDS, static_cast<uint16_t>(total - XTID_FIXED_WORDS),
                                    _access_password)) {
                return false;
            }
            tid.insert(tid.end(), rest.begin(), rest.end());
        }
        if (!decodeTid(identified, tid.data(), tid.size())) {
            return false;
        }
    }

    identified.tid.assign(tid.data(), tid.size());
    tag       = identified;
    _selected = identified;
    return true;
}

bool UHFLayer::readBank(std::vector<uint8_t>& out, const Bank bank, const uint16_t word_address,
                        const uint16_t word_count)
{
    out.clear();
    if (!_has_selection) {
        M5_LIB_LOGE("readBank needs a tag to have been selected");
        return false;
    }
    pause_polling();
    return _u.read_tag_memory(out, bank, word_address, word_count, _access_password);
}

bool UHFLayer::writeBank(const Bank bank, const uint16_t word_address, const std::vector<uint8_t>& data)
{
    if (!_has_selection) {
        M5_LIB_LOGE("writeBank needs a tag to have been selected");
        return false;
    }
    if (data.empty() || (data.size() % 2) != 0) {
        M5_LIB_LOGE("A write of %u bytes is not a whole number of words", (unsigned)data.size());
        return false;
    }
    pause_polling();

    // One command carries at most WRITE_MAX_WORDS, which is a limit of the reader's command
    // frame rather than of the tag or of EPC Gen2. A caller writing a bank larger than that
    // should not have to know it, so the write is split here. The module is already looping
    // internally: Gen2 writes one word at a time and it is the module that batches them
    size_t written = 0;
    while (written < data.size()) {
        const size_t chunk = std::min(data.size() - written, m5::unit::m100::WRITE_MAX_WORDS * 2);
        const uint16_t at  = static_cast<uint16_t>(word_address + written / 2);
        if (!_u.write_tag_memory(bank, at, data.data() + written, chunk, _access_password)) {
            // Say how far it got: whatever came before this is already on the tag
            M5_LIB_LOGE("Wrote %u of %u words before failing at word %u", (unsigned)(written / 2),
                        (unsigned)(data.size() / 2), at);
            return false;
        }
        written += chunk;
    }

    // An EPC mask matches on the bytes that were just replaced, so it no longer picks this tag
    // out. Dropping the selection turns a silent mis-address into an explicit "select again"
    if (bank == Bank::Epc && _mask_bank == Bank::Epc) {
        M5_LIB_LOGW("The EPC the selection matched on has been rewritten; select the tag again");
        deselect();
    }
    return true;
}

bool UHFLayer::lock(const std::vector<LockSetting>& settings, const bool allow_permanent)
{
    if (!_has_selection) {
        M5_LIB_LOGE("lock needs a tag to have been selected");
        return false;
    }
    if (settings.empty()) {
        M5_LIB_LOGE("lock was given nothing to change");
        return false;
    }
    if (!allow_permanent) {
        for (auto&& setting : settings) {
            if (isPermanent(setting.action)) {
                M5_LIB_LOGE("A permanent lock cannot be undone; pass allow_permanent to mean it");
                return false;
            }
        }
    }
    pause_polling();
    return _u.lock_tag_memory(buildLockPayload(settings.data(), settings.size()), _access_password);
}

bool UHFLayer::kill(const Tag& tag, const uint32_t kill_password)
{
    if (!_has_selection) {
        M5_LIB_LOGE("kill needs a tag to have been selected");
        return false;
    }
    if (kill_password == 0) {
        // A tag whose kill password is zero refuses to be killed (Gen2 v2.1 6.3.2.12.3.4), so
        // sending zero can only fail. Refusing it here keeps a forgotten password from reading
        // as a tag that cannot be killed
        M5_LIB_LOGE("A kill password of zero is refused by every tag");
        return false;
    }
    // Killing the wrong tag cannot be undone, so the caller has to name the tag it means and it
    // has to be the one the mask picks out
    const bool same_epc = !tag.epc.empty() && !_selected.epc.empty() && tag.epc == _selected.epc;
    const bool same_tid = !tag.tid.empty() && !_selected.tid.empty() && tag.tid == _selected.tid;
    if (!same_epc && !same_tid) {
        M5_LIB_LOGE("kill was given a tag that is not the selected one");
        return false;
    }

    pause_polling();
    if (!_u.kill_tag(kill_password)) {
        return false;
    }
    // Nothing answers to the mask any more
    _has_selection = false;
    _selected      = Tag{};
    _u.write_select_enabled(false);
    resume_polling();
    return true;
}

}  // namespace uhf
}  // namespace m5
