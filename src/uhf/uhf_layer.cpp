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
#include <cstdio>
#include <cstring>

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
//! @brief QT control word, bit 15: the tag reduces its range once open or secured
constexpr uint16_t QT_SHORT_RANGE{0x8000};
//! @brief QT control word, bit 14: the tag shows its public memory map
constexpr uint16_t QT_PUBLIC_MEMORY{0x4000};
//! @brief Words of the TID that sit at a fixed address whatever the tag is
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

bool UHFLayer::pause_polling()
{
    if (!_u.inPolling()) {
        return true;
    }
    // Resuming is wanted either way: the caller asked for polling and deselect() is what takes
    // that back, whether or not the module heard the stop
    _resume_polling = true;
    if (!_u.stopPolling()) {
        M5_LIB_LOGE("The module is still running inventory rounds; tag operations will fail");
        return false;
    }
    return true;
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

bool UHFLayer::detect(Tag& tag, const uint32_t timeout_ms)
{
    tag = Tag{};

    const bool was_polling = _u.inPolling();
    if (!was_polling && !_u.startPolling(_u.config().polling_count)) {
        M5_LIB_LOGE("Failed to startPolling");
        return false;
    }
    // Discard notifications that arrived before this call
    _u.flush();

    const unsigned long expire_at = m5::utility::millis() + timeout_ms;
    while (m5::utility::millis() < expire_at && !tag.valid()) {
        _u.update();
        if (_u.available()) {
            tag = _u.oldest();
            _u.discard();
        } else {
            m5::utility::delay(1);
        }
    }

    if (!was_polling) {
        _u.stopPolling();
    }
    return tag.valid();
}

bool UHFLayer::apply_selection(const Bank bank, const uint32_t pointer_bits, const uint8_t* mask, const size_t mask_len,
                               const uint32_t access_password, const bool verify)
{
    if (mask == nullptr || mask_len == 0) {
        M5_LIB_LOGE("An empty mask would match every tag, not one");
        return false;
    }
    if (!pause_polling()) {
        return false;
    }

    if (!_u.writeSelectParameter(bank, pointer_bits, mask, mask_len)) {
        resume_polling();
        return false;
    }
    // Storing the parameter is documented to switch the module over on its own, but saying so
    // explicitly is what makes the state the same whether or not a deselect() came before
    if (!_u.writeSelectEnabled(true)) {
        resume_polling();
        return false;
    }

    _access_password = access_password;
    _mask_bank       = bank;
    _has_selection   = true;

    if (verify && !verify_selection()) {
        _has_selection = false;
        _u.writeSelectEnabled(false);
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
    if (!_u.readTagMemory(word, Bank::Epc, EPC_FIRST_WORD, 1, _access_password)) {
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
    // A tag that sends an XPC sends it ahead of the EPC, so what detection reported as the EPC
    // begins with a word or two that lives at 210h rather than at 20h (EPC Gen2 v2.1
    // 6.3.2.1.2.2). A mask laid over the EPC from those bytes matches nothing, and none of the
    // tags this was tried on carry one, so it is refused rather than guessed at
    if (pcXPCIndicator(tag.pc)) {
        M5_LIB_LOGE("This tag sends an XPC, and what was reported as its EPC begins with it");
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
    // The first three words name the chip, and every tag of that model carries the same bytes
    // there. A mask that reaches no further would address all of them at once
    if (!tidTellsTagsApart(tid.begin(), tid.size)) {
        M5_LIB_LOGE("A TID mask of %u bytes cannot tell one tag of this chip from another", (unsigned)tid.size);
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
    const bool switched = _u.writeSelectEnabled(false);
    resume_polling();
    return switched;
}

bool UHFLayer::identify(Tag& tag)
{
    if (!_has_selection) {
        M5_LIB_LOGE("identify needs a tag to have been selected");
        return false;
    }
    if (!pause_polling()) {
        return false;
    }

    // A TID says its own length as it is read: the fixed words say whether an XTID follows,
    // and the XTID header says how long the rest is. tidReadPlan walks that
    std::vector<uint8_t> tid{};
    Tag identified = _selected;
    for (;;) {
        const TidReadPlan plan = tidReadPlan(tid.data(), tid.size());
        if (!plan.fits) {
            M5_LIB_LOGE("XTID of more than %zu bytes cannot be kept", TID_MAX_BYTES);
            return false;
        }
        if (plan.words == 0) {
            break;
        }
        std::vector<uint8_t> part{};
        if (!_u.readTagMemory(part, Bank::Tid, plan.word_address, plan.words, _access_password)) {
            // A tag can promise more TID than it has: an Impinj Monza 4QT showing its public
            // map still says it has an extended TID, and answers with a memory overrun when
            // asked for one. What was read before that already names the chip, so it is kept
            if (tid.empty()) {
                return false;
            }
            M5_LIB_LOGW("The tag has less TID than it says; keeping the %zu bytes that read", tid.size());
            break;
        }
        // A short answer would leave the plan asking for the same words again, so the loop
        // insists on getting what it asked for
        if (part.size() != static_cast<size_t>(plan.words) * 2) {
            M5_LIB_LOGE("Asked for %u words of TID and got %zu bytes", plan.words, part.size());
            return false;
        }
        tid.insert(tid.end(), part.begin(), part.end());
        if (!decodeTid(identified, tid.data(), tid.size())) {
            M5_LIB_LOGE("Not an EPCglobal TID: %02X", tid.empty() ? 0 : tid[0]);
            return false;
        }
    }

    identified.tid.assign(tid.data(), tid.size());
    tag       = identified;
    _selected = identified;
    return true;
}

uint16_t UHFLayer::bank_words(const Bank bank)
{
    switch (bank) {
        case Bank::Reserved:
            // Kill password then access password, two words each. Fixed by EPC Gen2
            return 4;
        case Bank::Epc:
            // The stored CRC and the PC, then as much EPC as the PC says there is. On a tag
            // that sends an XPC the length would count that too, so such a tag is turned away
            // at select() and never reaches here
            return static_cast<uint16_t>(2 + pcEPCLengthWords(_selected.pc));
        case Bank::Tid:
            // Whatever identify() ended up keeping, which is the whole of the TID
            return static_cast<uint16_t>(_selected.tid.size / 2);
        default:
            // Only the tag or the chip can say, and often neither does
            return static_cast<uint16_t>(_selected.user_memory_bits / 16);
    }
}

bool UHFLayer::dump_words(const char* what, const Bank bank, const uint16_t word_address, const uint16_t words)
{
    printf("== %s ==\n", what);
    if (words == 0) {
        // Saying nothing here would read as an empty bank, which is a different thing
        printf("(size not known)\n");
        return false;
    }

    // Eight words to a line, so sixteen bytes
    constexpr uint16_t PER_LINE{8};
    for (uint16_t off = 0; off < words; off += PER_LINE) {
        const uint16_t at = static_cast<uint16_t>(word_address + off);
        const uint16_t n  = std::min<uint16_t>(PER_LINE, words - off);
        std::vector<uint8_t> data{};
        if (!readBank(data, bank, at, n)) {
            printf("[%03u/%03X] ERROR\n", at, at);
            return false;
        }
        printf("[%03u/%03X]:", at, at);
        for (size_t i = 0; i + 1 < data.size(); i += 2) {
            printf("%02X%02X ", data[i], data[i + 1]);
        }
        printf("\n");
    }
    return true;
}

bool UHFLayer::dump(const Bank bank, const uint16_t word_address, const uint16_t words)
{
    if (!_has_selection) {
        M5_LIB_LOGE("dump needs a tag to have been selected");
        return false;
    }
    static const char* names[] = {"Reserved", "EPC", "TID", "User"};
    return dump_words(names[static_cast<uint8_t>(bank) & 0x03], bank, word_address, words);
}

bool UHFLayer::dump(const Bank bank)
{
    return dump(bank, 0, bank_words(bank));
}

bool UHFLayer::dump()
{
    if (!_has_selection) {
        M5_LIB_LOGE("dump needs a tag to have been selected");
        return false;
    }
    // The TID says which chip this is, and the chip is what says how much User memory there is
    // when the tag itself does not. Without it the last bank would always be skipped
    if (_selected.tid.empty()) {
        Tag identified{};
        if (!identify(identified)) {
            M5_LIB_LOGW("Could not identify the tag; its TID and User bank stay unread");
        }
    }

    bool ok = dump(Bank::Reserved);
    ok &= dump(Bank::Epc);
    ok &= dump(Bank::Tid);
    if (bank_words(Bank::User) != 0) {
        ok &= dump(Bank::User);
    } else if (chipHasNoUserMemory(_selected.chip)) {
        // Not a failure to read one: this chip is built without the bank
        printf("== User ==\n(the chip has none)\n");
    } else {
        // Neither the tag nor the chip table gave a size. Reading one word is what says whether
        // the bank is there at all, which the PC does not: a chip that computes its user memory
        // indicator reports zero for a bank it has but that nobody has written to
        std::vector<uint8_t> probe{};
        printf("== User ==\n");
        printf(readBank(probe, Bank::User, 0, 1) ? "(size not known, and the first word reads)\n"
                                                 : "(size not known, and it did not read)\n");
    }
    return ok;
}

bool UHFLayer::readBank(std::vector<uint8_t>& out, const Bank bank, const uint16_t word_address,
                        const uint16_t word_count)
{
    out.clear();
    if (!_has_selection) {
        M5_LIB_LOGE("readBank needs a tag to have been selected");
        return false;
    }
    if (!pause_polling()) {
        return false;
    }
    return _u.readTagMemory(out, bank, word_address, word_count, _access_password);
}

bool UHFLayer::readBank(uint8_t* out, uint16_t& out_len, const Bank bank, const uint16_t word_address,
                        const uint16_t word_count)
{
    const uint16_t capacity = out_len;
    out_len                 = 0;
    if (out == nullptr || capacity < word_count * 2) {
        M5_LIB_LOGE("A buffer of %u bytes cannot hold %u words", capacity, word_count);
        return false;
    }
    std::vector<uint8_t> read{};
    if (!readBank(read, bank, word_address, word_count)) {
        return false;
    }
    // A tag answering with more than was asked for is not something to write past the end of
    // the caller's buffer for
    if (read.size() > capacity) {
        M5_LIB_LOGE("The tag answered with %u bytes, more than the %u asked for", (unsigned)read.size(), capacity);
        return false;
    }
    std::memcpy(out, read.data(), read.size());
    out_len = static_cast<uint16_t>(read.size());
    return true;
}

bool UHFLayer::writeBank(const Bank bank, const uint16_t word_address, const uint8_t* data, const uint16_t data_len)
{
    if (!_has_selection) {
        M5_LIB_LOGE("writeBank needs a tag to have been selected");
        return false;
    }
    if (data == nullptr || data_len == 0 || (data_len % 2) != 0) {
        M5_LIB_LOGE("A write of %u bytes is not a whole number of words", data_len);
        return false;
    }
    // The first word of the EPC bank holds the CRC the tag works out for itself, and a tag told
    // to write there does not write and reports the parameters as unsupported instead (Gen2
    // v2.1 6.3.2.1.2.1). Turning it away here keeps a request that cannot succeed off the air,
    // where its failure would look like any other
    if (bank == Bank::Epc && word_address == 0) {
        M5_LIB_LOGE("The first word of the EPC bank is the tag's own CRC and cannot be written");
        return false;
    }
    if (!pause_polling()) {
        return false;
    }

    // How many words one command may carry is left to the chip. Gen2 makes Write, which carries
    // one word, mandatory, and BlockWrite, which carries several, optional (v2.1 6.3.2.1). Of
    // the chips this has run against, UCODE G2iM gives its BlockWrite as 32 bits and Monza 4
    // takes two words only from an even address, refusing more than two outright. A command
    // carrying more than two words can come back failed with some of them written and the rest
    // not, and nothing in the answer says how many.
    // One word at a time is therefore what is sent, since Write is the one command every tag
    // has to implement. The Reserved bank is the exception: a password there is two words and
    // is also what authorises the write, so a word at a time would invalidate the credential
    // the second word goes under, and two words from an even address is what these chips
    // document as their limit
    const uint16_t chunk_words = (bank == Bank::Reserved) ? 2 : 1;

    uint16_t written = 0;
    while (written < data_len) {
        const uint16_t at = static_cast<uint16_t>(word_address + written / 2);
        // An odd start costs one single-word write to reach the even address the pair needs
        const uint16_t words = (at % 2) ? 1 : chunk_words;
        const uint16_t chunk = std::min(static_cast<uint16_t>(data_len - written), static_cast<uint16_t>(words * 2));
        if (!_u.writeTagMemory(bank, at, data + written, chunk, _access_password)) {
            M5_LIB_LOGE("Wrote %u of %u words, then failed in the %u from word %u", written / 2, data_len / 2,
                        chunk / 2, at);
            // Part of what the mask matches on may have been replaced, which leaves a selection
            // that no longer picks this tag out. That is as true of a failure as of a success
            drop_selection_if_mask_rewritten(bank);
            return false;
        }
        written = static_cast<uint16_t>(written + chunk);
    }

    drop_selection_if_mask_rewritten(bank);
    return true;
}

void UHFLayer::drop_selection_if_mask_rewritten(const Bank bank)
{
    // A mask matches on bytes that have just been replaced, so it no longer picks this tag out.
    // Dropping the selection turns a silent mis-address into an explicit "select again"
    if (bank == Bank::Epc && _mask_bank == Bank::Epc) {
        M5_LIB_LOGW("The EPC the selection matched on has been rewritten; select the tag again");
        deselect();
    }
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
    if (!pause_polling()) {
        return false;
    }

    // A lock needs the tag in a state the reader has to put it in first, and getting there is
    // an exchange of its own that can fail on its own. The module reports that the same way it
    // reports a tag that never heard the command, so a single try cannot tell the two apart.
    // The mask goes back before each further try, since what was lost may be the singulation
    constexpr uint8_t ATTEMPTS{3};
    constexpr uint32_t RETRY_INTERVAL_MS{20};
    const uint32_t payload = buildLockPayload(settings.data(), settings.size());
    for (uint8_t attempt = 0; attempt < ATTEMPTS; ++attempt) {
        if (attempt && !reapply_selection()) {
            break;
        }
        if (_u.lockTagMemory(payload, _access_password)) {
            return true;
        }
        m5::utility::delay(RETRY_INTERVAL_MS);
    }
    return false;
}

bool UHFLayer::readBlockPermalock(std::vector<uint8_t>& mask, const Bank bank, const uint16_t block_pointer,
                                  const uint8_t block_range)
{
    mask.clear();
    if (!_has_selection) {
        M5_LIB_LOGE("readBlockPermalock needs a tag to have been selected");
        return false;
    }
    if (!pause_polling()) {
        return false;
    }
    return _u.blockPermalock(mask, bank, block_pointer, block_range, nullptr, 0, _access_password, false);
}

bool UHFLayer::blockPermalock(const Bank bank, const uint8_t* mask, const size_t mask_len, const bool allow_permanent,
                              const uint16_t block_pointer, const uint8_t block_range)
{
    if (!_has_selection) {
        M5_LIB_LOGE("blockPermalock needs a tag to have been selected");
        return false;
    }
    if (!allow_permanent) {
        M5_LIB_LOGE("blockPermalock is permanent; pass allow_permanent to mean it");
        return false;
    }
    if (mask == nullptr || mask_len == 0) {
        M5_LIB_LOGE("blockPermalock needs a mask saying which blocks to lock");
        return false;
    }
    if (!pause_polling()) {
        return false;
    }
    std::vector<uint8_t> ignored{};
    return _u.blockPermalock(ignored, bank, block_pointer, block_range, mask, mask_len, _access_password,
                             allow_permanent);
}

bool UHFLayer::readQTParameters(QTParameters& qt)
{
    qt = QTParameters{};
    if (!_has_selection) {
        M5_LIB_LOGE("readQTParameters needs a tag to have been selected");
        return false;
    }
    // Only a chip that is known not to have the command is turned away. One this table does
    // not name is tried: not knowing is not the same as knowing it cannot
    if (_selected.chip != Chip::Unknown && !chipSupportsQT(_selected.chip)) {
        M5_LIB_LOGE("%s has no QT command", _selected.chipAsString().c_str());
        return false;
    }
    if (!pause_polling()) {
        return false;
    }
    uint16_t control{};
    if (!_u.qtCommand(control, false, false, _access_password)) {
        return false;
    }
    qt.short_range   = (control & QT_SHORT_RANGE) != 0;
    qt.public_memory = (control & QT_PUBLIC_MEMORY) != 0;
    return true;
}

bool UHFLayer::writeQTParameters(const QTParameters& qt, const bool persistent)
{
    if (!_has_selection) {
        M5_LIB_LOGE("writeQTParameters needs a tag to have been selected");
        return false;
    }
    // Only a chip that is known not to have the command is turned away. One this table does
    // not name is tried: not knowing is not the same as knowing it cannot
    if (_selected.chip != Chip::Unknown && !chipSupportsQT(_selected.chip)) {
        M5_LIB_LOGE("%s has no QT command", _selected.chipAsString().c_str());
        return false;
    }
    if (!pause_polling()) {
        return false;
    }
    uint16_t control =
        static_cast<uint16_t>((qt.short_range ? QT_SHORT_RANGE : 0) | (qt.public_memory ? QT_PUBLIC_MEMORY : 0));
    if (!_u.qtCommand(control, true, persistent, _access_password)) {
        return false;
    }
    // The public map answers with an EPC of its own, so a mask built from the private one stops
    // matching. Whichever way it went, the mask that is stored may no longer name this tag
    if (qt.public_memory) {
        M5_LIB_LOGW("The tag answers under a different EPC now; select it again");
        _has_selection = false;
    }
    return true;
}

bool UHFLayer::reapply_selection()
{
    if (_mask_bank == Bank::Tid && !_selected.tid.empty()) {
        return _u.writeSelectParameter(Bank::Tid, TID_MASK_POINTER_BITS, _selected.tid.begin(), _selected.tid.size) &&
               _u.writeSelectEnabled(true);
    }
    if (_mask_bank == Bank::Epc && !_selected.epc.empty()) {
        return _u.writeSelectParameter(Bank::Epc, EPC_MASK_POINTER_BITS, _selected.epc.begin(), _selected.epc.size) &&
               _u.writeSelectEnabled(true);
    }
    M5_LIB_LOGW("Nothing to build a mask from; the selection cannot be put back");
    return false;
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

    if (!pause_polling()) {
        return false;
    }
    if (!_u.killTag(kill_password)) {
        return false;
    }
    // Nothing answers to the mask any more
    _has_selection = false;
    _selected      = Tag{};
    _u.writeSelectEnabled(false);
    resume_polling();
    return true;
}

}  // namespace uhf
}  // namespace m5
