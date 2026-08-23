/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file uhf.hpp
  @brief Common vocabulary for UHF-RFID (EPCglobal UHF Class 1 Gen 2 / ISO 18000-6C)
*/
#ifndef M5_UNIT_RFID_UHF_UHF_HPP
#define M5_UNIT_RFID_UHF_UHF_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace m5 {
/*!
  @namespace uhf
  @brief UHF-RFID (EPC Gen2) related namespace
 */
namespace uhf {

/*!
  @enum Bank
  @brief EPC Gen2 memory bank
 */
enum class Bank : uint8_t {
    Reserved,  //!< Kill password and access password
    Epc,       //!< CRC-16, PC and EPC
    Tid,       //!< Tag identification
    User,      //!< User memory
};

/*!
  @enum Region
  @brief Operating region
  @note Japan is not defined by the module; the 920-925MHz band overlaps China900MHz
 */
enum class Region : uint8_t {
    Unspecified,  //!< Keep the module's factory setting
    China900MHz,
    America,
    Europe,
    China800MHz,
    SouthKorea,
};

/*!
  @enum Session
  @brief EPC Gen2 session
 */
enum class Session : uint8_t { S0, S1, S2, S3 };

/*!
  @enum Target
  @brief EPC Gen2 inventoried flag target
 */
enum class Target : uint8_t { A, B };

//! @brief Longest EPC the standard allows: the PC length field is 5 bits, so 31 words
constexpr size_t EPC_MAX_BYTES{62};
/*!
  @brief How much of the TID identify() keeps
  @details Enough for the XTID header, a serial of up to 144 bits, and the segments that
  follow it (TDS 16.2)
 */
constexpr size_t TID_MAX_BYTES{32};

/*!
  @struct Epc
  @brief EPC of a tag
  @details A tag's EPC is variable length, so the used size travels with the bytes. The
  storage is a fixed array: queueing tags must never touch the heap
 */
struct Epc {
    std::array<uint8_t, EPC_MAX_BYTES> data{};  //!< Bytes, only the first size are valid
    uint8_t size{};                             //!< Number of valid bytes

    //! @brief Byte at the given index
    inline uint8_t operator[](const size_t i) const
    {
        return data[i];
    }
    //! @brief Byte at the given index
    inline uint8_t& operator[](const size_t i)
    {
        return data[i];
    }
    inline const uint8_t* begin() const
    {
        return data.data();
    }
    inline const uint8_t* end() const
    {
        return data.data() + size;
    }
    //! @brief Holds no EPC?
    inline bool empty() const
    {
        return size == 0;
    }
    //! @brief Copy in from a raw buffer, refusing anything longer than EPC_MAX_BYTES
    bool assign(const uint8_t* src, const size_t len);
    bool operator==(const Epc& o) const;
    //! @brief Uppercase hex
    std::string toString() const;
};

/*!
  @struct Tid
  @brief TID of a tag
  @details Permalocked at manufacture, so unlike the EPC it never changes. The serial is 0 to
  144 bits long depending on the XTID header, hence the variable size
 */
struct Tid {
    std::array<uint8_t, TID_MAX_BYTES> data{};  //!< Bytes, only the first size are valid
    uint8_t size{};                             //!< Number of valid bytes

    //! @brief Byte at the given index
    inline uint8_t operator[](const size_t i) const
    {
        return data[i];
    }
    //! @brief Byte at the given index
    inline uint8_t& operator[](const size_t i)
    {
        return data[i];
    }
    inline const uint8_t* begin() const
    {
        return data.data();
    }
    inline const uint8_t* end() const
    {
        return data.data() + size;
    }
    //! @brief Holds no TID?
    inline bool empty() const
    {
        return size == 0;
    }
    //! @brief Copy in from a raw buffer, refusing anything longer than TID_MAX_BYTES
    bool assign(const uint8_t* src, const size_t len);
    bool operator==(const Tid& o) const;
    //! @brief Uppercase hex
    std::string toString() const;
};

/*!
  @enum Vendor
  @brief Tag mask-designer identifier held in TID bits 0Bh to 13h
 */
enum class Vendor : uint16_t {
    Unknown = 0x000,
    Impinj  = 0x001,
    Alien   = 0x003,
    NXP     = 0x006,
};

/*!
  @enum Chip
  @brief Chip a tag is built around, resolved from the mask-designer identifier and model number
 */
enum class Chip : uint8_t {
    Unknown,
    AlienHiggs3,
    AlienHiggs9,
    ImpinjMonza4QT,
    NxpUcodeG2iM,
    NxpUcodeG2iMPlus,
};

/*!
  @struct Tag
  @brief A tag, as the EPC Gen2 standard calls it
  @details Detection fills the inventory half of this; identifying the tag fills the rest by
  reading its TID
 */
struct Tag {
    // Filled in by detection
    uint16_t pc{};   //!< Protocol Control
    uint16_t crc{};  //!< CRC-16 reported by the tag
    int8_t rssi{};   //!< RSSI in dBm (signed)
    Epc epc{};       //!< EPC

    // Filled in by identification
    Tid tid{};                        //!< TID starting at word 0
    Chip chip{Chip::Unknown};         //!< Identified chip
    Vendor vendor{Vendor::Unknown};   //!< Mask-designer identifier
    uint16_t model_number{};          //!< Tag model number (12 bits)
    uint16_t serial_bits{};           //!< Length of the XTID serial, 0 when the tag has none
    uint16_t user_memory_bits{};      //!< Size of the User bank, 0 when unknown
    uint16_t epc_max_bits{};          //!< Largest EPC the chip accepts, 0 when unknown
    uint16_t permalock_block_bits{};  //!< BlockPermalock granularity, 0 when unknown
    bool has_xtid{};                  //!< XTID indicator (TID bit 08h)
    bool supports_security{};         //!< Security indicator (TID bit 09h)
    bool supports_file{};             //!< File indicator (TID bit 0Ah)

    //! @brief Holds a usable EPC?
    inline bool valid() const
    {
        return !epc.empty();
    }
    //! @brief Chip name, "Unknown" until the tag has been identified
    std::string chipAsString() const;
};

//! @brief ISO/IEC 15963 allocation class identifier used by EPCglobal tags
constexpr uint8_t TID_CLASS_EPCGLOBAL{0xE2};

/*!
  @brief Serial length an XTID header announces
  @param xtid_header XTID header word, TID bits 20h to 2Fh
  @return Length of the serial in bits, 0 when the tag carries none
  @details TDS 16.2.2 puts the serialisation bits in 15 to 13: zero means no serial, and any
  other value means 48 + (value - 1) * 16 bits, so up to 144
 */
inline uint16_t xtidSerialBits(const uint16_t xtid_header)
{
    const uint8_t v = static_cast<uint8_t>((xtid_header >> 13) & 0x07);
    return v == 0 ? 0U : static_cast<uint16_t>(48 + (v - 1) * 16);
}

/*!
  @brief Resolve a chip from its mask-designer identifier and model number
  @param vendor Mask-designer identifier
  @param model_number Tag model number
  @return Chip, or Chip::Unknown when the pair is not one we have confirmed
  @note Only pairs read from a manufacturer datasheet are listed. A tag whose chip comes back
  Unknown still reports its vendor and model number
 */
inline Chip resolveChip(const Vendor vendor, const uint16_t model_number)
{
    if (vendor == Vendor::NXP) {
        // UCODE G2iM/G2iM+ datasheet Rev 3.7: TID starts E200680Ah and E200680Bh
        if (model_number == 0x80A) {
            return Chip::NxpUcodeG2iM;
        }
        if (model_number == 0x80B) {
            return Chip::NxpUcodeG2iMPlus;
        }
    }
    return Chip::Unknown;
}

/*!
  @brief Decode the fixed part of a TID into a tag
  @param[in,out] tag Tag to fill
  @param tid TID bytes starting at word 0
  @param len Length of tid in bytes, at least 4
  @return True if the TID carries the EPCglobal class identifier
  @details Gen2 v2.1 6.3.2.1.3 fixes the layout of the first 32 bits: an 8-bit class
  identifier, the XTID, security and file indicators, a 9-bit mask-designer identifier and a
  12-bit model number. Bytes 4 and 5, when present and the tag has an XTID, hold the XTID header
 */
inline bool decodeTid(Tag& tag, const uint8_t* tid, const size_t len)
{
    if (tid == nullptr || len < 4 || tid[0] != TID_CLASS_EPCGLOBAL) {
        return false;
    }
    tag.has_xtid          = (tid[1] & 0x80) != 0;
    tag.supports_security = (tid[1] & 0x40) != 0;
    tag.supports_file     = (tid[1] & 0x20) != 0;
    tag.vendor            = static_cast<Vendor>(((tid[1] & 0x1F) << 4) | (tid[2] >> 4));
    tag.model_number      = static_cast<uint16_t>(((tid[2] & 0x0F) << 8) | tid[3]);
    tag.chip              = resolveChip(tag.vendor, tag.model_number);
    tag.serial_bits       = 0;
    if (tag.has_xtid && len >= 6) {
        tag.serial_bits = xtidSerialBits(static_cast<uint16_t>((tid[4] << 8) | tid[5]));
    }
    return true;
}

/*!
  @enum LockTarget
  @brief What a lock setting applies to, in the order EPC Gen2 lays them out
 */
enum class LockTarget : uint8_t { KillPassword, AccessPassword, Epc, Tid, User };

/*!
  @enum LockAction
  @brief What a lock setting does
  @details EPC Gen2 does not define these as four values but as two independent bits, a
  pwd-write bit and a permalock bit (v1.2.0 Table 6.43). These four names are the four
  combinations of those bits
 */
enum class LockAction : uint8_t {
    Open          = 0x00,  //!< Writeable from either the open or the secured state
    PermanentOpen = 0x01,  //!< Permanently writeable and can never be locked. Cannot be undone
    Lock          = 0x02,  //!< Writeable from the secured state only
    PermanentLock = 0x03,  //!< Not writeable from any state. Cannot be undone
};

/*!
  @struct LockSetting
  @brief One entry of a lock operation. Targets left out keep their current lock state
 */
struct LockSetting {
    LockTarget target{};
    LockAction action{};
};

//! @brief Is this action one of the two that cannot be undone?
inline bool isPermanent(const LockAction action)
{
    return action == LockAction::PermanentOpen || action == LockAction::PermanentLock;
}

/*!
  @brief Build the 20-bit payload of the Gen2 Lock command
  @param settings Settings to apply
  @param count Number of settings
  @return 20-bit payload, Mask in bits 19-10 and Action in bits 9-0
  @details Each target owns a 2-bit action, a pwd-write bit and a permalock bit, and a 2-bit
  mask that says which of those two to overwrite. Only the permalock bit is masked in when the
  action asserts it: a permalock bit can never be cleared once set (v1.2.0 6.3.2.11.3.5), so
  asking to write a zero there would be asking for an error
 */
inline uint32_t buildLockPayload(const LockSetting* settings, const size_t count)
{
    uint32_t payload{};
    for (size_t i = 0; i < count; ++i) {
        const uint8_t slot         = static_cast<uint8_t>(settings[i].target);  // 0:Kill ... 4:User
        const uint8_t action       = static_cast<uint8_t>(settings[i].action);
        const uint8_t mask         = static_cast<uint8_t>(0x02 | (action & 0x01));
        const uint8_t mask_shift   = static_cast<uint8_t>(18 - slot * 2);
        const uint8_t action_shift = static_cast<uint8_t>(8 - slot * 2);
        payload |= static_cast<uint32_t>(mask) << mask_shift;
        payload |= static_cast<uint32_t>(action) << action_shift;
    }
    return payload;
}

/*!
  @struct QueryParameters
  @brief EPC Gen2 query parameters
 */
struct QueryParameters {
    uint8_t q{};                   //!< Q value
    Session session{Session::S0};  //!< Session
    Target target{Target::A};      //!< Inventoried flag target
};

/*!
  @struct ModuleInformation
  @brief Reader module information
 */
struct ModuleInformation {
    std::string hardware_version{};
    std::string software_version{};
    std::string manufacturer{};
};

/*!
  @struct ChannelLevels
  @brief Signal level measured on each channel of the operating region
  @details The reader scans a contiguous range of channels and reports one signed level per
  channel, so the level of channel first_channel + i is dbm[i]
 */
struct ChannelLevels {
    uint8_t first_channel{};    //!< Index of the first measured channel
    std::vector<int8_t> dbm{};  //!< Measured level of each channel in dBm (signed)
};

namespace detail {
//! @brief Render a byte buffer as uppercase hex
inline std::string to_hex(const uint8_t* data, const size_t len)
{
    static const char digits[] = "0123456789ABCDEF";
    std::string s{};
    s.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        s += digits[(data[i] >> 4) & 0x0F];
        s += digits[data[i] & 0x0F];
    }
    return s;
}
}  // namespace detail

inline bool Epc::assign(const uint8_t* src, const size_t len)
{
    if (!src || len > EPC_MAX_BYTES) {
        return false;
    }
    data.fill(0);
    for (size_t i = 0; i < len; ++i) {
        data[i] = src[i];
    }
    size = static_cast<uint8_t>(len);
    return true;
}

inline bool Epc::operator==(const Epc& o) const
{
    if (size != o.size) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        if (data[i] != o.data[i]) {
            return false;
        }
    }
    return true;
}

inline std::string Epc::toString() const
{
    return detail::to_hex(data.data(), size);
}

inline bool Tid::assign(const uint8_t* src, const size_t len)
{
    if (!src || len > TID_MAX_BYTES) {
        return false;
    }
    data.fill(0);
    for (size_t i = 0; i < len; ++i) {
        data[i] = src[i];
    }
    size = static_cast<uint8_t>(len);
    return true;
}

inline bool Tid::operator==(const Tid& o) const
{
    if (size != o.size) {
        return false;
    }
    for (size_t i = 0; i < size; ++i) {
        if (data[i] != o.data[i]) {
            return false;
        }
    }
    return true;
}

inline std::string Tid::toString() const
{
    return detail::to_hex(data.data(), size);
}

inline std::string Tag::chipAsString() const
{
    switch (chip) {
        case Chip::AlienHiggs3:
            return "Alien Higgs-3";
        case Chip::AlienHiggs9:
            return "Alien Higgs-9";
        case Chip::ImpinjMonza4QT:
            return "Impinj Monza 4QT";
        case Chip::NxpUcodeG2iM:
            return "NXP UCODE G2iM";
        case Chip::NxpUcodeG2iMPlus:
            return "NXP UCODE G2iM+";
        default:
            return "Unknown";
    }
}

/*!
  @brief EPC length in 16-bit words encoded in the PC
  @param pc Protocol Control
  @return Number of 16-bit words comprising the EPC
 */
inline uint8_t pcEPCLengthWords(const uint16_t pc)
{
    return static_cast<uint8_t>((pc >> 11) & 0x1F);
}

/*!
  @brief User Memory Indicator of the PC
  @param pc Protocol Control
  @return True if the tag reports user memory holding data
 */
inline bool pcUserMemoryIndicator(const uint16_t pc)
{
    return (pc & 0x0400) != 0;
}

/*!
  @brief XPC indicator of the PC
  @param pc Protocol Control
  @return True if an XPC word follows
 */
inline bool pcXPCIndicator(const uint16_t pc)
{
    return (pc & 0x0200) != 0;
}

/*!
  @brief Numbering System Identifier of the PC
  @param pc Protocol Control
  @return NSI (0x000 for EPCglobal)
 */
inline uint16_t pcNumberingSystemIdentifier(const uint16_t pc)
{
    return static_cast<uint16_t>(pc & 0x01FF);
}

/*!
  @brief Append a tag unless an identical EPC is already present
  @param[in,out] dst Destination
  @param tag Tag to append
  @return True if the tag was appended
  @note A tag whose EPC is empty is never appended
 */
inline bool append_unique(std::vector<Tag>& dst, const Tag& tag)
{
    if (tag.epc.empty()) {
        return false;
    }
    for (size_t i = 0; i < dst.size(); ++i) {
        if (dst[i].epc == tag.epc) {
            return false;
        }
    }
    dst.push_back(tag);
    return true;
}

}  // namespace uhf
}  // namespace m5
#endif
