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

#include <m5_utility/crc.hpp>

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
  @details The regions the module's firmware offers, carrying the values it uses for them
 */
enum class Region : uint8_t {
    Unspecified = 0x00,  //!< Keep the module's factory setting
    China900MHz = 0x01,
    America     = 0x02,
    Europe      = 0x03,
    China800MHz = 0x04,
    SouthKorea  = 0x06,
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

/*!
  @enum SelectFilter
  @brief Which tags an inventory round invites, in terms of the SL flag
  @details Decides whether a stored select mask narrows the field down at all. A reader left on
  All ignores the SL flag outright, so a Select that acts on SL stores a mask that changes
  nothing; one that acts on an inventoried flag still works because the round is qualified by
  the session instead (EPC Gen2 v2.1 Table 6-32 and 6.3.2.3)
 */
enum class SelectFilter : uint8_t {
    All,          //!< Every tag answers, whatever its SL flag says
    AllAlias,     //!< Same as All; the standard leaves this encoding equivalent
    NotSelected,  //!< Only tags whose SL flag is deasserted
    Selected,     //!< Only tags whose SL flag is asserted
};

//! @brief Longest EPC the standard allows: the PC length field is 5 bits, so 31 words
constexpr size_t EPC_MAX_BYTES{62};
/*!
  @brief How much of the TID identify() keeps
  @details The longest XTID the standard can describe: the two fixed words, the header, a
  144-bit serial and all four optional segments (TDS Table 16-2). Twenty words in all, so a
  fully populated XTID always fits
 */
constexpr size_t TID_MAX_BYTES{40};

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
  @brief Mask designers this library knows by name
  @details The value is the mask-designer identifier the registration authority assigned, as
  held in TID bits 0Bh to 13h. The list covers the designers whose chips turn up on tags in the
  field, not the whole registry; a tag from any other reports Vendor::Unknown while Tag::mdid
  still carries the raw identifier, which can be looked up in the GS1 registry at
  https://www.gs1.org/docs/epc/mdid_list.json
  @note Being named here says who made the chip, nothing more. Chip resolution is a separate
  step, and only the designers in resolveChip() have their model numbers listed
 */
enum class Vendor : uint16_t {
    Unknown               = 0x000,
    Impinj                = 0x001,
    TexasInstruments      = 0x002,
    Alien                 = 0x003,
    Atmel                 = 0x005,
    NXP                   = 0x006,
    STMicroelectronics    = 0x007,
    EPMicroelectronics    = 0x008,
    EMMicroelectronic     = 0x00B,
    Quanray               = 0x00F,
    Fujitsu               = 0x010,
    Nationz               = 0x01B,
    Invengo               = 0x01C,
    RFMicron              = 0x024,
    FudanMicroelectronics = 0x027,
    AMS                   = 0x02F,
    HuadaSemiconductor    = 0x032,
    Mikron                = 0x034,
};

namespace detail {
//! @brief Read one big-endian 16-bit word out of a byte buffer
inline uint16_t word_at(const uint8_t* data, const size_t word_index)
{
    return static_cast<uint16_t>((data[word_index * 2] << 8) | data[word_index * 2 + 1]);
}

/*!
  @struct VendorEntry
  @brief A named mask designer paired with the name the registry spells it out as
 */
struct VendorEntry {
    Vendor vendor;     //!< Enumerator
    const char* name;  //!< Name as the GS1 registry gives it
};

/*!
  @brief The mask designers this library names
  @param[out] count Number of entries
  @return Pointer to the first entry
  @details resolveVendor and Tag::vendorAsString both read this table, so a designer added here
  becomes both recognised and printable in a single edit
 */
inline const VendorEntry* vendorEntries(size_t& count)
{
    static const VendorEntry entries[] = {
        {Vendor::Impinj, "Impinj"},
        {Vendor::TexasInstruments, "Texas Instruments"},
        {Vendor::Alien, "Alien Technology"},
        {Vendor::Atmel, "Atmel"},
        {Vendor::NXP, "NXP Semiconductors"},
        {Vendor::STMicroelectronics, "ST Microelectronics"},
        {Vendor::EPMicroelectronics, "EP Microelectronics"},
        {Vendor::EMMicroelectronic, "EM Microelectronic"},
        {Vendor::Quanray, "Quanray Electronics"},
        {Vendor::Fujitsu, "Fujitsu"},
        {Vendor::Nationz, "Nationz"},
        {Vendor::Invengo, "Invengo"},
        {Vendor::RFMicron, "RFMicron"},
        {Vendor::FudanMicroelectronics, "Shanghai Fudan Microelectronics Group"},
        {Vendor::AMS, "ams AG"},
        {Vendor::HuadaSemiconductor, "Huada Semiconductor"},
        {Vendor::Mikron, "PJSC Mikron"},
    };
    count = sizeof(entries) / sizeof(entries[0]);
    return entries;
}
}  // namespace detail

/*!
  @enum Chip
  @brief Chip a tag is built around, resolved from the mask-designer identifier and model number
 */
enum class Chip : uint8_t {
    Unknown,
    AlienHiggs3,
    AlienHiggs4,
    AlienHiggs9,
    ImpinjMonza4D,
    ImpinjMonza4E,
    ImpinjMonza4i,
    ImpinjMonza4QT,
    ImpinjM730,
    ImpinjM750,
    ImpinjM770,
    ImpinjM780,
    ImpinjM781,
    NxpUcodeG2iM,
    NxpUcodeG2iMPlus,
    NxpUcode7,
    NxpUcode8,
    NxpUcode8m,
    NxpUcode9,
    NxpUcode9xe,
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
    Tid tid{};                       //!< TID starting at word 0
    Chip chip{Chip::Unknown};        //!< Identified chip, Unknown when the pair below is not listed
    Vendor vendor{Vendor::Unknown};  //!< Named mask designer, Unknown when mdid is not listed
    uint16_t mdid{};                 //!< Raw mask-designer identifier (9 bits)
    uint16_t model_number{};         //!< Tag model number (12 bits)
    uint16_t serial_bits{};          //!< Length of the XTID serial, 0 when the tag has none
    //! @brief Size of the User bank, 0 when unknown. On a chip that shares a pool with the EPC
    //! this is what the bank holds at the EPC length the PC states, not a fixed property
    uint32_t user_memory_bits{};
    uint32_t epc_max_bits{};          //!< Largest EPC the chip accepts, 0 when unknown
    uint32_t permalock_block_bits{};  //!< BlockPermalock granularity, 0 when unknown
    bool has_xtid{};                  //!< XTID indicator (TID bit 08h)
    bool supports_security{};         //!< Security indicator (TID bit 09h)
    bool supports_file{};             //!< File indicator (TID bit 0Ah)
    bool supports_block_permalock{};  //!< BlockPermaLock support, as the XTID reports it

    //! @brief Holds a usable EPC?
    inline bool valid() const
    {
        return !epc.empty();
    }
    //! @brief Mask designer as a printable name, "Unknown" when the identifier is not one we list
    std::string vendorAsString() const;
    //! @brief Chip name, "Unknown" until the tag has been identified
    std::string chipAsString() const;
};

/*!
  @brief User memory a chip is known to hold, in bits
  @param chip Chip
  @return Size in bits, 0 when this library does not know
  @details Where a tag says nothing about itself this is what is left
  @note Zero means unknown, not empty. chipHasNoUserMemory is what says a chip has none
 */
inline uint32_t chipUserMemoryBits(const Chip chip)
{
    switch (chip) {
        case Chip::AlienHiggs9:
            // ALC-390 offers "688 bit user memory", which a tag holds only while its EPC is
            // the 96 bit it leaves the factory with. chipSharedUserMemoryBits is what sizes
            // this chip; this is the figure to fall back on when the PC is unknown
            return 688;
        case Chip::AlienHiggs4:
            return 128;  // Higgs 4 V1.3.6 3: "128 bits of NVM for User data"
        // Monza 4 V10.0 Table 2-8
        case Chip::ImpinjMonza4D:
            return 32;
        case Chip::ImpinjMonza4E:
            return 128;
        case Chip::ImpinjMonza4i:
            return 480;
        case Chip::ImpinjMonza4QT:
            return 512;  // Monza 4 Rev8.0 2.3.1: "512 bits of user memory". 32 words, confirmed
        // M700 Series v6.4 1, and M780 / M781 v1.1 1
        case Chip::ImpinjM750:
            return 32;
        case Chip::ImpinjM770:
            return 32;
        case Chip::ImpinjM780:
            return 128;
        case Chip::ImpinjM781:
            return 512;
        case Chip::NxpUcode8m:
            return 32;  // SL3S1205_15 Rev3.6 Table 7: "User Memory 32 bit"
        case Chip::NxpUcodeG2iM:
            // Rev3.7 Table 7 and Table 15. The "640 bit configurable User Memory" of the
            // general description is the whole pool, and this part keeps its EPC fixed at
            // 256 bit, which takes 128 of those bits away
            return 512;
        case Chip::NxpUcodeG2iMPlus:
            // Rev3.7 Table 16, which is what it ships with. The EPC can be taken up to 448 bit
            // and the User bank shrinks by as much, so chipSharedUserMemoryBits is what sizes
            // this chip; this is the figure to fall back on when the PC is unknown
            return 640;
        default:
            return 0;
    }
}

/*!
  @brief Largest EPC a chip is known to accept, in bits
  @param chip Chip
  @return Size in bits, 0 when this library does not know
  @details Sourced the same way as chipUserMemoryBits
 */
inline uint32_t chipEpcMaxBits(const Chip chip)
{
    switch (chip) {
        case Chip::AlienHiggs9:
            return 496;  // ALC-390: "Supports EPC size up to 496b"
        case Chip::AlienHiggs4:
            return 128;  // Higgs 4 V1.3.6 3: "128 bits of NVM for Electronic Product Code"
        // Monza 4 V10.0 Table 2-8
        case Chip::ImpinjMonza4D:
            return 128;
        case Chip::ImpinjMonza4E:
            return 496;
        case Chip::ImpinjMonza4i:
            return 256;
        case Chip::ImpinjMonza4QT:
            return 128;  // Monza 4 Rev8.0 2.3.1: "128 bits of EPC memory"
        // M700 Series v6.4 1, and M780 / M781 v1.1 1
        case Chip::ImpinjM730:
            return 128;
        case Chip::ImpinjM750:
            return 96;
        case Chip::ImpinjM770:
            return 128;
        case Chip::ImpinjM780:
            return 496;
        case Chip::ImpinjM781:
            return 128;
        case Chip::NxpUcode7:
            return 128;  // SL3S1204 Rev4.0 Table 8
        case Chip::NxpUcode8m:
            return 96;  // SL3S1205_15 Rev3.6 Table 7
        case Chip::NxpUcode8:
            // SL3S1205_15 Rev3.6 Table 6 gives 128 bit, and a real tag takes a PC of eight
            // words and refuses nine
            return 128;
        case Chip::NxpUcode9:
            return 96;  // SL3S1206 Rev3.5 Table 5: "EPC (excluding 16 bit CRC-16 and 16-bit PC) 96 bit"
        case Chip::NxpUcode9xe:
            return 128;  // SL3S1216 Rev3.3 2.1.1: "128-bit of EPC memory"
        case Chip::NxpUcodeG2iM:
            return 256;  // G2iM Rev3.7: "256 bit of EPC memory"
        case Chip::NxpUcodeG2iMPlus:
            return 448;  // G2iM Rev3.7: "up to 448 bit EPC for UCODE G2iM+"
        default:
            return 0;
    }
}

/*!
  @brief Words of TID a chip carries without an XTID header to say so
  @param chip Chip
  @return Total words of TID, or zero where the header is the only thing that says
  @details A chip whose XTID header reads zero still has whatever its datasheet lays out, and
  some keep a permalocked serial there. Stopping at the two fixed words would leave that behind,
  and a TID that stops short of the serial is the same on every tag of the model
 */
inline uint16_t chipTidWords(const Chip chip)
{
    switch (chip) {
        case Chip::AlienHiggs3:
            // Higgs 3 IC Datasheet Supplement, TID Bank rows 0 to 5: the first two words name
            // the chip and the four after them are Unique ID Word 0 to 3. Row 6 onwards is
            // reserved, and the XTID indicator of this chip reads zero
            return 6;
        case Chip::NxpUcodeG2iM:
        case Chip::NxpUcodeG2iMPlus:
            // SL3S1003_1013 Rev3.7 Table 8: 30h to 5Fh is a permalocked serial number, under an
            // XTID header that reads 00h. The TID ends there; 60h onwards is User TID memory
            return 6;
        default:
            return 0;
    }
}

/*!
  @brief Is this a chip whose datasheet gives it the QT command?
  @param chip Chip
  @return True where the datasheet says it has it
  @details Only a chip that is known not to have it can be turned away. A chip this table does
  not name is worth trying rather than refusing: not knowing is not the same as knowing it
  cannot
 */
inline bool chipSupportsQT(const Chip chip)
{
    switch (chip) {
        case Chip::ImpinjMonza4QT:
            // Monza 4 V10.0 Table 2-8, the only part of the family with the column ticked
            return true;
        default:
            return false;
    }
}

/*!
  @brief Size of the block BlockPermalock works in, in bits
  @param chip Chip
  @return Block size, or zero where it is not known
  @details The standard leaves this to the chip, and a tag says it only through the XTID
  segment that most chips do not carry. Where the datasheet gives it, this is where it comes
  from instead
 */
inline uint32_t chipPermalockBlockBits(const Chip chip)
{
    switch (chip) {
        case Chip::ImpinjMonza4QT:
            // Monza 4 V10.0 2.3.1: "four fixed, 128-bit sections of user memory"
            return 128;
        default:
            return 0;
    }
}

/*!
  @brief Is this a chip whose datasheet says it has no User bank at all?
  @param chip Chip
  @return True when the chip is known to have none
  @details Nothing a tag says distinguishes "no User bank" from "a User bank nobody has written
  to yet", so this is the only thing that can tell a caller the bank is not worth reading
 */
inline bool chipHasNoUserMemory(const Chip chip)
{
    switch (chip) {
        case Chip::NxpUcode8:
            // SL3S1205_15 Rev3.6 Table 8: the memory map has no Bank 11 and the UMI bit of the
            // PC is "hardwired to 0"
            return true;
        case Chip::NxpUcode9:
            return true;  // SL3S1206 Rev3.5 Table 6, the same way
        case Chip::NxpUcode9xe:
            return true;  // SL3S1216 Rev3.3 Table 5, the same way
        case Chip::NxpUcode7:
            return true;  // SL3S1204 Rev4.0 features: "No User Memory"
        case Chip::ImpinjM730:
            return true;  // M700 Series v6.4 1: "128 bits of EPC memory, 0 bits of user memory"
        default:
            return false;
    }
}

/*!
  @brief User memory of a chip that carves it out of one pool with the EPC, in bits
  @param chip Chip
  @param pc Protocol Control of the tag, which is where the length of the EPC is
  @return Size in bits, 0 when the chip holds the two apart or the PC is not known
  @details A few chips give the EPC and the User bank one pool to share, so neither size can be
  stated without the other. Measured on a Higgs 9: every word the EPC grows by is a word the
  User bank loses, and the two always add up to 49. Its datasheet offers 496 bit of EPC and 688
  bit of User, which overrun the 1024 bit the same sheet gives the chip; both are true, and
  neither is true at the same time as the other
 */
inline uint32_t chipSharedUserMemoryBits(const Chip chip, const uint16_t pc)
{
    // ALC-390: 6 words of EPC and 43 of User as it ships, and every word the EPC grows by is a
    // word the User bank loses
    constexpr uint16_t HIGGS9_POOL_WORDS{49};
    // Rev3.7 Tables 16 and 17: 128 bit of EPC with 640 of User, or 448 with 320. Both come to
    // 768 bit, which the datasheet says moves "in steps of 64 bit"
    constexpr uint16_t G2IM_PLUS_POOL_WORDS{48};

    uint16_t pool{};
    switch (chip) {
        case Chip::AlienHiggs9:
            pool = HIGGS9_POOL_WORDS;
            break;
        case Chip::NxpUcodeG2iMPlus:
            pool = G2IM_PLUS_POOL_WORDS;
            break;
        default:
            return 0;
    }
    // A tag selected by its TID reaches identify() with no PC, and a bank sized from a length
    // of zero would be the whole pool. Returning 0 rather than a wrong size
    if (pc == 0) {
        return 0;
    }
    const uint16_t epc_words = static_cast<uint16_t>((pc >> 11) & 0x1F);
    return epc_words < pool ? static_cast<uint32_t>(pool - epc_words) * 16U : 0U;
}

//! @brief ISO/IEC 15963 allocation class identifier used by EPCglobal tags
constexpr uint8_t TID_CLASS_EPCGLOBAL{0xE2};

/*!
  @brief Does a TID say it carries an extended TID, and with it a serial number?
  @param tid TID bytes starting at word 0
  @param len Length of tid in bytes, at least 2
  @return True when the XTID indicator is set
  @details Bit 08h of the TID. What lies past word 1 is only defined when this is set: the Tag
  Data Standard calls a TID without it a short TID, "in which the values beyond address 1Fh are
  not defined" (TDS 1.5 16.1), and EPC Gen2 v3 asks for a serial number only of tags that set it
 */
inline bool tidHasXtid(const uint8_t* tid, const size_t len)
{
    return tid != nullptr && len >= 2 && (tid[1] & 0x80) != 0;
}

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
  @name XTID header bits announcing an optional segment (TDS Table 16-3)
  @details The segments follow the serial in the order the bits are listed here, from the most
  significant down, and a segment that is absent moves the ones below it to lower addresses
 */
///@{
constexpr uint16_t XTID_OPTIONAL_COMMAND_SUPPORT{0x1000};    //!< bit 12, one word
constexpr uint16_t XTID_BLOCKWRITE_BLOCKERASE{0x0800};       //!< bit 11, four words
constexpr uint16_t XTID_USER_MEMORY_BLOCKPERMALOCK{0x0400};  //!< bit 10, two words
constexpr uint16_t XTID_LOCK_BIT{0x0200};                    //!< bit 9, one word
constexpr uint16_t XTID_EXTENDED_HEADER{0x0001};             //!< bit 0, header continues
///@}

//! @brief Words every TID starts with, which say what the chip is and whether an XTID follows
constexpr size_t TID_FIXED_WORDS{2};
//! @brief Words the two fixed TID words and the XTID header take up
constexpr size_t XTID_FIXED_WORDS{3};

/*!
  @brief Length of the whole TID an XTID header describes, in 16-bit words
  @param xtid_header XTID header word
  @return Words from TID word 0 through the end of the last segment the header announces
  @details Reading the TID takes two passes because of this: the header has to be in hand
  before the length of what follows it is known (TDS 16.2)
 */
inline size_t xtidTotalWords(const uint16_t xtid_header)
{
    size_t words = XTID_FIXED_WORDS + xtidSerialBits(xtid_header) / 16;
    if (xtid_header & XTID_OPTIONAL_COMMAND_SUPPORT) {
        words += 1;
    }
    if (xtid_header & XTID_BLOCKWRITE_BLOCKERASE) {
        words += 4;
    }
    if (xtid_header & XTID_USER_MEMORY_BLOCKPERMALOCK) {
        words += 2;
    }
    if (xtid_header & XTID_LOCK_BIT) {
        words += 1;
    }
    return words;
}

//! @brief Words at the front of a TID that name the chip rather than the individual tag
//! @details The allocation class identifier, the mask designer and the model number, then the
//! XTID header. Every tag of one model carries the same bytes there
constexpr size_t TID_CHIP_WORDS{3};

/*!
  @brief Can a mask built from this TID tell one tag of the chip from another?
  @param tid TID bytes, starting at word 0
  @param len Their length in bytes
  @return True when the TID reaches past what names the chip and carries something there
  @details What makes a TID unique is the serial, and the serial begins at word 3 in both of
  the layouts in use: after the XTID header when a tag has one (TDS 16.2), and at TID 30h on
  chips that keep a serial without advertising it in the header. Asking what the mask can
  actually tell apart therefore answers for both, where the XTID indicator answers only for the
  first
  @note A chip with no serial there is not addressed by such a mask at all, so getting this
  wrong ends in a tag that does not answer rather than in the wrong tag being addressed
  @note The rule this replaced came from the XTID indicator, on the strength of TDS 1.5 16
  calling everything past address 1Fh undefined without one. That sentence is not in TDS 1.13
  or anything after it, and chips carry a serial there regardless
 */
inline bool tidTellsTagsApart(const uint8_t* tid, const size_t len)
{
    if (tid == nullptr || len <= TID_CHIP_WORDS * 2) {
        return false;
    }
    for (size_t i = TID_CHIP_WORDS * 2; i < len; ++i) {
        if (tid[i] != 0x00) {
            return true;
        }
    }
    return false;
}

/*!
  @brief Resolve a named mask designer from its identifier
  @param mdid Mask-designer identifier (9 bits)
  @return Vendor, or Vendor::Unknown when the identifier is not one we list
 */
inline Vendor resolveVendor(const uint16_t mdid)
{
    size_t count{};
    const detail::VendorEntry* entries = detail::vendorEntries(count);
    for (size_t i = 0; i < count; ++i) {
        if (static_cast<uint16_t>(entries[i].vendor) == mdid) {
            return entries[i].vendor;
        }
    }
    return Vendor::Unknown;
}

/*!
  @brief Resolve a chip from its mask designer and model number
  @param vendor Named mask designer
  @param model_number Tag model number
  @return Chip, or Chip::Unknown when the pair is not one we have confirmed
  @note A tag whose chip comes back Unknown still reports its raw mask-designer identifier and
  model number
 */
inline Chip resolveChip(const Vendor vendor, const uint16_t model_number)
{
    switch (vendor) {
        case Vendor::Impinj:
            switch (model_number) {
                // Monza 4 datasheet V10.0 Table 4-8
                case 0x100:
                    return Chip::ImpinjMonza4D;
                case 0x105:
                    return Chip::ImpinjMonza4QT;
                case 0x10C:
                    return Chip::ImpinjMonza4E;
                case 0x114:
                    return Chip::ImpinjMonza4i;
                // M700 Series datasheet v6.4 Table 23
                case 0x190:
                    return Chip::ImpinjM750;
                case 0x191:
                    return Chip::ImpinjM730;
                case 0x1A0:
                    return Chip::ImpinjM770;
                // M780 / M781 datasheet v1.1 Table 21
                case 0x1C0:
                    return Chip::ImpinjM780;
                case 0x1C1:
                    return Chip::ImpinjM781;
                default:
                    return Chip::Unknown;
            }
        case Vendor::Alien:
            switch (model_number) {
                case 0x412:
                    return Chip::AlienHiggs3;  // Higgs 3 IC datasheet, Table 1
                case 0x414:
                    // Higgs 4 IC Datasheet V1.3.6 TID table: "1 / 0x3414 / Alien Manufacturer
                    // Code lower bits, Higgs 4 IC"
                    return Chip::AlienHiggs4;
                case 0x821:
                    // Neither Alien nor the GS1 registry publishes this one; it comes from
                    // third-party tables, and was read off a real Higgs 9 whose TID is
                    // E2803821 2000 6820042E3E3B
                    return Chip::AlienHiggs9;
                default:
                    return Chip::Unknown;
            }
        case Vendor::NXP:
            switch (model_number) {
                case 0x80A:
                    return Chip::NxpUcodeG2iM;  // UCODE G2iM datasheet Rev 3.7
                case 0x80B:
                    return Chip::NxpUcodeG2iMPlus;
                case 0x890:
                    return Chip::NxpUcode7;  // SL3S1204 Rev4.0 Table 12: "E28068902000"
                case 0x894:
                    return Chip::NxpUcode8;  // SL3S1205/1215 datasheet
                case 0x994:
                    return Chip::NxpUcode8m;  // SL3S1205_15 Rev3.6, the 8m of the same datasheet
                case 0xA16:
                case 0xA96:
                    // The same change of model number, made to this part at the same time. Its
                    // datasheet gives both TIDs outright: "HEX E280 6A16 ... or HEX E280 6A96"
                    // (SL3S1216 Rev3.3 Table 5 note 1)
                    return Chip::NxpUcode9xe;
                case 0x995:
                case 0x915:
                    // The model number changed without the part being renamed, and the
                    // datasheet gives both TIDs: "HEX E280 6995 ... or HEX E280 6915 ..."
                    // (SL3S1206 Rev3.5 Table 6 note 1)
                    return Chip::NxpUcode9;
                default:
                    return Chip::Unknown;
            }
        default:
            return Chip::Unknown;
    }
}

/*!
  @brief Fill in what the tag left unsaid from what its chip is known to hold
  @param[in,out] tag Tag whose TID has been decoded
  @details The XTID is where a tag states its own memory sizes, and hardly any tag does: in
  practice an XTID carries a serial number and nothing else. What the chip is known to hold is
  the only thing left, and the mask designer and model number are what name the chip
 */
inline void fillSizesFromChip(Tag& tag)
{
    if (tag.user_memory_bits == 0) {
        // A chip that shares a pool has no one size to look up, so the PC is asked first
        tag.user_memory_bits = chipSharedUserMemoryBits(tag.chip, tag.pc);
    }
    if (tag.user_memory_bits == 0) {
        tag.user_memory_bits = chipUserMemoryBits(tag.chip);
    }
    if (tag.epc_max_bits == 0) {
        tag.epc_max_bits = chipEpcMaxBits(tag.chip);
    }
    if (tag.permalock_block_bits == 0) {
        tag.permalock_block_bits = chipPermalockBlockBits(tag.chip);
    }
}

/*!
  @brief Name the chip from the two fixed words of a TID
  @param tid TID bytes starting at word 0
  @param len Their length in bytes
  @return Chip, or Chip::Unknown where the bytes do not name one
  @details The mask-designer identifier and the model number sit in the first two words, which
  is what makes the chip known before the rest of the TID has been read
 */
inline Chip chipFromTid(const uint8_t* tid, const size_t len)
{
    if (tid == nullptr || len < TID_FIXED_WORDS * 2 || tid[0] != TID_CLASS_EPCGLOBAL) {
        return Chip::Unknown;
    }
    const uint16_t mdid  = static_cast<uint16_t>(((tid[1] & 0x1F) << 4) | (tid[2] >> 4));
    const uint16_t model = static_cast<uint16_t>(((tid[2] & 0x0F) << 8) | tid[3]);
    return resolveChip(resolveVendor(mdid), model);
}

/*!
  @struct TidReadPlan
  @brief What to read next of a tag's TID, given what has been read of it so far
 */
struct TidReadPlan {
    uint16_t word_address{};  //!< Word address the next read starts at
    uint16_t words{};         //!< Words to read, zero once nothing is left to read
    bool fits{true};          //!< False when the header describes more than a Tid can hold
};

/*!
  @brief Say what to read next of a TID
  @param tid Bytes read so far, starting at TID word 0
  @param len Their length in bytes
  @return The next read, asking for no words once the TID is complete
  @details A TID cannot be read in one go because each part says how much more there is: the two
  fixed words say whether an XTID follows, and the XTID header says how long the rest is (TDS
  16.1 and 16.2). Calling this after every read walks the whole TID in
  @note A header describing more than TID_MAX_BYTES gives fits == false. Nothing is read for it,
  since what came back could not be kept
 */
inline TidReadPlan tidReadPlan(const uint8_t* tid, const size_t len)
{
    TidReadPlan plan{};
    if (len < TID_FIXED_WORDS * 2) {
        plan.words = static_cast<uint16_t>(TID_FIXED_WORDS);
        return plan;
    }
    if (!tidHasXtid(tid, len)) {
        // No header to say how long the TID is. The chip may still be one that keeps a serial
        // past the words that name it, and that serial is the whole point of reading a TID
        const size_t total = chipTidWords(chipFromTid(tid, len));
        const size_t held  = len / 2;
        if (total > held) {
            plan.word_address = static_cast<uint16_t>(held);
            plan.words        = static_cast<uint16_t>(total - held);
        }
        return plan;
    }
    if (len < XTID_FIXED_WORDS * 2) {
        plan.word_address = static_cast<uint16_t>(TID_FIXED_WORDS);
        plan.words        = static_cast<uint16_t>(XTID_FIXED_WORDS - TID_FIXED_WORDS);
        return plan;
    }
    const uint16_t header = static_cast<uint16_t>((tid[TID_FIXED_WORDS * 2] << 8) | tid[TID_FIXED_WORDS * 2 + 1]);
    const size_t total    = xtidTotalWords(header);
    if (total * 2 > TID_MAX_BYTES) {
        plan.fits = false;
        return plan;
    }
    const size_t have = len / 2;
    if (have < total) {
        plan.word_address = static_cast<uint16_t>(have);
        plan.words        = static_cast<uint16_t>(total - have);
    }
    return plan;
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
    tag.mdid              = static_cast<uint16_t>(((tid[1] & 0x1F) << 4) | (tid[2] >> 4));
    tag.vendor            = resolveVendor(tag.mdid);
    tag.model_number      = static_cast<uint16_t>(((tid[2] & 0x0F) << 8) | tid[3]);
    tag.chip              = resolveChip(tag.vendor, tag.model_number);

    tag.serial_bits              = 0;
    tag.epc_max_bits             = 0;
    tag.user_memory_bits         = 0;
    tag.permalock_block_bits     = 0;
    tag.supports_block_permalock = false;
    if (!tag.has_xtid || len < XTID_FIXED_WORDS * 2) {
        fillSizesFromChip(tag);
        return true;
    }

    const uint16_t header = detail::word_at(tid, 2);
    tag.serial_bits       = xtidSerialBits(header);

    // The segments sit past the serial, in the order the header lists them. Each one is stepped
    // over whether or not the caller read far enough to hold it, so the offsets of the segments
    // below it stay right even when only part of the TID is in hand
    size_t word = XTID_FIXED_WORDS + tag.serial_bits / 16;
    if (header & XTID_OPTIONAL_COMMAND_SUPPORT) {
        if ((word + 1) * 2 <= len) {
            const uint16_t ocs = detail::word_at(tid, word);
            // Max EPC Size is what fits in the length field of the PC, so it counts words
            tag.epc_max_bits             = static_cast<uint32_t>(ocs & 0x1F) * 16U;
            tag.supports_block_permalock = (ocs & 0x1000) != 0;
        }
        word += 1;
    }
    if (header & XTID_BLOCKWRITE_BLOCKERASE) {
        word += 4;
    }
    if (header & XTID_USER_MEMORY_BLOCKPERMALOCK) {
        if ((word + 2) * 2 <= len) {
            // The lower address holds bits 31 to 16 and the higher one bits 15 to 0, so the
            // block size comes first and the user memory size second (TDS Table 16-2)
            tag.permalock_block_bits = static_cast<uint32_t>(detail::word_at(tid, word)) * 16U;
            tag.user_memory_bits     = static_cast<uint32_t>(detail::word_at(tid, word + 1)) * 16U;
        }
        word += 2;
    }
    fillSizesFromChip(tag);
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

    LockSetting() = default;
    // Written out because the members carry defaults, which stops C++11 from treating the
    // struct as an aggregate and so rules out LockSetting{target, action} without it
    LockSetting(const LockTarget t, const LockAction a) : target{t}, action{a}
    {
    }
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
    uint8_t q{};                   //!< Q value: a tag answers in one of 2^Q slots
    Session session{Session::S0};  //!< Session whose inventoried flag qualifies the round
    Target target{Target::A};      //!< Inventoried flag value that is invited to answer
    /*!
      Which tags the round invites in terms of SL. Reported so that a select mask can be aimed
      at something the round actually looks at: aiming one at SL while this reads All stores a
      mask that filters nothing
     */
    SelectFilter filter{SelectFilter::All};
};

//! @brief Longest select mask the module accepts, the length field being 8 bits wide
constexpr size_t SELECT_MASK_MAX_BYTES{32};

/*!
  @struct QTParameters
  @brief QT control word of an Impinj Monza 4QT
  @details The chip keeps two memory maps and shows one at a time. Which one it shows, and
  whether it insists on being close to the antenna before it can be switched, is what this
  word holds (Monza 4 V10.0 Table 2-3)
 */
struct QTParameters {
    //! @brief The tag answers at a reduced range once it is open or about to be
    bool short_range{};
    //! @brief The tag shows its public memory map rather than its private one
    bool public_memory{};
};

/*!
  @struct SelectParameter
  @brief The mask the reader holds, and how it is matched
  @details Reading it back is the only way to see what the reader is actually addressing. A
  Select produces no reply from any tag (EPC Gen2 v2.1 6.3.2.12.1.1), so nothing else confirms
  that the reader and the caller agree on which tag is meant
 */
struct SelectParameter {
    uint8_t target{};            //!< Target: which flag the action acts on, 100 being SL
    uint8_t action{};            //!< Action: what matching and non-matching tags are set to
    Bank bank{Bank::Epc};        //!< Bank the mask is matched against
    uint32_t pointer_bits{};     //!< Where the mask starts, as a bit address inside the bank
    uint8_t mask_length_bits{};  //!< Length of the mask in bits
    bool truncate{};             //!< Whether a matching tag replies with only the bits past the mask
    std::array<uint8_t, SELECT_MASK_MAX_BYTES> mask{};  //!< Mask bytes
    uint8_t mask_size{};                                //!< Bytes of mask in use

    //! @brief Mask as uppercase hex
    std::string maskAsString() const;
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

inline std::string SelectParameter::maskAsString() const
{
    return detail::to_hex(mask.data(), mask_size);
}

inline std::string Tid::toString() const
{
    return detail::to_hex(data.data(), size);
}

inline std::string Tag::vendorAsString() const
{
    size_t count{};
    const detail::VendorEntry* entries = detail::vendorEntries(count);
    for (size_t i = 0; i < count; ++i) {
        if (entries[i].vendor == vendor) {
            return entries[i].name;
        }
    }
    return "Unknown";
}

inline std::string Tag::chipAsString() const
{
    switch (chip) {
        case Chip::AlienHiggs3:
            return "Alien Higgs-3";
        case Chip::AlienHiggs9:
            return "Alien Higgs-9";
        case Chip::AlienHiggs4:
            return "Alien Higgs-4";
        case Chip::ImpinjMonza4D:
            return "Impinj Monza 4D";
        case Chip::ImpinjMonza4E:
            return "Impinj Monza 4E";
        case Chip::ImpinjMonza4i:
            return "Impinj Monza 4i";
        case Chip::ImpinjMonza4QT:
            return "Impinj Monza 4QT";
        case Chip::ImpinjM730:
            return "Impinj M730";
        case Chip::ImpinjM750:
            return "Impinj M750";
        case Chip::ImpinjM770:
            return "Impinj M770";
        case Chip::ImpinjM780:
            return "Impinj M780";
        case Chip::ImpinjM781:
            return "Impinj M781";
        case Chip::NxpUcodeG2iM:
            return "NXP UCODE G2iM";
        case Chip::NxpUcodeG2iMPlus:
            return "NXP UCODE G2iM+";
        case Chip::NxpUcode7:
            return "NXP UCODE 7";
        case Chip::NxpUcode8:
            return "NXP UCODE 8";
        case Chip::NxpUcode8m:
            return "NXP UCODE 8m";
        case Chip::NxpUcode9:
            return "NXP UCODE 9";
        case Chip::NxpUcode9xe:
            return "NXP UCODE 9xe";
        default:
            return "Unknown";
    }
}

//! @brief Initial value of the EPC Gen2 CRC-16
constexpr uint16_t GEN2_CRC16_INIT{0xFFFF};
//! @brief Polynomial of the EPC Gen2 CRC-16 (x^16 + x^12 + x^5 + 1)
constexpr uint16_t GEN2_CRC16_POLYNOMIAL{0x1021};
//! @brief Final xor value of the EPC Gen2 CRC-16
constexpr uint16_t GEN2_CRC16_XOROUT{0xFFFF};

/*!
  @brief Calculate the EPC Gen2 CRC-16
  @param data Data to calculate over
  @param len Length in bytes
  @return CRC-16
 */
inline uint16_t gen2_crc16(const uint8_t* data, const size_t len)
{
    m5::utility::CRC16 crc{GEN2_CRC16_INIT, GEN2_CRC16_POLYNOMIAL, false, false, GEN2_CRC16_XOROUT};
    return crc.range(data, len);
}

/*!
  @brief Verify the CRC-16 reported with a detected tag
  @param tag Tag to verify
  @return True if the CRC-16 recalculated from PC and EPC matches the reported one
  @note The Gen2 CRC-16 covers the PC followed by the EPC
 */
inline bool verify_tag_crc(const Tag& tag)
{
    if (tag.epc.empty()) {
        return false;
    }
    // The Gen2 CRC-16 covers the PC followed by the EPC, so they are laid out contiguously
    uint8_t buf[2 + EPC_MAX_BYTES]{};
    buf[0] = static_cast<uint8_t>(tag.pc >> 8);
    buf[1] = static_cast<uint8_t>(tag.pc & 0xFF);
    for (size_t i = 0; i < tag.epc.size; ++i) {
        buf[2 + i] = tag.epc[i];
    }
    return gen2_crc16(buf, tag.epc.size + 2U) == tag.crc;
}

/*!
  @brief EPC length in 16-bit words encoded in the PC
  @param pc Protocol Control
  @return Number of 16-bit words comprising the EPC
  @warning The PC a tag backscatters is not the one in its memory. A tag that sends an XPC
  ahead of its EPC raises the length here by the word or two that takes, while leaving EPC
  memory 10h to 14h alone (EPC Gen2 v2.1 6.3.2.1.2.2). Counted from a backscattered PC this is
  therefore the length of what came over the air, not the length of the EPC
 */
inline uint8_t pcEPCLengthWords(const uint16_t pc)
{
    return static_cast<uint8_t>((pc >> 11) & 0x1F);
}

/*!
  @brief User Memory Indicator of the PC
  @param pc Protocol Control
  @return True if the bit is asserted
  @warning This does not say whether the tag has a User bank. EPC Gen2 v2.1 6.3.2.1.2.1 lets a
  chip either fix the bit at what it was built with or compute it, and a chip that computes it
  ORs together bits 03h to 07h of File_0 and nothing else, so it reports zero for a bank it has
  but whose first five bits have never been written. A UCODE G2iM was measured reporting zero
  with 512 usable bits
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
  @brief Toggle bit of the PC
  @param pc Protocol Control
  @return True when the EPC bank holds an ISO 15961 UII rather than an EPC
  @details Of the bits below the length this is the one that can always be believed: it is
  never one of the bits a tag substitutes from its XPC (GS1 TDS 2.3 9.4). A tag asserting it
  is telling the reader that what follows the PC is not an EPC, and nothing here decodes it as
  one
 */
inline bool pcToggle(const uint16_t pc)
{
    return (pc & 0x0100) != 0;
}

/*!
  @brief Numbering System Identifier of the PC
  @param pc Protocol Control
  @return NSI (0x000 for EPCglobal)
  @warning These bits cannot be relied on to be the identifier. What is spoken of as the PC
  word is the top eight bits of the Protocol Control joined to the bottom eight of the Extended
  Protocol Control, and a tag substitutes the latter for its own flags whether or not it goes
  on to send an XPC word (GS1 TDS 2.3 9.4). Only pcToggle() sits outside what is substituted
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
