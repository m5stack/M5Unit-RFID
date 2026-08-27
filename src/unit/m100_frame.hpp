/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file m100_frame.hpp
  @brief Frame codec for the magicRF M100 family (JRD-4035 / JRD-100 / R200)
  @note Intentionally free of M5UnitComponent so that it can be built and tested
  without the ESP32 toolchain
*/
#ifndef M5_UNIT_RFID_UNIT_M100_FRAME_HPP
#define M5_UNIT_RFID_UNIT_M100_FRAME_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "uhf/uhf.hpp"

namespace m5 {
namespace unit {
/*!
  @namespace m100
  @brief magicRF M100 chip specific definitions
  @details The frame format, command codes and receiver settings belong to the chip, not to any
  one module built around it. JRD-4035, JRD-100, R200 and YRM100 all carry an M100 and speak
  this, differing only in the two bytes that delimit a frame
 */
namespace m100 {

/*!
  @namespace jrd
  @brief Framing the JRD-4035 and JRD-100 modules use
  @details The bytes that delimit a frame are the one thing that varies between modules built
  around this chip, so each family gets its own pair. The R200 uses 0xAA and 0xDD and everything
  else about it is the same, which is why they are grouped here rather than named for the chip
 */
namespace jrd {
//! @brief Frame header
constexpr uint8_t FRAME_HEADER{0xBB};
//! @brief Frame end
constexpr uint8_t FRAME_END{0x7E};
}  // namespace jrd

/*!
  @brief Calculate the frame checksum
  @param body Pointer to the Type byte (the checksum covers Type through the last Parameter byte)
  @param len Length in bytes
  @return Least significant byte of the sum
 */
inline uint8_t checksum(const uint8_t* body, const size_t len)
{
    uint32_t sum{};
    for (size_t i = 0; i < len; ++i) {
        sum += body[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

//! @brief Maximum parameter length accepted for a frame
constexpr uint16_t MAX_PARAMETER_LENGTH{512};

/*!
  @brief Build a command frame
  @param[out] out Built frame
  @param type Frame type (0x00: command)
  @param command Command code
  @param param Parameter (nullptr if none)
  @param param_len Parameter length
  @param header Frame header (0xBB for JRD-4035, 0xAA for R200)
  @param end Frame end (0x7E for JRD-4035, 0xDD for R200)
  @return True if successful
 */
inline bool build_frame(std::vector<uint8_t>& out, const uint8_t type, const uint8_t command, const uint8_t* param,
                        const uint16_t param_len, const uint8_t header = jrd::FRAME_HEADER,
                        const uint8_t end = jrd::FRAME_END)
{
    if (param_len > MAX_PARAMETER_LENGTH || (param == nullptr && param_len != 0)) {
        return false;
    }

    out.clear();
    out.reserve(static_cast<size_t>(param_len) + 7);
    out.push_back(header);
    out.push_back(type);
    out.push_back(command);
    out.push_back(static_cast<uint8_t>(param_len >> 8));
    out.push_back(static_cast<uint8_t>(param_len & 0xFF));
    for (uint16_t i = 0; i < param_len; ++i) {
        out.push_back(param[i]);
    }
    // The checksum covers Type through the last Parameter byte (index 1 onwards)
    out.push_back(checksum(out.data() + 1, out.size() - 1));
    out.push_back(end);
    return true;
}

//! @brief Fixed part of a frame (Header, Type, Command, PL_MSB, PL_LSB, Checksum, End)
constexpr size_t FRAME_OVERHEAD{7};
//! @brief Offset of the Type byte
constexpr size_t FRAME_TYPE_OFFSET{1};
//! @brief Offset of the Parameter
constexpr size_t FRAME_PARAMETER_OFFSET{5};

//! @brief Frame type: command (host to module)
constexpr uint8_t TYPE_COMMAND{0x00};
//! @brief Frame type: response (module to host)
constexpr uint8_t TYPE_RESPONSE{0x01};
//! @brief Frame type: notification (module to host)
constexpr uint8_t TYPE_NOTIFICATION{0x02};

/*!
  @struct Frame
  @brief Parsed frame
 */
struct Frame {
    uint8_t type{};                    //!< Frame type
    uint8_t command{};                 //!< Command code
    std::vector<uint8_t> parameter{};  //!< Parameter
};

/*!
  @brief Parse a frame
  @param[out] out Parsed frame
  @param raw Raw bytes (a whole frame including the header and the end)
  @param len Length of raw
  @param header Expected frame header
  @param end Expected frame end
  @return True if the frame is well-formed and the checksum matches
 */
inline bool parse_frame(Frame& out, const uint8_t* raw, const size_t len, const uint8_t header = jrd::FRAME_HEADER,
                        const uint8_t end = jrd::FRAME_END)
{
    if (raw == nullptr || len < FRAME_OVERHEAD) {
        return false;
    }
    if (raw[0] != header || raw[len - 1] != end) {
        return false;
    }

    const uint16_t param_len = static_cast<uint16_t>((raw[3] << 8) | raw[4]);
    if (param_len > MAX_PARAMETER_LENGTH || len != static_cast<size_t>(param_len) + FRAME_OVERHEAD) {
        return false;
    }
    // The checksum covers Type through the last Parameter byte
    if (raw[len - 2] != checksum(raw + FRAME_TYPE_OFFSET, len - 3)) {
        return false;
    }

    out.type    = raw[1];
    out.command = raw[2];
    out.parameter.assign(raw + FRAME_PARAMETER_OFFSET, raw + FRAME_PARAMETER_OFFSET + param_len);
    return true;
}

/*!
  @enum FrameExtract
  @brief What came of trying to take a frame out of the bytes received so far
 */
enum class FrameExtract : uint8_t {
    Ok,        //!< A frame was taken out, and the bytes it was made of are gone from the buffer
    NeedMore,  //!< What is in the buffer could still turn into a frame once more bytes arrive
};

/*!
  @brief Take the first whole frame out of received bytes
  @param[out] out Frame, filled in only when Ok is returned
  @param[in,out] buffer Bytes received so far. What was used, and what was thrown away, is
  removed; everything else is left for the next call
  @param[out] discarded Bytes thrown away before a frame could be found
  @param header Frame header byte
  @param end Frame end byte
  @return Ok when a frame came out, NeedMore when the buffer has to grow first
  @details The header byte also occurs inside the data a tag notification carries, so a byte
  that looks like the start of a frame may be the middle of one. Rather than trusting it, the
  length, the checksum and the end byte are all made to agree; when they do not, one byte is
  thrown away and the search goes on from the next. Bytes are never read again to do this, so a
  frame that arrived in pieces costs nothing to wait for
 */
inline FrameExtract extract_frame(Frame& out, std::vector<uint8_t>& buffer, size_t& discarded, const uint8_t header,
                                  const uint8_t end)
{
    discarded = 0;
    for (;;) {
        // Nothing before a header byte can be the start of a frame
        size_t start = 0;
        while (start < buffer.size() && buffer[start] != header) {
            ++start;
        }
        if (start) {
            discarded += start;
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(start));
        }
        if (buffer.size() < FRAME_PARAMETER_OFFSET) {
            return FrameExtract::NeedMore;
        }
        const uint16_t param_len = static_cast<uint16_t>((buffer[3] << 8) | buffer[4]);
        const size_t total       = static_cast<size_t>(param_len) + FRAME_OVERHEAD;
        if (param_len > MAX_PARAMETER_LENGTH) {
            // A length no frame can have says this header byte was data
            discarded += 1;
            buffer.erase(buffer.begin());
            continue;
        }
        if (buffer.size() < total) {
            return FrameExtract::NeedMore;
        }
        if (parse_frame(out, buffer.data(), total, header, end)) {
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(total));
            return FrameExtract::Ok;
        }
        // The checksum or the end byte disagreed, so this was not a frame either
        discarded += 1;
        buffer.erase(buffer.begin());
    }
}

//! @brief Command code of the single polling notification
constexpr uint8_t COMMAND_SINGLE_POLLING{0x22};
//! @brief Command code of the multiple polling notification
constexpr uint8_t COMMAND_MULTIPLE_POLLING{0x27};

/*!
  @name Command codes of the operations an error code can name
 */
///@{
constexpr uint8_t COMMAND_READ_TAG_MEMORY{0x39};
constexpr uint8_t COMMAND_WRITE_TAG_MEMORY{0x49};
constexpr uint8_t COMMAND_KILL_TAG{0x65};
constexpr uint8_t COMMAND_LOCK_TAG_MEMORY{0x82};
constexpr uint8_t COMMAND_BLOCK_PERMALOCK{0xD3};
///@}
//! @brief Fixed part of a tag notification (RSSI, PC and CRC)
constexpr size_t TAG_NOTIFICATION_OVERHEAD{5};

/*!
  @brief Parse the parameter of a tag notification
  @param[out] out Parsed tag
  @param param Parameter of the notification frame (RSSI, PC, EPC and CRC)
  @param len Parameter length
  @return True if successful
  @note The EPC length is derived from len, so a variable length EPC is supported
 */
inline bool parse_tag_notification(m5::uhf::Tag& out, const uint8_t* param, const size_t len)
{
    if (param == nullptr || len <= TAG_NOTIFICATION_OVERHEAD) {
        return false;
    }

    const size_t epc_len = len - TAG_NOTIFICATION_OVERHEAD;
    // An EPC longer than the standard allows means the frame is not what it claims to be
    if (!out.epc.assign(param + 3, epc_len)) {
        return false;
    }
    out.rssi = static_cast<int8_t>(param[0]);
    out.pc   = static_cast<uint16_t>((param[1] << 8) | param[2]);
    out.crc  = static_cast<uint16_t>((param[3 + epc_len] << 8) | param[4 + epc_len]);
    return true;
}

//! @brief MemBank values of the Select parameter and of the read/write commands
constexpr uint8_t MEMBANK_RESERVED{0x00};
constexpr uint8_t MEMBANK_EPC{0x01};
constexpr uint8_t MEMBANK_TID{0x02};
constexpr uint8_t MEMBANK_USER{0x03};

//! @brief Truncate field of the Select parameter
constexpr uint8_t SELECT_TRUNCATE_OFF{0x00};
constexpr uint8_t SELECT_TRUNCATE_ON{0x80};

//! @brief Select modes of Set Select Mode (0x12)
constexpr uint8_t SELECT_MODE_ALWAYS{0x00};         //!< Select before every operation, inventory included
constexpr uint8_t SELECT_MODE_NEVER{0x01};          //!< Never send Select
constexpr uint8_t SELECT_MODE_NON_INVENTORY{0x02};  //!< Select before everything except inventory

//! @brief Longest write the module accepts, in 16-bit words
constexpr size_t WRITE_MAX_WORDS{32};
//! @brief Longest mask the Select parameter accepts, in bits
constexpr size_t SELECT_MASK_MAX_BITS{255};

/*!
  @brief Build the SelParam byte of Set Select Parameter
  @param target Target (3 bits)
  @param action Action (3 bits)
  @param membank Memory bank (2 bits)
  @return SelParam
 */
inline uint8_t select_parameter_byte(const uint8_t target, const uint8_t action, const uint8_t membank)
{
    return static_cast<uint8_t>(((target & 0x07) << 5) | ((action & 0x07) << 2) | (membank & 0x03));
}

/*!
  @brief Build the parameter of Set Select Parameter (0x0C)
  @param[out] out Parameter
  @param sel_param SelParam byte
  @param pointer_bits Start of the mask as a bit address inside the bank
  @param mask_length_bits Mask length in bits
  @param truncate SELECT_TRUNCATE_OFF or SELECT_TRUNCATE_ON
  @param mask Mask bytes
  @param mask_len Length of mask in bytes
  @return True if successful
  @note Reserved memory cannot be selected on: a tag ignores a Select that names it
 */
inline bool build_select_parameter(std::vector<uint8_t>& out, const uint8_t sel_param, const uint32_t pointer_bits,
                                   const uint8_t mask_length_bits, const uint8_t truncate, const uint8_t* mask,
                                   const size_t mask_len)
{
    if ((sel_param & 0x03) == MEMBANK_RESERVED) {
        return false;
    }
    if (mask_len == 0 || mask == nullptr || mask_length_bits == 0) {
        return false;
    }
    // SELECT_MASK_MAX_BITS is the whole range of the length field, so the type already keeps
    // the mask inside it; only the mask actually being that long still has to be checked
    if ((mask_length_bits + 7U) / 8U > mask_len) {
        return false;
    }

    out.clear();
    out.reserve(7 + mask_len);
    out.push_back(sel_param);
    out.push_back(static_cast<uint8_t>(pointer_bits >> 24));
    out.push_back(static_cast<uint8_t>(pointer_bits >> 16));
    out.push_back(static_cast<uint8_t>(pointer_bits >> 8));
    out.push_back(static_cast<uint8_t>(pointer_bits));
    out.push_back(mask_length_bits);
    out.push_back(truncate);
    out.insert(out.end(), mask, mask + mask_len);
    return true;
}

/*!
  @brief Parse the answer to Get Select Parameter (0x0B)
  @param[out] out Select parameter
  @param param Parameter of the response frame
  @param len Length of param
  @return True if successful
 */
inline bool parse_select_parameter(m5::uhf::SelectParameter& out, const uint8_t* param, const size_t len)
{
    // SelParam, a four byte pointer, the mask length, the truncate flag, then the mask itself
    constexpr size_t HEADER_LENGTH{7};
    if (param == nullptr || len < HEADER_LENGTH) {
        return false;
    }
    const size_t mask_len = len - HEADER_LENGTH;
    if (mask_len > m5::uhf::SELECT_MASK_MAX_BYTES) {
        return false;
    }
    out.target       = static_cast<uint8_t>((param[0] >> 5) & 0x07);
    out.action       = static_cast<uint8_t>((param[0] >> 2) & 0x07);
    out.bank         = static_cast<m5::uhf::Bank>(param[0] & 0x03);
    out.pointer_bits = (static_cast<uint32_t>(param[1]) << 24) | (static_cast<uint32_t>(param[2]) << 16) |
                       (static_cast<uint32_t>(param[3]) << 8) | param[4];
    out.mask_length_bits = param[5];
    out.truncate         = param[6] != SELECT_TRUNCATE_OFF;
    out.mask.fill(0);
    for (size_t i = 0; i < mask_len; ++i) {
        out.mask[i] = param[HEADER_LENGTH + i];
    }
    out.mask_size = static_cast<uint8_t>(mask_len);
    return true;
}

/*!
  @brief Build the parameter of Read Tag Memory Area (0x39)
  @param[out] out Parameter
  @param access_password Access password, 0 when the tag has none
  @param membank Memory bank
  @param word_address Start address in 16-bit words
  @param word_count Number of 16-bit words
  @return True if successful
  @note A word_count of zero would mean "to the end of the bank" in EPC Gen2, but the module
  does not document that, so it is refused here
 */
inline bool build_read_tag_memory(std::vector<uint8_t>& out, const uint32_t access_password, const uint8_t membank,
                                  const uint16_t word_address, const uint16_t word_count)
{
    if (word_count == 0) {
        return false;
    }
    out.clear();
    out.reserve(9);
    out.push_back(static_cast<uint8_t>(access_password >> 24));
    out.push_back(static_cast<uint8_t>(access_password >> 16));
    out.push_back(static_cast<uint8_t>(access_password >> 8));
    out.push_back(static_cast<uint8_t>(access_password));
    out.push_back(membank);
    out.push_back(static_cast<uint8_t>(word_address >> 8));
    out.push_back(static_cast<uint8_t>(word_address));
    out.push_back(static_cast<uint8_t>(word_count >> 8));
    out.push_back(static_cast<uint8_t>(word_count));
    return true;
}

/*!
  @brief Build the parameter of Write Tag Memory Area (0x49)
  @param[out] out Parameter
  @param access_password Access password, 0 when the tag has none
  @param membank Memory bank
  @param word_address Start address in 16-bit words
  @param data Bytes to write
  @param len Length of data, which must be even and at most 64
  @return True if successful
 */
inline bool build_write_tag_memory(std::vector<uint8_t>& out, const uint32_t access_password, const uint8_t membank,
                                   const uint16_t word_address, const uint8_t* data, const size_t len)
{
    if (data == nullptr || len == 0 || (len % 2) != 0 || len / 2 > WRITE_MAX_WORDS) {
        return false;
    }
    const uint16_t word_count = static_cast<uint16_t>(len / 2);
    if (!build_read_tag_memory(out, access_password, membank, word_address, word_count)) {
        return false;
    }
    out.insert(out.end(), data, data + len);
    return true;
}

/*!
  @brief Build the parameter of Lock (0x82)
  @param[out] out Parameter
  @param access_password Access password
  @param payload 20-bit lock payload, see m5::uhf::buildLockPayload
  @return True if successful
 */
inline bool build_lock_tag(std::vector<uint8_t>& out, const uint32_t access_password, const uint32_t payload)
{
    if (payload > 0x000FFFFFU) {
        return false;
    }
    out.clear();
    out.reserve(7);
    out.push_back(static_cast<uint8_t>(access_password >> 24));
    out.push_back(static_cast<uint8_t>(access_password >> 16));
    out.push_back(static_cast<uint8_t>(access_password >> 8));
    out.push_back(static_cast<uint8_t>(access_password));
    out.push_back(static_cast<uint8_t>(payload >> 16));
    out.push_back(static_cast<uint8_t>(payload >> 8));
    out.push_back(static_cast<uint8_t>(payload));
    return true;
}

/*!
  @brief Build the parameter of Kill (0x65)
  @param[out] out Parameter
  @param kill_password Kill password
  @return True if successful
  @note A tag whose kill password is zero refuses to be killed, so zero is refused here
 */
inline bool build_kill_tag(std::vector<uint8_t>& out, const uint32_t kill_password)
{
    if (kill_password == 0) {
        return false;
    }
    out.clear();
    out.reserve(4);
    out.push_back(static_cast<uint8_t>(kill_password >> 24));
    out.push_back(static_cast<uint8_t>(kill_password >> 16));
    out.push_back(static_cast<uint8_t>(kill_password >> 8));
    out.push_back(static_cast<uint8_t>(kill_password));
    return true;
}

/*!
  @struct TagOperationResult
  @brief What a tag answered to Read, Write, Lock or Kill
  @details All four answer in the same shape: the PC and EPC of the tag that responded, then
  either the words that were read or a single status byte
  @warning data points into the frame it was parsed from and is only valid while that lives
 */
struct TagOperationResult {
    uint16_t pc{};          //!< Protocol Control of the tag that answered
    m5::uhf::Epc epc{};     //!< EPC of the tag that answered
    const uint8_t* data{};  //!< Words read, or the one status byte
    size_t data_len{};      //!< Length of data in bytes
};

//! @brief Status byte a tag returns after a Write, Lock or Kill it carried out
constexpr uint8_t TAG_OPERATION_SUCCESS{0x00};

/*!
  @brief Parse the answer to a tag operation
  @param[out] out Parsed result
  @param param Parameter of the response frame
  @param len Length of param
  @return True if successful
  @details The leading byte counts the PC and EPC that follow it, so what comes after them is
  the payload however long the EPC happened to be
 */
inline bool parse_tag_operation(TagOperationResult& out, const uint8_t* param, const size_t len)
{
    if (param == nullptr || len < 4) {
        return false;
    }
    const size_t ul = param[0];
    // The count covers the two PC bytes as well, so anything smaller cannot be a valid answer
    if (ul < 2 || ul + 1U > len) {
        return false;
    }
    out.pc = static_cast<uint16_t>((param[1] << 8) | param[2]);
    if (!out.epc.assign(param + 3, ul - 2)) {
        return false;
    }
    out.data     = param + 1 + ul;
    out.data_len = len - 1 - ul;
    return true;
}

/*!
  @name Bit positions of the Query parameter fields
  @details The fields pack into the 16-bit word from the top down as DR(1) M(2) TRext(1) Sel(2)
  Session(2) Target(1) Q(4), leaving the bottom three bits unused. The vendor documents the
  layout by worked example: 0x1020 is spelled out as DR=8, M=1, TRext=use pilot tone, Sel=00,
  Session=00, Target=A, Q=4, which only holds with the padding at the bottom
 */
///@{
constexpr uint8_t QUERY_Q_SHIFT{3};
constexpr uint8_t QUERY_TARGET_SHIFT{7};
constexpr uint8_t QUERY_SESSION_SHIFT{8};
constexpr uint8_t QUERY_SEL_SHIFT{10};
///@}

/*!
  @brief Split a Query parameter word into its fields
  @param[out] qp Query parameters
  @param raw Query parameter word
 */
inline void parse_query_parameters(m5::uhf::QueryParameters& qp, const uint16_t raw)
{
    qp.q       = static_cast<uint8_t>((raw >> QUERY_Q_SHIFT) & 0x0F);
    qp.target  = static_cast<m5::uhf::Target>((raw >> QUERY_TARGET_SHIFT) & 0x01);
    qp.session = static_cast<m5::uhf::Session>((raw >> QUERY_SESSION_SHIFT) & 0x03);
    qp.filter  = static_cast<m5::uhf::SelectFilter>((raw >> QUERY_SEL_SHIFT) & 0x03);
}

/*!
  @brief Place the fields we expose into a Query parameter word
  @param qp Query parameters
  @param current Word the module currently holds
  @return Word to write back
  @details DR, M and TRext are carried over from the current word rather than being rebuilt.
  The module supports exactly one value of each, so there is nothing to choose and nothing to
  gain from letting a caller set them wrong
 */
inline uint16_t build_query_parameters(const m5::uhf::QueryParameters& qp, const uint16_t current)
{
    uint16_t raw = current;
    raw &= static_cast<uint16_t>(~((0x03U << QUERY_SEL_SHIFT) | (0x03U << QUERY_SESSION_SHIFT) |
                                   (0x01U << QUERY_TARGET_SHIFT) | (0x0FU << QUERY_Q_SHIFT)));
    raw |= static_cast<uint16_t>((static_cast<uint8_t>(qp.filter) & 0x03) << QUERY_SEL_SHIFT);
    raw |= static_cast<uint16_t>((static_cast<uint8_t>(qp.session) & 0x03) << QUERY_SESSION_SHIFT);
    raw |= static_cast<uint16_t>((static_cast<uint8_t>(qp.target) & 0x01) << QUERY_TARGET_SHIFT);
    raw |= static_cast<uint16_t>((qp.q & 0x0F) << QUERY_Q_SHIFT);
    return raw;
}

/*!
  @enum MixerGain
  @brief Gain of the receiver's mixer
  @details These are the gain steps the M100 receiver offers, not anything the EPC Gen2 standard
  defines, so they live with the chip rather than in the portable vocabulary. Lowering the gain
  shortens the distance the reader works over, which is what makes a tag sitting on the antenna
  readable
 */
enum class MixerGain : uint8_t {
    dB0,   //!< 0dB
    dB3,   //!< 3dB
    dB6,   //!< 6dB
    dB9,   //!< 9dB, the value the module leaves the factory with
    dB12,  //!< 12dB
    dB15,  //!< 15dB
    dB16,  //!< 16dB
};

/*!
  @enum IFGain
  @brief Gain of the receiver's intermediate frequency amplifier
  @details Lowering it shortens the distance the reader works over, as with MixerGain
 */
enum class IFGain : uint8_t {
    dB12,  //!< 12dB
    dB18,  //!< 18dB
    dB21,  //!< 21dB
    dB24,  //!< 24dB
    dB27,  //!< 27dB
    dB30,  //!< 30dB
    dB36,  //!< 36dB, the value the module leaves the factory with
    dB40,  //!< 40dB
};

/*!
  @brief Demodulation threshold the module leaves the factory with
  @details Documented as the lowest value worth using, not as the best one
 */
constexpr uint16_t DEMODULATOR_THRESHOLD_DEFAULT{0x01B0};

/*!
  @struct DemodulatorParameters
  @brief Receiver settings that decide how weak a reply the reader can still make sense of
  @details Where transmit power sets how far the reader reaches, these set how far it listens.
  Both are worth lowering for a tag that sits on the antenna: a reader configured for a metre
  and a half misses most inventory rounds at contact, however strong the reply is
 */
struct DemodulatorParameters {
    MixerGain mixer_gain{MixerGain::dB9};
    IFGain if_gain{IFGain::dB36};
    /*!
      Demodulation threshold. A lower one reaches replies of lower RSSI but is less stable, and
      below some point nothing demodulates at all; a higher one only reaches stronger replies,
      which means shorter range, and is more stable
     */
    uint16_t threshold{DEMODULATOR_THRESHOLD_DEFAULT};
};

//! @brief Mixer gain in dB
inline uint8_t mixerGainDb(const MixerGain gain)
{
    static const uint8_t db[] = {0, 3, 6, 9, 12, 15, 16};
    const uint8_t i           = static_cast<uint8_t>(gain);
    return i < sizeof(db) ? db[i] : 0;
}

//! @brief Intermediate frequency amplifier gain in dB
inline uint8_t ifGainDb(const IFGain gain)
{
    static const uint8_t db[] = {12, 18, 21, 24, 27, 30, 36, 40};
    const uint8_t i           = static_cast<uint8_t>(gain);
    return i < sizeof(db) ? db[i] : 0;
}
//! @brief Highest mixer gain the module accepts
constexpr uint8_t MIXER_GAIN_MAX{0x06};
//! @brief Highest intermediate frequency gain the module accepts
constexpr uint8_t IF_GAIN_MAX{0x07};
//! @brief Length of the demodulator parameter payload
constexpr size_t DEMODULATOR_PARAMETER_LENGTH{4};

/*!
  @brief Split a demodulator parameter payload into its fields
  @param[out] dp Demodulator parameters
  @param param Parameter of the response frame
  @param len Length of param
  @return True if successful
 */
inline bool parse_demodulator_parameters(DemodulatorParameters& dp, const uint8_t* param, const size_t len)
{
    if (param == nullptr || len < DEMODULATOR_PARAMETER_LENGTH) {
        return false;
    }
    if (param[0] > MIXER_GAIN_MAX || param[1] > IF_GAIN_MAX) {
        return false;
    }
    dp.mixer_gain = static_cast<MixerGain>(param[0]);
    dp.if_gain    = static_cast<IFGain>(param[1]);
    dp.threshold  = static_cast<uint16_t>((param[2] << 8) | param[3]);
    return true;
}

/*!
  @brief Build the parameter of Set Demodulator Parameter (0xF0)
  @param[out] out Parameter
  @param dp Demodulator parameters
  @return True if successful
 */
inline bool build_demodulator_parameters(std::vector<uint8_t>& out, const DemodulatorParameters& dp)
{
    const uint8_t mixer = static_cast<uint8_t>(dp.mixer_gain);
    const uint8_t amp   = static_cast<uint8_t>(dp.if_gain);
    if (mixer > MIXER_GAIN_MAX || amp > IF_GAIN_MAX) {
        return false;
    }
    out.clear();
    out.reserve(DEMODULATOR_PARAMETER_LENGTH);
    out.push_back(mixer);
    out.push_back(amp);
    out.push_back(static_cast<uint8_t>(dp.threshold >> 8));
    out.push_back(static_cast<uint8_t>(dp.threshold));
    return true;
}

//! @brief Command code used by every failure notification
constexpr uint8_t COMMAND_ERROR{0xFF};

/*!
  @enum Error
  @brief Error codes carried by a failure notification
 */
enum class Error : uint8_t {
    ReadFail           = 0x09,  //!< Failed to read the tag's data memory area
    WriteFail          = 0x10,  //!< Failed to write the tag's data memory area
    KillFail           = 0x12,  //!< Failed to kill the tag
    LockFail           = 0x13,  //!< Failed to lock the tag's data memory area
    BlockPermalockFail = 0x14,  //!< BlockPermalock execution failed
    InventoryFail      = 0x15,  //!< No tag responded or a data CRC check error occurred
    AccessFail         = 0x16,  //!< Failed to access the tag
    CommandError       = 0x17,  //!< Command error in the command frame
    FHSSFail           = 0x20,  //!< Frequency hopping channel search timed out
};

/*!
  @brief Is the command code of a failure notification?
  @param command Command code of the received frame
  @return True if the frame reports a failure
 */
inline bool is_error_frame(const uint8_t command)
{
    return command == COMMAND_ERROR;
}

/*!
  @brief Does the error code only mean "no tag was found this round"?
  @param error_code Error code carried by the failure notification
  @return True if no tag responded
  @note This answers a polling command and nothing else. Read, Write, Lock and Kill each report
  their own failure, so anywhere but in front of a polling command this is a notification left
  over from polling that has not finished draining
 */
inline bool is_no_tag(const uint8_t error_code)
{
    return error_code == static_cast<uint8_t>(Error::InventoryFail);
}

/*!
  @name Masks the module ORs onto an error the tag itself reported
  @details Only the low four bits of an EPC Gen2 error code carry meaning, so the module fills
  the high nibble with a marker for the operation that provoked it. The marker therefore says
  which command failed, and the low nibble says why
 */
///@{
constexpr uint8_t TAG_ERROR_READ{0xA0};
constexpr uint8_t TAG_ERROR_WRITE{0xB0};
constexpr uint8_t TAG_ERROR_LOCK{0xC0};
constexpr uint8_t TAG_ERROR_KILL{0xD0};
constexpr uint8_t TAG_ERROR_BLOCK_PERMALOCK{0xE0};
///@}

/*!
  @brief Did the tag itself report this error, rather than the module?
  @param error_code Error code carried by the failure notification
  @return True when the code is a Gen2 error the tag returned
 */
inline bool is_tag_error(const uint8_t error_code)
{
    return error_code >= TAG_ERROR_READ && error_code <= (TAG_ERROR_BLOCK_PERMALOCK | 0x0F);
}

/*!
  @brief Describe an error code in one word
  @param error_code Error code carried by the failure notification
  @return Description, never null
  @details A tag error is named by its Gen2 meaning (v1.2.0 Annex I), everything else by what
  the module's own documentation says the code stands for. Read, Write, Kill, Lock,
  BlockPermalock and Inventory all share one wording there, and it names two causes: the tag
  said nothing, or what it said did not pass the CRC check. Which of the two it was is not
  reported, so neither is claimed here
 */
inline const char* error_description(const uint8_t error_code)
{
    if (is_tag_error(error_code)) {
        switch (error_code & 0x0F) {
            case 0x00:
                return "Tag: other error";
            case 0x03:
                return "Tag: memory overrun";
            case 0x04:
                return "Tag: memory locked";
            case 0x0B:
                return "Tag: insufficient power";
            case 0x0F:
                return "Tag: non-specific error";
            default:
                return "Tag: unlisted error";
        }
    }
    switch (static_cast<Error>(error_code)) {
        case Error::ReadFail:
            return "Read failed: no answer, or a CRC error";
        case Error::WriteFail:
            return "Write failed: no answer, or a CRC error";
        case Error::KillFail:
            return "Kill failed: no answer, or a CRC error";
        case Error::LockFail:
            return "Lock failed: no answer, or a CRC error";
        case Error::BlockPermalockFail:
            return "BlockPermalock failed: no answer, or a CRC error";
        case Error::InventoryFail:
            return "No tag answered, or a CRC error";
        case Error::AccessFail:
            return "Access failed; the password may be wrong";
        case Error::CommandError:
            return "Command error in the command frame";
        case Error::FHSSFail:
            return "Frequency hopping channel search timed out";
        default:
            return "Unlisted error";
    }
}

/*!
  @brief Could this error code be the answer to this command?
  @param error_code Error code carried by the failure notification
  @param command Command code the module was asked to carry out
  @return True when the code can have come from that command
  @details An error the tag reported carries the operation in its high nibble, and the module's
  own failures are named after the command they belong to, so most codes say what they answer.
  A code that any command can provoke, and any code not listed here, is accepted for all of
  them: turning away an error that did belong to the command would leave the caller waiting out
  its timeout for a reply that has already arrived
 */
inline bool error_answers_command(const uint8_t error_code, const uint8_t command)
{
    if (is_tag_error(error_code)) {
        switch (error_code & 0xF0) {
            case TAG_ERROR_READ:
                return command == COMMAND_READ_TAG_MEMORY;
            case TAG_ERROR_WRITE:
                return command == COMMAND_WRITE_TAG_MEMORY;
            case TAG_ERROR_LOCK:
                return command == COMMAND_LOCK_TAG_MEMORY;
            case TAG_ERROR_KILL:
                return command == COMMAND_KILL_TAG;
            case TAG_ERROR_BLOCK_PERMALOCK:
                return command == COMMAND_BLOCK_PERMALOCK;
            default:
                return true;
        }
    }
    switch (static_cast<Error>(error_code)) {
        case Error::ReadFail:
            return command == COMMAND_READ_TAG_MEMORY;
        case Error::WriteFail:
            return command == COMMAND_WRITE_TAG_MEMORY;
        case Error::KillFail:
            return command == COMMAND_KILL_TAG;
        case Error::LockFail:
            return command == COMMAND_LOCK_TAG_MEMORY;
        case Error::BlockPermalockFail:
            return command == COMMAND_BLOCK_PERMALOCK;
        case Error::InventoryFail:
            return command == COMMAND_SINGLE_POLLING || command == COMMAND_MULTIPLE_POLLING;
        case Error::AccessFail:
            // Every tag operation is preceded by an access, so any of them can end here
            return command == COMMAND_READ_TAG_MEMORY || command == COMMAND_WRITE_TAG_MEMORY ||
                   command == COMMAND_LOCK_TAG_MEMORY || command == COMMAND_KILL_TAG ||
                   command == COMMAND_BLOCK_PERMALOCK;
        default:
            // A command error or a failed hop answers whatever was sent, and so does a code
            // this table does not name
            return true;
    }
}

/*!
  @brief What is to be done with a frame that has just been read
 */
enum class FrameRoute : uint8_t {
    TagNotification,  //!< A tag the module found. Parse it and queue it
    Response,         //!< The answer to the command being waited on
    Drop,             //!< Left over from an exchange that has already given up
    Unexpected,       //!< Nobody asked for this one
};

/*!
  @brief Decide what a received frame answers
  @param f Frame that has been read
  @param response_pending Is a command waiting for its answer?
  @param awaiting_command Command code being waited on, where there is one
  @return What is to be done with the frame
  @details A frame carries nothing to say which exchange it belongs to, so the command code and
  the error code are all there is to go on. One that cannot have come from the command being
  waited on is left over from an exchange that already gave up, and answering with it would
  shift every later exchange by one
 */
inline FrameRoute route_for(const Frame& f, const bool response_pending, const uint8_t awaiting_command)
{
    // The protocol document is inconsistent about the Type byte of a tag notification, so the
    // command code is what routes it
    if (f.command == COMMAND_SINGLE_POLLING || f.command == COMMAND_MULTIPLE_POLLING) {
        return FrameRoute::TagNotification;
    }
    if (is_error_frame(f.command)) {
        const uint8_t code = f.parameter.empty() ? 0x00 : f.parameter[0];
        // Inventory Fail answers a polling command and nothing else: every tag operation has a
        // failure code of its own. It arrives once per empty round, and the rounds already under
        // way keep arriving after a stop, so anywhere but in front of a polling command it is a
        // leftover rather than the answer to whatever was sent last
        const bool awaiting_inventory = response_pending && (awaiting_command == COMMAND_SINGLE_POLLING ||
                                                             awaiting_command == COMMAND_MULTIPLE_POLLING);
        if (is_no_tag(code) && !awaiting_inventory) {
            return FrameRoute::Drop;
        }
        if (!response_pending) {
            return FrameRoute::Drop;
        }
        return error_answers_command(code, awaiting_command) ? FrameRoute::Response : FrameRoute::Drop;
    }
    if (response_pending && f.command == awaiting_command) {
        return FrameRoute::Response;
    }
    // A module answering under a command code nobody is waiting for is exactly how an exchange
    // slips by one frame, so this is worth saying out loud rather than dropping quietly
    return FrameRoute::Unexpected;
}

/*!
  @brief Is this failure worth sending the same command again for?
  @param error_code Error code carried by the failure notification
  @return True when a repeat has a chance of succeeding
  @details A failure that means the tag said nothing, or said something that did not survive
  the air, says nothing about the tag being unwilling: this one exchange did not complete and
  the next one may. A tag that answered with a reason of its own will answer the same way
  however often it is asked, so those are excluded.
  A failed access is excluded for a different reason. Repeating one straight away is what
  starts a security timeout on the tag (EPC Gen2 v2.1 6.3.2.5), and the timeout outlasts the
  gap between two attempts here, so a repeat would be worth less than the harm it does
 */
inline bool is_worth_retrying(const uint8_t error_code)
{
    if (is_tag_error(error_code)) {
        return false;
    }
    switch (static_cast<Error>(error_code)) {
        case Error::ReadFail:
        case Error::WriteFail:
        case Error::KillFail:
        case Error::LockFail:
        case Error::BlockPermalockFail:
            return true;
        default:
            return false;
    }
}

}  // namespace m100
}  // namespace unit
}  // namespace m5
#endif
