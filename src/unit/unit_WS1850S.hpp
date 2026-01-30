/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_WS1850S.hpp
  @brief WS1850S Unit for M5UnitUnified
*/
#ifndef M5_UNIT_RFID2_UNIT_WS1850S_HPP
#define M5_UNIT_RFID2_UNIT_WS1850S_HPP

#include "unit_MFRC522.hpp"

namespace m5 {
namespace unit {

/*!
  @class UnitWS1850S
  @brief Radio frequency identification unit
  @details Functionally compatible with MFRC522
 */
class UnitWS1850S : public UnitMFRC522 {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitWS1850S, 0x28);

public:
    /*!
      @brief Constructor
      @param addr I2C address
     */
    explicit UnitWS1850S(const uint8_t addr = DEFAULT_ADDRESS) : UnitMFRC522(addr)
    {
    }
    /*!
      @brief Destructor
     */
    virtual ~UnitWS1850S()
    {
    }

    /*!
      @brief Begin the unit
      @return True if successful
     */
    virtual bool begin() override;

protected:
    virtual bool self_test() override;
};

}  // namespace unit
}  // namespace m5
#endif
