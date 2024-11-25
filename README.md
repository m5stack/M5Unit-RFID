# M5Unit - RFID

## Overview

Library for Unit-RFID using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnfied has a unified API and can control multiple units via PaHub, etc.

### SKU:U031-B

RFID2 is a radio frequency identification unit. Built-in WS1850S chip, working frequency is 13.56MHz. Supports reading card, writing card, recognition, recording, and encoding RF card Multiple functions such as authorization and authorization. Use magnetic field induction technology to realize non-contact two-way information interaction, read and verify the information of proximity cards. It can be used in access control systems, check-in systems, warehouse goods entry and storage, and community vehicle entry and exit registration needs Application scenarios for information verification.

## Related Link

- [Unit RFID2 & Datasheet](https://docs.m5stack.com/en/unit/rfid2)

## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)

## License

- [M5Unit-RFID -MIT](LICENSE)


## Feature
- Detection with collision resolution when multiple devices are in the detection range
- Identify MIFARE and NTAG devices in detail
- Support NFC-A Type-2 NDEF Read/Write


## Usage

```cpp
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedRFID.h>
namespace {
m5::unit::UnitUnified Units;
m5::unit::UnitRFID2 unit;
}  // namespace

using namespace m5::rfid;

void setup()
{
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 400 * 1000U);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        while (true) {
            m5::utility::delay(10000);
        }
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());
    M5_LOGI("Please put the devices...");
}

void loop()
{
    M5.update();
    Units.update();
    auto touch = M5.Touch.getDetail();
    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        if (unit.detectDevice()) {
            UID uid{};
            if (unit.activateDevice(uid)) {
                M5_LOGI("UID:%s %s", uid.uidAsString().c_str(), uid.typeAsString().c_str());

                //
                // Use any API...
                //
                
                unit.deactivateDevice(); // Once activated, clean up with deactivate.
            }
        }
    }
}
```

## Examples
See also [examples/UnitUnified](examples/UnitUnified)

## Doxygen document
If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required
- [Doxyegn](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)
