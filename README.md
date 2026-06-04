# M5Unit - RFID

## Overview

Library for Unit-RFID using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnified has a unified API and can control multiple units via PaHub, etc.

### SKU:U031-B

Unit RFID2 is a radio frequency identification (RFID) read/write unit based on the 13.56MHz frequency band. It integrates the WS1850S chip and complies with the ISO/IEC 14443 Type A/B standard, supporting data read/write operations for RFID cards such as MIFARE and NTAG series. The unit communicates via the I2C interface and has a read/write distance of less than 20mm.


## PICC Support

Raw R/W includes FileSystem via ISO-DEP when applicable.  
Support may be expanded in future updates to cover PICCs not listed here.

### NFC-A

| PICC Type | NFC Forum Tag (NDEF) | Detect | Identify | Raw R/W | Notes |
|---|---|---|---|---|---|
| MIFARE Classic Mini | None | Yes | Yes | Yes | Auth required |
| MIFARE Classic 1K | None | Yes | Yes | Yes | Auth required |
| MIFARE Classic 2K | None | Yes | Yes | Yes | Auth required |
| MIFARE Classic 4K | None | Yes | Yes | Yes | Auth required |
| MIFARE Ultralight | Type2 | Yes | Yes | Yes |  |
| MIFARE Ultralight EV1 MF0UL11 | Type2 | Yes | Yes | Yes |  |
| MIFARE Ultralight EV1 MF0UL21 | Type2 | Yes | Yes | Yes |  |
| MIFARE Ultralight Nano | Type2 | Yes | Yes | Yes |  |
| MIFARE Ultralight C | Type2 | Yes | Yes | Yes |  |
| NTAG 203 | Type2 | Yes | Yes | Yes |  |
| NTAG 210u | Type2 | Yes | Yes | Yes |  |
| NTAG 210 | Type2 | Yes | Yes | Yes |  |
| NTAG 212 | Type2 | Yes | Yes | Yes |  |
| NTAG 213 | Type2 | Yes | Yes | Yes |  |
| NTAG 215 | Type2 | Yes | Yes | Yes |  |
| NTAG 216 | Type2 | Yes | Yes | Yes |  |
| ST25TA512B | Type4 | Yes | Yes | Yes | ISO-DEP |
| ST25TA02K | Type4 | Yes | Yes | Yes | ISO-DEP |
| ST25TA16K | Type4 | Yes | Yes | Yes | ISO-DEP |
| ST25TA64K | Type4 | Yes | Yes | Yes | ISO-DEP |
| MIFARE Plus 2K (S/X/EV1/EV2) | None | Yes | Yes | Yes | SL0/SL1(\*1)|
| MIFARE Plus 4K (S/X/EV1/EV2) | None | Yes | Yes | Yes | SL0/SL1(\*1)|
| MIFARE Plus SE 1K | None | Yes | Yes | Yes | SL0/SL1/SL3|
| MIFARE DESFire 2K (EV1/EV2/EV3) | Type4 | Yes | Yes | Yes | ISO-DEP |
| MIFARE DESFire 4K (EV1/EV2/EV3) | Type4 | Yes | Yes | Yes | ISO-DEP |
| MIFARE DESFire 8K (EV1/EV2/EV3) | Type4 | Yes | Yes | Yes | ISO-DEP |
| MIFARE DESFire Light | Type4 | Yes | Yes | Yes | NDEF is not supported yet |

- \*1 MIFARE Plus SL3 operation has issues.

### NFC-B

**Available on UnitRFID2 (WS1850S) only.** UnitRFID (MFRC522) does NOT support NFC-B.

| PICC Type | NFC Forum Tag (NDEF) | Detect | Identify | Raw R/W | Notes |
|---|---|---|---|---|---|
| Unclassified | None | Yes | Partial | Yes | ISO-DEP transport only |

> **Note:** NFC-B is **not supported on the M5Dial builtin WS1850S**. The builtin small loop antenna cannot generate sufficient RF field for Type B PICC activation (Type B uses 10% ASK + BPSK subcarrier which is more sensitive to field strength and SNR than Type A). NFC-B requires the external **UnitRFID2** (with larger antenna).


## Emulation

Emulation is **NOT** supported on UnitRFID and UnitRFID2.

## Known Issues

- MIFARE Plus SL3 operation has issues.

### NessoN1 Connection
GROVE port (port_b) on NessoN1 uses SoftwareI2C (M5HAL Bus), which causes I2C register polling latency too high for MFRC522/WS1850S RF timing requirements.  
Use **QWIIC port (port_a)** with a QWIIC-GROVE conversion cable instead.

> **Note:** GROVE port support may be added in a future update if SoftwareI2C performance improves.


## Related Link

- [Unit RFID2 & Datasheet](https://docs.m5stack.com/en/unit/rfid2)

## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)
- [M5Unit-NFC](https://github.com/m5stack/M5Unit-NFC)

## License

- [M5Unit-RFID - MIT](LICENSE)


## Examples
See also [examples/UnitUnified](examples/UnitUnified)

> **Note:** The examples in this library are imported from [M5Unit-NFC](https://github.com/m5stack/M5Unit-NFC) and shared between both libraries.
> The same source file supports multiple units via `#define` switches.

### For ArduinoIDE
Each example contains the following block to select the unit:

```cpp
// For UnitNFC
// #define USING_UNIT_NFC
// For CapNFC
// #define USING_CAP_CC1101
// For UnitRFID2
// #define USING_UNIT_RFID2
```

The examples are shared with [M5Unit-NFC](https://github.com/m5stack/M5Unit-NFC), which is why other unit definitions exist.  
For this library, uncomment `USING_UNIT_RFID2`:

```cpp
// #define USING_UNIT_NFC
// #define USING_CAP_CC1101
#define USING_UNIT_RFID2
```

## Doxygen document
[GitHub Pages](https://m5stack.github.io/M5Unit-RFID/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required
- [Doxygen](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)
