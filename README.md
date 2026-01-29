# M5Unit - RFID

## Overview

Library for Unit-RFID using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnfied has a unified API and can control multiple units via PaHub, etc.

### SKU:U031-B

RFID2 is a radio frequency identification unit. Built-in WS1850S chip, working frequency is 13.56MHz. Supports reading card, writing card, recognition, recording, and encoding RF card Multiple functions such as authorization and authorization. Use magnetic field induction technology to realize non-contact two-way information interaction, read and verify the information of proximity cards. It can be used in access control systems, check-in systems, warehouse goods entry and storage, and community vehicle entry and exit registration needs Application scenarios for information verification.


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


## Emulation

Emulation is not supported on Unit-RFID.

## Known Issues

- MIFARE Plus SL3 operation has issues.


## Related Link

- [Unit RFID2 & Datasheet](https://docs.m5stack.com/en/unit/rfid2)

## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)
- [M5Unit-NFC](https://github.com/m5stack/M5Unit-NFC)

## License

- [M5Unit-RFID -MIT](LICENSE)


## Examples
See also [examples/UnitUnified](examples/UnitUnified)

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
