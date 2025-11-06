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
- [Doxyegn](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)
