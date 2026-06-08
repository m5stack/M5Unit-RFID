# NFCB Examples shared between M5Unit-NFC and M5Unit-RFID

These examples are shared between [M5Unit-NFC](https://github.com/m5stack/M5Unit-NFC) and [M5Unit-RFID](https://github.com/m5stack/M5Unit-RFID).

**M5Unit-NFC is the primary repository for these examples.**
Modifications should be made in M5Unit-NFC first, then propagated to M5Unit-RFID via [follow_nfc_examples.sh](https://github.com/m5stack/M5Unit-RFID/blob/develop/follow_nfc_examples.sh).

**Do not edit these files directly in the M5Unit-RFID repository.**

> Note: The M5Dial built-in WS1850S is **not supported for NFC-B** (vendor limitation) and is therefore excluded from the unit selection guard in these examples. UnitNFC / CapCC1101NFC (ST25R3916, both I2C and SPI) and the external UnitRFID2 (WS1850S over I2C) are supported.
