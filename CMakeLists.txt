# ESP-IDF component manifest for M5Unit-RFID.
# MFRC522 / WS1850S are I2C polling devices (no GPIO ISR / SPI / crypto on the
# host side), so unlike M5Unit-NFC this component needs no driver / mbedtls
# dependency. MIFARE authentication runs on-chip via the MFAuthent command.
file(GLOB_RECURSE SRCS "src/*.cpp")
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES M5UnitUnified M5Unit-NFC
    PRIV_REQUIRES M5Utility
)
