# Map the Kconfig "Target unit / board" choice (see common/Kconfig.variant) to the source-level
# USING_* macro that both the Arduino and ESP-IDF builds use. Include this from an example's
# main/CMakeLists.txt *after* idf_component_register() (it needs ${COMPONENT_LIB}).
# Default (nothing selected): UnitNFC (I2C).
if(CONFIG_EXAMPLE_USING_CAP_CC1101)
    set(M5UNIT_VARIANT USING_CAP_CC1101)
elseif(CONFIG_EXAMPLE_USING_UNIT_RFID2)
    set(M5UNIT_VARIANT USING_UNIT_RFID2)
elseif(CONFIG_EXAMPLE_USING_M5DIAL_BUILTIN_WS1850S)
    set(M5UNIT_VARIANT USING_M5DIAL_BUILTIN_WS1850S)
else()
    set(M5UNIT_VARIANT USING_UNIT_NFC)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${M5UNIT_VARIANT})
