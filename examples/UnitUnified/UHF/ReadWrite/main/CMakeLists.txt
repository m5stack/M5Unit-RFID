# Dependencies (M5Unit-RFID, M5UnitUnified, M5Unified, ...) are pulled in via
# main/idf_component.yml and auto-added as requirements by the component manager.
idf_component_register(
    SRCS "ReadWrite.cpp"
    INCLUDE_DIRS ".")

# Unit UHF-RFID (U107) is the only UHF unit, so no Kconfig variant selection is needed.
