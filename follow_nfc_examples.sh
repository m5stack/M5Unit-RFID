#!/bin/bash

REMOTE_NAME="nfc_examples"
REMOTE_URL="git@github.com:m5stack/M5Unit-NFC.git"
#REMOTE_BRANCH="main"
REMOTE_BRANCH="develop"

SUBDIRS=("NFCA" "NFCB" "common")

# Prevent M5Unit-NFC's tags from polluting this repo's tag namespace.
# Fetch auto-follows tags pointing to downloaded objects (git subtree's internal
# fetch too), which imported NFC's 0.0.3 / 0.1.0 etc. into refs/tags here.
# --no-tags on the remote disables tag-following for every fetch to it.
git config remote.$REMOTE_NAME.tagOpt --no-tags

echo "--- 1. Fetch nfc_examples ---"
git fetch --no-tags $REMOTE_NAME

for SUBDIR in "${SUBDIRS[@]}"; do
    SRC_DIR="examples/UnitUnified/$SUBDIR"   # Path(M5Unit-NFC)
    DEST_DIR="examples/UnitUnified/$SUBDIR"  # Path(M5Unit-RFID)

    echo "--- 2. Extracting $SRC_DIR ---"
    SHA=$(git subtree split --prefix=$SRC_DIR $REMOTE_NAME/$REMOTE_BRANCH)
    if [ -z "$SHA" ]; then
        echo "Error: Failed to extract $SUBDIR"
        exit 1
    fi
    echo "Extracted SHA: $SHA"

    if [ -d "$DEST_DIR" ]; then
        echo "--- 3. Merge (pull) into $DEST_DIR ---"
        git subtree pull --prefix=$DEST_DIR $REMOTE_NAME $SHA --squash
    else
        echo "--- 3. Add into $DEST_DIR ---"
        git subtree add --prefix=$DEST_DIR $REMOTE_NAME $SHA --squash
    fi
done

# --- 4. Rewrite example idf_component.yml self-reference for M5Unit-RFID ---
# NFC follow leaves a self-ref override_path to M5Unit-NFC. In this repo the
# self-component is M5Unit-RFID; M5Unit-NFC (and M5UnitUnified) are pulled in
# transitively via M5Unit-RFID's own root idf_component.yml, so we only need to
# rename the self-reference (no explicit M5Unit-NFC entry here).
# NOTE: this leaves the manifests modified in the working tree; commit them
# yourself before the next follow (subtree pull requires a clean tree).
echo "--- 4. Rewrite example idf_component.yml self-reference for M5Unit-RFID ---"
for YML in $(find examples/UnitUnified/NFCA examples/UnitUnified/NFCB -path '*/main/idf_component.yml'); do
    perl -0pi -e 's{  m5stack/M5Unit-NFC:\n    (override_path|path): "\.\./\.\./\.\./\.\./\.\."}{  m5stack/M5Unit-RFID:\n    $1: "../../../../.."}g' "$YML"
    echo "  rewrote $YML"
done

# --- 5. Rewrite Kconfig for M5Unit-RFID (menu label + default variant) ---
# The shared common/ Kconfig files label the menuconfig menu "M5Unit-NFC example"
# and default to UnitNFC. In this repo we surface "M5Unit-RFID example" and default
# to UnitRFID2 where offered. Like §4 this leaves the common/ files modified in the
# working tree; commit before the next follow.
echo "--- 5. Rewrite Kconfig for M5Unit-RFID ---"
for KC in examples/UnitUnified/common/Kconfig.variant.*; do
    perl -0pi -e 's{menu "M5Unit-NFC example"}{menu "M5Unit-RFID example"}g' "$KC"
    echo "  rewrote menu label: $KC"
done
# Default to UnitRFID2 where it is offered (full / no_dial; basic has no RFID2 option).
for KC in examples/UnitUnified/common/Kconfig.variant.full examples/UnitUnified/common/Kconfig.variant.no_dial; do
    perl -0pi -e 's{default EXAMPLE_USING_UNIT_NFC}{default EXAMPLE_USING_UNIT_RFID2}g' "$KC"
    echo "  rewrote default: $KC"
done

echo "--- Done ---"
