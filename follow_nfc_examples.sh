#!/bin/bash

REMOTE_NAME="nfc_examples"
REMOTE_URL="git@github.com:m5stack/M5Unit-NFC.git"
#REMOTE_BRANCH="main"
REMOTE_BRANCH="develop"

SUBDIRS=("NFCA" "NFCB")

echo "--- 1. Fetch nfc_examples ---"
git fetch $REMOTE_NAME

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

    echo "--- 3. Merge into $DEST_DIR ---"
    git subtree pull --prefix=$DEST_DIR $REMOTE_NAME $SHA --squash
done

echo "--- Done ---"
