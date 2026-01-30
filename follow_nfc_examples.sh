#!/bin/bash

REMOTE_NAME="nfc_examples"
REMOTE_URL="git@github.com:m5stack/M5Unit-NFC.git"
#REMOTE_BRANCH="main"
REMOTE_BRANCH="develop"
SRC_DIR="examples/UnitUnified/NFCA"  # Path(M5Unit-NFC)
DEST_DIR="examples/UnitUnified/NFCA" # Path(M5Unit-RFID)

echo "--- 1. Fetch nfc_examples ---"
git fetch $REMOTE_NAME

echo "--- 2. Extracting ---"
SHA=$(git subtree split --prefix=$SRC_DIR $REMOTE_NAME/$REMOTE_BRANCH)
if [ -z "$SHA" ]; then
    echo "Error: Failed to extract"
    exit 1
fi

echo "Extracted SHA: $SHA"

echo "--- 3. Merge ---"
git subtree pull --prefix=$DEST_DIR $REMOTE_NAME $SHA --squash

echo "--- Done ---"

