#!/bin/bash

# Please execute on repository root

## Get version from library.properties (POSIX-only: grep + cut + tr)
LIB_VERSION="$(grep '^version' library.properties | cut -d= -f2 | tr -d ' ')"

## Get git rev of HEAD (suppress suffix if not in a git repo)
GIT_REV="$(git rev-parse --short HEAD 2>/dev/null)"

DOXYGEN_PROJECT_NUMBER="${LIB_VERSION}${GIT_REV:+ git rev:$GIT_REV}" doxygen docs/Doxyfile
