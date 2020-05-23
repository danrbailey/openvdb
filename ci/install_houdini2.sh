#!/usr/bin/env bash

set -e

CACHING="$1"

ls
ls hou

# move hou tarball into top-level
cp hou/hou.tar.gz .
tar -xzf hou.tar.gz

# remove the hou directory, this disables the ability to cache
if [ "$CACHING" == "OFF" ]; then
    rm -rf hou
fi
