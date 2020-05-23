#!/usr/bin/env bash

set -e

CACHING="$1"

ls
ls hou

# move hou tarball into top-level
cp hou/hou.tar.gz .
tar -xzf hou.tar.gz
