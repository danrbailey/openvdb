#!/usr/bin/env bash

set -ex

VERSION="$1"
COMPILER="$2"
RELEASE="$3"
EXTRAS="$4"

cd /opt/hou${VERSION}
source houdini_setup_bash
cd -

mkdir build
cd build
cmake \
    -DCMAKE_CXX_COMPILER=${COMPILER} \
    -DCMAKE_BUILD_TYPE=${RELEASE} \
    -DOPENVDB_CXX_STRICT=ON \
    -DOPENVDB_BUILD_HOUDINI_PLUGIN=ON \
    -DOPENVDB_BUILD_BINARIES=${EXTRAS} \
    -DOPENVDB_BUILD_PYTHON_MODULE=${EXTRAS} \
    -DOPENVDB_BUILD_UNITTESTS=${EXTRAS} \
     ..

# Can only build using one thread with GCC due to memory constraints
if [ "$COMPILER" = "clang++" ]; then
    make -j2
else
    make
fi
