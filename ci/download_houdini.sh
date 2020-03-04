#!/usr/bin/env bash

set -e

HOUDINI_MAJOR="$1"
GOLD="$2"
HOUDINI_CLIENT_ID="$4"
HOUDINI_SECRET_KEY="$5"

if [ "$HOUDINI_CLIENT_ID" == "" ]; then
    echo "HOUDINI_CLIENT_ID GitHub Action Secret needs to be set to install Houdini builds"
    exit 1
fi
if [ "$HOUDINI_SECRET_KEY" == "" ]; then
    echo "HOUDINI_SECRET_KEY GitHub Action Secret needs to be set to install Houdini builds"
    exit 1
fi

pip install --user requests

python ci/download_houdini.py $HOUDINI_MAJOR $GOLD $HOUDINI_CLIENT_ID $HOUDINI_SECRET_KEY


mkdir stage
mv hou.tar.gz stage/.
cd stage

mkdir -p hou/bin
mkdir -p hou/houdini
mkdir -p hou/toolkit
mkdir -p hou/dsolib

tar -xzf hou.tar.gz
cd houdini*
tar -xzf houdini.tar.gz

cp houdini_setup* ../hou/.
cp -r toolkit/cmake ../hou/toolkit/.
cp -r toolkit/include ../hou/toolkit/.
cp -r dsolib/libHoudini* ../hou/dsolib/.
cp -r dsolib/libblosc* ../hou/dsolib/.
cp -r dsolib/libhboost* ../hou/dsolib/.
cp -r dsolib/libz* ../hou/dsolib/.
cp -r dsolib/libtbb* ../hou/dsolib/.
cp -r dsolib/libHalf* ../hou/dsolib/.
cp -r dsolib/libjemalloc* ../hou/dsolib/.

cd ../..

mkdir houdini18_0

mv stage/hou houdini18_0/.

cd houdini18_0

tar -czvf hou.tar.gz hou
rm -rf hou
