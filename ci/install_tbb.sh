#!/usr/bin/env bash

set -ex

TBB_VERSION="$1"

git clone https://github.com/01org/tbb.git
cd tbb

if [ "$TBB_VERSION" != "latest" ]; then
    git checkout tags/${TBB_VERSION} -b ${TBB_VERSION}
fi

make -j4
sudo cp -r include/serial /usr/local/include/.
sudo cp -r include/tbb /usr/local/include/.
sudo cp -r build/*/*.so* /usr/local/lib/.
