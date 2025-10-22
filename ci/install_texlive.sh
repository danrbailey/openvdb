#!/usr/bin/env bash
# Copyright Contributors to the OpenVDB Project
# SPDX-License-Identifier: Apache-2.0

set -ex

# 1. Download the TeX Live installer
wget https://mirror.ctan.org/systems/texlive/tlnet/install-tl-unx.tar.gz

# 2. Extract
tar xzf install-tl-unx.tar.gz
rm install-tl-unx.tar.gz
cd install-tl-*

# 3. Run the installer (interactive)
./install-tl --no-interaction --scheme=small
