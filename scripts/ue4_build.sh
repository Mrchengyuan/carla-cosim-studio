#!/bin/bash
set -o pipefail
cd ~/UnrealEngine_4.26
export http_proxy=http://127.0.0.1:7897 https_proxy=http://127.0.0.1:7897 HTTP_PROXY=http://127.0.0.1:7897 HTTPS_PROXY=http://127.0.0.1:7897
export no_proxy=localhost,127.0.0.1
echo "=== Setup $(date)"
yes n | ./Setup.sh --proxy=http://127.0.0.1:7897 --threads=16 || { echo "SETUP_FAILED"; exit 1; }
echo "=== GenerateProjectFiles $(date)"
./GenerateProjectFiles.sh < /dev/null || { echo "GPF_FAILED"; exit 1; }
echo "=== make $(date)"
make < /dev/null || { echo "MAKE_FAILED"; exit 1; }
echo "=== UE4_BUILD_DONE $(date)"
