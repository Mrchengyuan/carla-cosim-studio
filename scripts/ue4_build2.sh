#!/bin/bash
cd ~/UnrealEngine_4.26
echo "=== GenerateProjectFiles $(date)"
./GenerateProjectFiles.sh < /dev/null || { echo "GPF_FAILED"; exit 1; }
echo "=== make $(date)"
make < /dev/null || { echo "MAKE_FAILED"; exit 1; }
echo "=== UE4_BUILD_DONE $(date)"
