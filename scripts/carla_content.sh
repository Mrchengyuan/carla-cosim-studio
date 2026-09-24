#!/bin/bash
set -e
D=~/carla_carsim/carla_src/Unreal/CarlaUE4/Content/Carla
mkdir -p "$D" ~/carla_carsim/downloads && cd ~/carla_carsim/downloads
U=https://carla-assets.s3.us-east-005.backblazeb2.com/20250912_2171890.tar.gz
aria2c --all-proxy=http://127.0.0.1:7897 -x16 -s16 -k16M --file-allocation=none --summary-interval=30 -c -o Content_0916.tar.gz "$U"
echo "extracting..."; tar -xzf Content_0916.tar.gz -C "$D" && rm Content_0916.tar.gz
echo 20250912_2171890 > "$D/.version"; echo CONTENT_DONE
