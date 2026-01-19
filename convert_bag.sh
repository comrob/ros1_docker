#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: convert_bag.sh <path_to_bagfile>"
    exit 1
fi

INPUT_PATH=$(realpath "$1")
INPUT_DIR=$(dirname "$INPUT_PATH")
INPUT_FILE=$(basename "$INPUT_PATH")

echo "[HOST] Mounting: $INPUT_DIR"
echo "[HOST] Target:   $INPUT_FILE"
echo "----------------------------------------"

docker run --rm -it \
  -v "$INPUT_DIR":/data \
  --user 1000:1000 \
  ros_bag_converter:latest \
  python3 /home/dev/ros1_to_mcap.py /data/"$INPUT_FILE"