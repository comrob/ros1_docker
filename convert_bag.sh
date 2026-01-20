#!/bin/bash

# Usage Check
if [ -z "$1" ]; then
    echo "Usage: ./convert_bag.sh <input_path> [options]"
    echo "Examples:"
    echo "  Folder (Series): ./convert_bag.sh /path/to/bags --series"
    echo "  Single File:     ./convert_bag.sh /path/to/my.bag"
    exit 1
fi

# 1. Analyze Input Path
INPUT_PATH="$1"
ABS_PATH=$(realpath "$INPUT_PATH")

if [ -d "$ABS_PATH" ]; then
    # CASE A: Input is a Directory
    MOUNT_HOST="$ABS_PATH"
    # Inside container, we point to the mount root
    CONTAINER_ARG="/data"
    echo "[HOST] Detected Folder Input"
else
    # CASE B: Input is a File
    MOUNT_HOST=$(dirname "$ABS_PATH")
    FILENAME=$(basename "$ABS_PATH")
    # Inside container, we point to the specific file
    CONTAINER_ARG="/data/$FILENAME"
    echo "[HOST] Detected File Input"
fi

echo "[HOST] Mounting: $MOUNT_HOST -> /data"

# 2. Build Python Arguments
# We reconstruct the argument list to pass to the Python script.
# - The first arg ($1) is replaced by our calculated CONTAINER_ARG.
# - All subsequent args are checked:
#   - If they look like flags (--series), keep them as-is.
#   - If they look like extra files, assume they are in the same folder and rewrite path.

PY_ARGS="$CONTAINER_ARG"
shift # Skip the first argument since we handled it

for arg in "$@"
do
    if [[ "$arg" == -* ]]; then
        # It's a flag (e.g. --series, --distro)
        PY_ARGS="$PY_ARGS $arg"
    else
        # It's another file. We assume it lives in the same mounted directory.
        FNAME=$(basename "$arg")
        PY_ARGS="$PY_ARGS /data/$FNAME"
    fi
done

echo "[CMD] Running: python3 convert.py $PY_ARGS"
echo "----------------------------------------"

# 3. Run Container
docker run --rm -it \
  -v "$MOUNT_HOST":/data \
  --user $(id -u):$(id -g) \
  ros_bag_converter:latest \
  python3 /home/dev/convert.py $PY_ARGS