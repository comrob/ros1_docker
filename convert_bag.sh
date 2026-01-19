#!/bin/bash

# Check args
if [ -z "$1" ]; then
    echo "Usage: convert_bag.sh <folder_or_file> [more_files...] [--batch]"
    exit 1
fi

# Get absolute path of the FIRST argument
FIRST_ARG=$(realpath "$1")

# Determine mount point
if [ -d "$FIRST_ARG" ]; then
    # If first arg is a directory, mount it directly
    MOUNT_DIR="$FIRST_ARG"
    TARGET="/data"
else
    # If first arg is a file, mount its parent directory
    MOUNT_DIR=$(dirname "$FIRST_ARG")
    TARGET="/data"
fi

echo "[HOST] Mounting: $MOUNT_DIR"
echo "[HOST] Target:   /data"
echo "----------------------------------------"

# We construct the command arguments for Python
# We replace the host path with /data/ for all file arguments
PY_ARGS=""
for var in "$@"
do
    # Check if the argument is a file/path that exists inside our mount dir
    # If yes, rewrite path to /data/...
    # If no (like --batch), keep as is.
    if [[ "$var" == -* ]]; then
        PY_ARGS="$PY_ARGS $var"
    else
        # Basename only works if all files are in that one folder
        FNAME=$(basename "$var")
        PY_ARGS="$PY_ARGS /data/$FNAME"
    fi
done

docker run --rm -it \
  -v "$MOUNT_DIR":/data \
  --user 1000:1000 \
  ros_bag_converter:latest \
  python3 /home/dev/convert.py $PY_ARGS