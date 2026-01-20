#!/bin/bash


SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# <--- INSERT START --->
# --- 0. Help Flag Handler ---
# If help is requested, bypass path checks/mounting and let Python print usage.
for arg in "$@"; do
    if [[ "$arg" == "--help" ]] || [[ "$arg" == "-h" ]]; then
        echo "==============================================================="
        echo " ROS1 -> MCAP Converter (Dockerized Wrapper)"
        echo "==============================================================="
        echo " This script wraps the conversion logic in a Docker container."
        echo " It automatically mounts the input directory to /data inside"
        echo " the container and forwards arguments to the internal script."
        echo ""
        echo " [WRAPPER CONSTRAINTS]"
        echo "   1. All input files must reside in the same host directory."
        echo "   2. Output is saved to the source directory (or defined subfolder)."
        echo ""
        echo " [INTERNAL PYTHON HELP]"
        
        # Run the internal help
        HOST_DATA_DIR="." \
        CURRENT_UID=$(id -u) \
        CURRENT_GID=$(id -g) \
        docker compose -f "$SCRIPT_DIR/docker-compose.yml" run --rm converter \
        python3 /home/dev/convert.py --help
        
        exit 0
    fi
done

# Usage Check
if [ -z "$1" ]; then
    echo "Usage: convert_bag <input_path> [options]"
    exit 1
fi

SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# --- 1. Safety Check: Verify all files are in the same folder ---
# We take the directory of the first argument as the reference.
REF_DIR=$(dirname "$(realpath "$1")")

for arg in "$@"; do
    # Skip flags
    if [[ "$arg" == -* ]]; then continue; fi
    
    # Get absolute path of current arg
    CUR_PATH=$(realpath "$arg")
    
    # If it's a directory, we check its parent (because the dir itself is the input)
    if [ -d "$CUR_PATH" ]; then
        CUR_DIR=$(dirname "$CUR_PATH")
    else
        CUR_DIR=$(dirname "$CUR_PATH")
    fi

    # Compare against reference
    if [ "$CUR_DIR" != "$REF_DIR" ]; then
        echo "[HOST][ERROR] All input files must be in the same directory."
        echo "[HOST]  Reference: $REF_DIR"
        echo "[HOST]  Mismatch:  $CUR_DIR ($arg)"
        exit 1
    fi
done

# --- 2. Prepare Docker Mounts ---
# We mount the directory containing the inputs to /data
HOST_MOUNT_DIR="$REF_DIR"
echo "[HOST] [INFO] Mounting Host: $HOST_MOUNT_DIR -> Container: /data"

# --- 3. Build Python Arguments ---
# We convert host paths to container paths (relative to /data)
PY_ARGS=""

for arg in "$@"; do
    if [[ "$arg" == -* ]]; then
        # Pass flags through
        PY_ARGS="$PY_ARGS $arg"
    else
        # Just take the filename (or folder name)
        NAME=$(basename "$arg")
        PY_ARGS="$PY_ARGS /data/$NAME"
    fi
done

# --- 4. Run Docker ---
HOST_DATA_DIR="$HOST_MOUNT_DIR" \
CURRENT_UID=$(id -u) \
CURRENT_GID=$(id -g) \
docker compose -f "$SCRIPT_DIR/docker-compose.yml" run --rm converter \
    python3 /home/dev/convert.py $PY_ARGS