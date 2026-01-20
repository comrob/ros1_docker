#!/bin/bash

SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# --- Minimal Argument Processing ---
# We just ensure that if you provide a file path, we convert it to a 
# relative path so it works inside the container (which mounts $PWD).

PY_ARGS=""

for arg in "$@"; do
    # Check if the argument is a file or directory that exists on the host
    if [ -e "$arg" ]; then
        # Convert absolute paths (or ../ paths) to purely relative paths
        # e.g., /home/user/data/bag.bag -> data/bag.bag
        REL_PATH=$(realpath --relative-to="$PWD" "$arg")
        PY_ARGS="$PY_ARGS $REL_PATH"
    else
        # It's likely a flag (like --series) or a string (humble), pass it as-is
        PY_ARGS="$PY_ARGS $arg"
    fi
done

# --- Run Docker ---
# We mount the Current Directory ($PWD) to /data.
# The container's working_dir is /data, so relative paths work perfectly.

echo "[HOST] Running in: $PWD"
echo "[HOST] Arguments:  $PY_ARGS"

HOST_DATA_DIR="$PWD" \
CURRENT_UID=$(id -u) \
CURRENT_GID=$(id -g) \
docker compose -f "$SCRIPT_DIR/docker-compose.yml" run --rm converter \
    python3 /home/dev/convert.py $PY_ARGS