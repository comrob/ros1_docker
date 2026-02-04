#!/bin/bash

# Configuration
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"

if [ -z "$1" ]; then
    echo "Usage: convert_bag <input_path> [options]"
    exit 1
fi

# ==============================================================================
# 1. DETERMINISTIC MOUNT CALCULATION
# ==============================================================================

# A. Handle INPUT (Assume $1 is always the input)
# ------------------------------------------------------------------------------
INPUT_ABS=$(realpath -m "$1")
MOUNT_HOST=$(dirname "$INPUT_ABS")

# B. Handle OUTPUT (Scan specifically for --out-dir)
# ------------------------------------------------------------------------------
OUTPUT_ABS=""
ARGS=("$@")
for ((i=0; i<$#; i++)); do
    if [[ "${ARGS[i]}" == "--out-dir" ]]; then
        # The NEXT argument is the output path
        NEXT_IDX=$((i+1))
        raw_out="${ARGS[NEXT_IDX]}"
        
        # Create output parent on host now to ensure permissions/existence
        mkdir -p "$(dirname "$raw_out")"
        
        OUTPUT_ABS=$(realpath -m "$raw_out")
        
        # Expand MOUNT_HOST to include this output path
        while [[ "$OUTPUT_ABS" != "$MOUNT_HOST"* ]]; do
            MOUNT_HOST=$(dirname "$MOUNT_HOST")
            if [[ "$MOUNT_HOST" == "/" ]]; then break; fi
        done
        break
    fi
done

# ==============================================================================
# 2. STRICT RELATIVIZATION
# ==============================================================================
PY_ARGS=""
skip_next=false

for arg in "$@"; do
    if [ "$skip_next" = true ]; then
        skip_next=false
        continue
    fi
    
    # [FIX] Skip flags starting with '-' so realpath doesn't crash on them
    if [[ "$arg" == -* ]]; then
        PY_ARGS="$PY_ARGS $arg"
        continue
    fi

    # Case 1: It matches the known INPUT path
    if [[ "$(realpath -m "$arg")" == "$INPUT_ABS" ]]; then
        REL_PATH=$(realpath -m --relative-to="$MOUNT_HOST" "$INPUT_ABS")
        PY_ARGS="$PY_ARGS $REL_PATH"

    # Case 2: It matches the known OUTPUT path
    elif [[ -n "$OUTPUT_ABS" && "$(realpath -m "$arg")" == "$OUTPUT_ABS" ]]; then
        REL_PATH=$(realpath -m --relative-to="$MOUNT_HOST" "$OUTPUT_ABS")
        PY_ARGS="$PY_ARGS $REL_PATH"
        
    # Case 3: Pass everything else through untouched (Topics, Numbers, etc)
    else
        PY_ARGS="$PY_ARGS $arg"
    fi
done

# ==============================================================================
# 3. EXECUTION
# ==============================================================================
echo "[DEBUG] Host Mount: $MOUNT_HOST"
echo "[DEBUG] Docker Args: $PY_ARGS"

export HOST_DATA_DIR="$MOUNT_HOST"
export CURRENT_UID=$(id -u)
export CURRENT_GID=$(id -g)

docker compose -f "$COMPOSE_FILE" run --rm \
    -e HOST_DATA_DIR \
    -w /data \
    converter \
    python3 /home/dev/src/convert.py $PY_ARGS