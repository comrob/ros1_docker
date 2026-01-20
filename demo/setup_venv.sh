#!/bin/bash
set -e

VENV_DIR=".venv"

# 1. Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "[SETUP] Creating Python virtual environment in $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
else
    echo "[SETUP] Virtual environment found."
fi

# 2. Install gdown if not present
if [ ! -f "$VENV_DIR/bin/gdown" ]; then
    echo "[SETUP] Installing gdown..."
    "$VENV_DIR/bin/pip" install --upgrade pip
    "$VENV_DIR/bin/pip" install gdown
fi