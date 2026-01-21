# Batch Processing & Organization Tools

This directory contains utility scripts to automate the organization and conversion of large datasets. The intended workflow is **Group -> Convert**.

## 1. Organize Data (`group_bags.py`)

The **Bag Converter** requires that split bag files (e.g., `_0.bag`, `_1.bag`) reside in their own dedicated subfolder to correctly generate metadata and inject TF data.

This script recursively scans a directory and moves loose bag files into dedicated folders based on their timestamps and sequence indices.

### Usage

```bash
python3 group_bags.py [ROOT_DIR] [--dry-run]

```

### Behavior

* **Input:** `my_dataset/experiment_2025-07-10-14-22-04_0.bag` (and `_1.bag`, etc.)
* **Action:** Creates folder `my_dataset/experiment_2025-07-10-14-22-04/` and moves the files inside.
* **Dry Run:** Use `--dry-run` to see what would happen without moving any files.

---

## 2. Batch Conversion (`batch_converter.py`)

Once your data is organized, this script converts the entire folder structure to MCAP. It mirrors your input directory structure to the output directory and skips folders that have already been converted.

### Features

* **Zero Dependencies:** Uses standard Python libraries (no `pip install` required).
* **Mirroring:** Recreates your input folder structure in the output location.
* **Resumable:** Checks for `metadata.yaml` in the destination to skip finished jobs.
* **Notifications:** Supports `ntfy.sh` to send phone/desktop alerts on errors or completion.

### Usage

```bash
python3 batch_converter.py \
  --input /path/to/ros1_data \
  --output /path/to/mcap_data \
  [--with-plugins] \
  [--ntfy TOPIC_NAME] \
  [--dry-run]

```

### Arguments

| Argument | Description |
| --- | --- |
| `--input` | Root directory containing the organized ROS1 folders. |
| `--output` | Target root directory for MCAP files. |
| `--with-plugins` | Pass this flag to enable plugins (e.g., Debayer) defined in `plugins.yaml`. |
| `--ntfy` | (Optional) A topic name for [ntfy.sh](https://ntfy.sh) notifications. |
| `--converter` | Command to run the underlying converter (default: `convert_bag`). |

---

## Workflow Example

### Step 1: Group loose files

First, tidy up the raw data folder.

```bash
# Preview changes
python3 group_bags.py /mnt/external_drive/raw_data --dry-run

# Apply changes
python3 group_bags.py /mnt/external_drive/raw_data

```

### Step 2: Run Batch Conversion

Convert everything to a new location, applying plugins, and get notified on your phone when done.

```bash
python3 batch_converter.py \
  --input /mnt/external_drive/raw_data \
  --output /mnt/external_drive/mcap_data \
  --with-plugins \
  --ntfy my_robot_data_updates

```

**Result:**
You will receive a notification:

> 🚀 **Batch Started:** Started batch conversion of 50 folders.

Followed by progress updates or error alerts if a specific bag fails.

### Step 3: Verify

The script provides a final summary. You can also verify the output structure:

```bash
tree /mnt/external_drive/mcap_data

```