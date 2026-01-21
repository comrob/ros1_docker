# ROS1 -> ROS2 (mcap) Bag Conversion

![ROS Logo](https://github.com/comrob/github-media/blob/main/img/image.png?raw=true)

## Quick Start (Demo)

We provide a demo script that **automatically downloads test bagfiles** and runs the conversion logic for you. The script also opens the converted file in [Foxglove](https://foxglove.dev/download) if it is installed.

```bash
# Run the demo (downloads data -> converts -> verifies)
./run_demo.sh

```

*Prefer to test manually? You can download the sample split-bag dataset from [Google Drive here](https://drive.google.com/drive/folders/18X8cS0u-40FkafMt82_sUuik2VwluMv8?usp=drive_link).*

---

## TL;DR

```bash
bash ./install.sh

# Basic conversion
convert_bag /path/to/single_ros1.bag

# Series conversion (auto-split detection)
convert_bag /path/to/ros1_bag_folder --series
convert_bag /path/to/split_0.bag /path/to/split_1.bag --series

```

This repository provides a containerized solution for two primary tasks:

1. **ROSbags Conversion:** A utility to convert ROS 1 (`.bag`) files to ROS 2 (`.mcap`) format. Features include:
* **Plugin System:** Extendable architecture to modify data on the fly (e.g., Debayering images) using Python plugins.
* **Automatic Type Repair:** Migrates custom messages without source code. Automatically fixes ROS1 vs ROS2 incompatibilities (e.g., stripping `seq` from Headers, fixing `CameraInfo` capitalization).
* **Split-Bag Handling:** Automatically detects split bags and injects `/tf_static` into every chunk. Every output file is self-contained and playable.
* **Crash-Safe:** Implements graceful signal handling. Hit `Ctrl+C` anytime, and your MCAP file will still be valid and playable.
* **Zero Dependencies:** Dockerized solution. No need to install ROS 1 or build custom message packages locally.


2. **ROS Noetic Development:** A persistent environment for developing, building, and executing ROS 1 (Noetic) packages.

## Prerequisites

* **Docker:** Ensure Docker and Docker Compose are installed.
* **Permissions:** The current user must have permission to run Docker commands (e.g., be in the `docker` group).

---

## Part 1: Bag Converter

The converter runs as an ephemeral container. It mounts the target data directory, processes the files, and terminates.

### Installation

Run the provided installation script to create a symbolic link for the execution wrapper and pull the docker image.

```bash
./install.sh

```

**Note:** Ensure `~/.local/bin` is in your system `$PATH`.

### Usage

The `convert_bag` command can be executed from any location.

#### 1. Single File Conversion

Converts a single `.bag` file. The output `.mcap` file is created in the same directory as the input.

```bash
convert_bag /path/to/recording.bag

```

#### 2. Series Conversion (Split Bags)

Use the `--series` flag when processing split bag files (e.g., `_0.bag`, `_1.bag`). This mode enables **Static TF Injection** (ensuring all parts have TF data) and generates `metadata.yaml` for seamless playback.

```bash
# Option A: Process all bags in a folder
convert_bag /path/to/data_folder --series

# Option B: Process specific list of files
convert_bag /path/to/data/bag_0.bag /path/to/data/bag_1.bag --series

```

---

### Plugin System

The converter features a modular plugin architecture allowing users to manipulate messages during the conversion process (e.g., debayering images, filtering topics, or anonymizing data).

#### Architecture

The system uses a **hook-based architecture**:

1. **Loader:** `plugin_manager.py` scans the `src/plugins/` directory.
2. **Configuration:** It reads `src/plugins.yaml` to determine which plugins to enable and their execution order.
3. **Execution:** Enabled plugins are initialized and their processing methods are called for every relevant message passing through the converter.

#### Configuration

Plugins are managed via `src/plugins.yaml`.

```yaml
plugins:
  debayer:
    enabled: true
    params:
      topic_pattern: "/camera/image_raw"

```

<details>
<summary><strong>How to Develop a Plugin</strong> (Click to Expand)</summary>

You can add custom logic by adding a Python script to the `src/plugins/` directory.

**1. Reference Example**
See `src/plugins/debayer.py` for a complete working example of how to manipulate image data.

**2. Development Steps**

1. Create a new file (e.g., `src/plugins/my_filter.py`).
2. Implement a class that accepts `config` in its `__init__`.
3. Implement the processing method (refer to the base class or `debayer.py` for the exact signature).
4. Register your plugin in `src/plugins.yaml`.

**3. Hot-Reloading**
Because the `src` folder is mounted into the container, you do **not** need to rebuild the Docker image to test new plugins. Simply edit the Python file and run `convert_bag`.

</details>

---

### Advanced Configuration

The script accepts optional arguments passed directly to the internal Python converter:

* `--out-dir <path>`: Forces a specific output directory.

<details>
<summary><strong>Feature: Auto-Generating ROS 2 Message Definitions</strong> (Click to Expand)</summary>

The conversion tool embeds message definitions directly into the `.mcap` file. Modern visualization tools like **Foxglove Studio** read these embedded schemas automatically.

However, if you want to replay the bag using CLI tools (`ros2 bag play`) or inspect topics (`ros2 topic echo`), your local ROS 2 environment needs the compiled message packages installed.

**Usage:**

1. **Run the extractor (in your ROS 2 environment):**
```bash
python3 additional/extract_mcap_msgs.py /path/to/my_data.mcap --out-dir src/

```


2. **Build the packages:**
```bash
colcon build
source install/setup.bash

```


3. **Check the packages:**
```bash
ros2 interface list | grep <YourCustomMessage>

```



</details>

---

<details>
<summary><strong>Part 2: ROS 1 Development Environment</strong> (Click to Expand)</summary>

The `ros_dev` service provides a full ROS Noetic desktop environment with GUI support.

### Configuration

Map your host directories to the container by editing `docker-compose.yml`:

| Host Path | Container Path | Description |
| --- | --- | --- |
| `~/repos` | `/home/dev/repos` | Source code repositories. |
| `./catkin_ws` | `/home/dev/catkin_ws` | The active Catkin workspace. |

### Workflow

1. **Start the Service:**
```bash
docker compose up -d ros_dev

```


2. **Access the Shell:**
```bash
docker exec -it ros_pet_container bash

```


3. **GUI Visualization:**
If your host supports X11 forwarding, you can run GUI tools (`rviz`) directly.

</details>

---

## Troubleshooting

* **`convert_bag: command not found`**: Add `~/.local/bin` to your `$PATH`.
* **Changes to code not appearing**: The `src` folder is mounted. Changes apply immediately. If you add *system dependencies* (pip/apt), run `docker compose build converter`.

---

<details>
<summary><strong>Improvement Proposals (TODO)</strong> (Click to Expand)</summary>

### Pending Improvements

* **MCAP-to-MCAP Tooling:** Generalize the architecture to support MCAP-to-MCAP manipulation. This would allow using the plugin system (filtering, anonymization) on native ROS 2 data, not just during conversion.
* **Progress Bars:** Replace line-based logging with `tqdm` for visual progress during long conversions.
* **Pre-Flight Checks:** Implement a `--dry-run` mode to validate paths and disk space.
* **Summary Report:** Print a tabulated summary of converted topics and message counts at the end of execution.

### Completed

* [x] **Auto-splitting:** Series conversion with static TF injection is implemented.
* [x] **Plugin System:** Architecture for modular data manipulation is implemented.
* [x] **Automated Image Publishing:** CI/CD scripts (`publish-image.sh`) are in place.

</details>

---

## Maintainer Guide

To update the system dependencies (Dockerfile):

1. **Login:** `echo $GITHUB_TOKEN | docker login ghcr.io -u USER --password-stdin`
2. **Publish:** `./publish-image.sh`