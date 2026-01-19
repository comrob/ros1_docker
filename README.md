# ROS Development & Bag Conversion Toolkit

This repository provides a containerized environment for:
1.  **Converting** ROS 1 `.bag` files to ROS 2 `.mcap` files from any location.
2.  **Developing** and running ROS 1 (Noetic) packages.

## 📂 Directory Setup

Ensure your host directories are set up as defined in `docker-compose.yml`.

| Host Path | Container Path | Purpose |
| :--- | :--- | :--- |
| `~/repos` | `/home/dev/repos` | Source code repositories. |
| `./catkin_ws` | `/home/dev/catkin_ws` | Your ROS workspace. |
| `~/bag` | `/home/dev/bags` | **Legacy/Dev only:** Default bag location for the development container. |
| `.` (Current Dir) | `/app` | Internal scripts. |

---

## ⚙️ Installation (One-Time Setup)

Before using the converter, you must build the image and set up the helper script.

### 1. Build the Converter Image
This bakes the conversion script into the Docker image:
```bash
docker compose build converter

```

### 2. Install the Wrapper Script

To run the conversion from any directory, create an alias or symlink to `convert_bag.sh`.

**Option A: System-wide (Recommended)**

```bash
chmod +x convert_bag.sh
# Ensure ~/.local/bin exists and is in your PATH
mkdir -p ~/.local/bin
ln -s $(pwd)/convert_bag.sh ~/.local/bin/convert_bag

```

**Option B: Shell Alias**
Add this to your `~/.bashrc`:

```bash
alias convert_bag='/path/to/your/docker_ros/convert_bag.sh'

```

---

## 🚀 1. Converting Bag Files

You can now convert files located **anywhere** on your system without moving them.

### Usage

Simply run the command followed by the path to your bag file:

```bash
convert_bag /path/to/my_recording.bag

```

**Features:**

* **Automatic Output Name:** By default, it creates `/path/to/my_recording.mcap`.
* **Optional Output Name:** You can specify a custom output name as a second argument:
```bash
convert_bag my_recording.bag my_custom_name.mcap

```


* **Safety:** The script refuses to overwrite the input file if you accidentally specify the same name.

---

## 🛠️ 2. Running ROS 1 Packages

The `ros_dev` service is a persistent container for development, compilation, and execution (including GUI tools like Rviz).

### Start the Environment

Start the container in the background:

```bash
docker compose up -d ros_dev

```

### Enter the Container

Open a shell inside the running container:

```bash
docker exec -it ros_pet_container bash

```

### Workflow inside the Container

Once inside, you are user `dev` in `~/catkin_ws`.

1. **Build Workspace:**
```bash
catkin build

```


2. **Source Environment:**
```bash
source devel/setup.bash

```


3. **Run Packages:**
```bash
roslaunch my_package my_launchfile.launch

```


4. **GUI Tools:**
If you have X11 forwarding configured (handled by `DISPLAY` env var), you can run GUI apps directly:
```bash
roscore &
rviz

```



### Stopping the Environment

To stop and remove the dev container:

```bash
docker compose down

```

---

## ⚠️ Troubleshooting

**"docker: command not found" inside the script**

* **Cause:** Docker is not installed or the user does not have permission to run it.
* **Fix:** Ensure you can run `docker ps` without sudo.

**"File not found"**

* **Cause:** The script checks for the file before running Docker.
* **Fix:** Ensure the path provided to `convert_bag` is correct relative to your current terminal location.

**Changes to `ros1_to_mcap.py` not appearing**

* **Cause:** The Python script is copied into the Docker image during the build.
* **Fix:** If you edit the Python code, you must run `docker compose build converter` again.
