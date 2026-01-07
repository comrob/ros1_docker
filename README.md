# ROS Development & Bag Conversion Toolkit

This repository provides a containerized environment for:
1.  **Converting** ROS 1 `.bag` files to ROS 2 `.mcap` files.
2.  **Developing** and running ROS 1 (Noetic) packages.

## 📂 Directory Setup

Ensure your host directories are set up as defined in `docker-compose.yml`:

| Host Path | Container Path | Purpose |
| :--- | :--- | :--- |
| `~/bag` | `/home/dev/bags` | **Place your input `.bag` files here.** |
| `~/repos` | `/home/dev/repos` | Source code repositories. |
| `./catkin_ws` | `/home/dev/catkin_ws` | Your ROS workspace. |
| `.` (Current Dir) | `/app` | Scripts (`ros1_to_mcap.py`). |

---

## 🚀 1. Converting Bag Files

The converter utility maps your host's `~/bag` directory directly to the container's working directory.

### Step 1: Prepare the Data
Move your `.bag` files into `~/bag` on your host machine.
* Example: `mv my_recording.bag ~/bag/`

### Step 2: Run the Conversion
Run the following command from this directory (where `docker-compose.yml` is located):

```bash
docker compose run --rm converter python3 /app/ros1_to_mcap.py input.bag output.mcap

```

**Notes:**

* **Pathing:** Do not add `~/bag/` to the command arguments. The container is already inside that folder. Just use the filename (e.g., `input.bag`).
* **Subfolders:** If you have `~/bag/dataset1/test.bag`, use `dataset1/test.bag`.
* **Safety:** The script includes a safety check to prevent overwriting the input file.

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

**"Got only 0 of requested bytes"**

* **Cause:** You likely used the same filename for input and output.
* **Fix:** Ensure your output filename is different (e.g., add `.mcap` extension). Check file sizes in `~/bag` to see if the original was overwritten.

**"File not found" during conversion**

* **Cause:** The script cannot see the file.
* **Fix:** Ensure the file is actually inside `~/bag` on your host, not in the local `bags` folder inside the docker project directory (unless you symlinked them).