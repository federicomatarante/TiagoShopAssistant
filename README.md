# ROS 2 Workspace – Node and Service Generator

This is a ROS 2 workspace with helper scripts to quickly create Python-based nodes and services. It includes example scripts to create and remove nodes/services dynamically, a setup script, and an automatic runner script.

---

## 📋 System Requirements

- **Ubuntu 22.04.5 LTS (Jammy Jellyfish)**
- **ROS 2 Humble Hawksbill**
- **Python 3.10.12**

---

## 🗂 Project Structure

```
ros2_ws/
├── src/                      # ROS 2 packages go here
├── create_node.py           # Script to generate a ROS 2 Python node
├── create_service.py        # Script to generate a ROS 2 Python service
├── remove_node.py           # Script to delete a previously created node
├── remove_service.py        # Script to delete a previously created service
├── start_project.bash       # Setup script (e.g., for sourcing env and creating base structure)
├── run.bash                 # 🔁 Script that builds and runs all nodes automatically
├── build/                   # Build artifacts (ignored by Git)
├── install/                 # Install tree (ignored by Git)
├── log/                     # Colcon logs (ignored by Git)
└── .gitignore               # Git ignore rules
```

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone git@github.com:federicomatarante/ros2_ws.git
cd ros2_ws
```

### 2. Source ROS 2 Humble Environment

```bash
source /opt/ros/humble/setup.bash
```

> 💡 Make sure ROS 2 Humble is installed. You can follow the official guide: https://docs.ros.org/en/humble/Installation.html

### 3. Run the Project Setup Script

```bash
bash start_project.bash
```

This script can automatically set up folders, permissions, and any default structure you want.

---

## ⚙️ Building the Workspace

```bash
colcon build
```

After building, source the local workspace:

```bash
source install/setup.bash
```

Or simply:

```bash
. install/setup.bash
```

---

## 🧱 Creating a Node

To generate a new node:

```bash
python3 create_node.py <node_name>
```

Example:

```bash
python3 create_node.py my_node
```

This will create a Python ROS 2 node named `my_node.py` in the appropriate `src/` package folder.

---

## 🧩 Creating a Service

To generate a new service:

```bash
python3 create_service.py <service_name>
```

Example:

```bash
python3 create_service.py add_numbers
```

---

## 🗑 Removing a Node or Service

```bash
python3 remove_node.py <node_name>
python3 remove_service.py <service_name>
```

These scripts clean up any auto-generated files or references.

---

## ▶️ Running a Node

Manually:

```bash
ros2 run <package_name> <executable_name>
```

Example:

```bash
ros2 run my_package my_node
```

✅ **OR use the auto-run script**:

```bash
bash run.bash
```

This script:
- Sources ROS 2 and the workspace
- Builds the workspace
- Launches all configured nodes automatically

---

## 📦 Notes

- All scripts assume a typical ROS 2 Python package structure.
- You can customize `create_node.py` and `create_service.py` to fit your preferred code template.
- Update `run.bash` to launch only the nodes you want, using `ros2 run` or a `ros2 launch` command.

---

## 📝 License

MIT License. See `LICENSE` for details.
