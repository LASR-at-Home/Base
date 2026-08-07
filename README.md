# LASR Base 

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![ROS](https://img.shields.io/badge/ROS-ROS2-version.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)

**LASR Base** is the core ROS codebase maintained by the King College London's **LASR@Home** team at the [Sensible Robots Research Lab](https://www.sensiblerobotsresearch.org/lasr/). 


## Structure

The packages in this repository are organized into distinct layers to separate infrastructure, reusable behaviors, and high-level task execution:

| Folder | Sub-package | Description |
| :--- | :--- | :--- |
| **`common/`** | `foundation_models` | Visual-Language Model (VLM) service for multimodal inference. |
| | `helpers` | Common Python and ROS helper utilities and services. |
| | `interfaces` | Web-based and touch user interfaces for robot control/monitoring. |
| | `language` | LLM inference services for information extraction. |
| | `manipulation` | Grasping and motion control developed for the TIAGo gripper. |
| | `navigation` | Augmentations, planners, and controllers for mobile navigation. |
| | [`simulation`](common/simulation/README.md) | Gazebo simulation environments for testing and development. |
| | `speech` | Whisper based speach recognition and Speech-To-Text (STT). |
| | `vision` | YOLO based services for object detection, segmentation, keypoint estimation and so on. |
| [**`documentation/`**](documentation/README.md) | — | General documenation and Guides. |
| **`skills/`** | — | Fully modular states and state machines providing sub-behaviors (e.g., detection, speak, follow person). |
| **`tasks/`** | — | Full task-level implementations for RoboCup@Home Tasks and complex scenarios. |
---
**sub-packages contain multiple additional packages*

## Quick Start


### Prerequisites
* **Ubuntu 24 OS** (a dedicated partition or device is recommended - however if not possible you can use a Virtual Machine such as [VirutalBox](https://www.virtualbox.org/) for Mac or [WSL](https://ubuntu.com/wsl) for Windows)
* **ROS 2 Humble** [Legacy ROS 1 Noetic codebase](https://github.com/LASR-at-Home/Base/tree/ros1) can be found in the `ros1` branch
* **Python 3.x**
* [**Apptainer**](https://apptainer.org/docs/admin/main/installation.html) required for development on the codebase.

### Apptainer Container 
#### 1. Setup

1. Ensure you are in the root directory  
2. Download `tiago_humble_os.sif` into the root directory *(contact a team member/ Dr. Gerard Canal for access)*
3. Build the container:

   ```bash
   apptainer build ros2.sif ros2.def
   ```

#### 2. Running the Container 

   ```bash
   apptainer run -B /run/user/$UID ros2.sif
   ```

(*Use `--nv` if an NVIDIA GPU is available and you are on Ubuntu 22*)

---

### Building the Workspace

1. Clone this repository into your ROS workspace source directory:
  ```bash
   cd ~/ros_ws/src
   git clone https://github.com/LASR-at-Home/Base.git
  ```

2. Build and source the workspace
  ```bash
  cd ~/ros_ws/
  colcon build
  source install/setup.bash
  ```

3. Run the package <br>
    *(Individual packages can be launched directly through a launch file or executable. Refer to the package's README.md for further information.)*

## External Resources and References

### Core Software and external Packages

- PAL Robotics - [TIAGo](https://docs.pal-robotics.com/25.01/tiago)  
- Locus Robotics - [ament_virtualenv](https://github.com/locusrobotics/ament_virtualenv)  
- Box Robotics. - [ros2 numpy](https://github.com/Box-Robotics/ros2_numpy)  
- ULE Robotics Group - [YASMIN](https://github.com/uleroboticsgroup/yasmin)  