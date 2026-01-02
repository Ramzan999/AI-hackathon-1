---
sidebar_position: 2
---

# Getting Started with Physical AI & Humanoid Robotics

This comprehensive guide will help you set up your development environment for the Physical AI & Humanoid Robotics course. Follow these steps carefully to ensure all required tools and dependencies are properly installed and configured.

## System Requirements

Before beginning the installation process, ensure your system meets the following requirements:

### Minimum Hardware Requirements
- **CPU**: Intel i7 or AMD Ryzen 7 (8+ cores recommended)
- **RAM**: 16GB (32GB recommended for simulation work)
- **GPU**: NVIDIA RTX 3060 or better (CUDA-capable GPU with 8GB+ VRAM)
- **Storage**: 50GB+ free space for all tools and simulations
- **OS**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2

### Recommended Hardware
- **CPU**: Intel i9 or AMD Ryzen 9 (16+ cores)
- **RAM**: 32GB or more
- **GPU**: NVIDIA RTX 4080/4090 or RTX A5000/A6000 (for professional development)
- **Storage**: 100GB+ SSD storage

## Prerequisites Installation

### 1. Install ROS 2 Humble Hawksbill

ROS 2 is the core framework for our robotic systems. Follow these steps to install ROS 2 Humble Hawksbill:

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Set locale to support UTF-8
locale  # Check for UTF-8
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

# Add ROS 2 GPG key and repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list and install ROS 2
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install NVIDIA Drivers and CUDA

For GPU-accelerated simulation and AI processing:

```bash
# Check for NVIDIA GPU
lspci | grep -i nvidia

# Install NVIDIA drivers (if not already installed)
sudo apt install nvidia-driver-535 nvidia-utils-535

# Install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cuda-toolkit-12-3

# Verify installation
nvidia-smi
nvcc --version
```

### 3. Install Python and Development Tools

```bash
# Install Python 3.10+ and development tools
sudo apt install python3.10 python3.10-dev python3.10-venv python3-pip
sudo apt install build-essential cmake git

# Install pip dependencies
pip3 install --upgrade pip
pip3 install colcon-common-extensions vcstool
pip3 install numpy matplotlib opencv-python
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Development Environment Setup

### 1. Create Workspace Directory

```bash
# Create workspace directory
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (should build with no packages initially)
colcon build
source install/setup.bash
```

### 2. Install Gazebo Harmonic

```bash
# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-ros2-control-demos

# Test Gazebo installation
gz sim -v
```

### 3. Install Isaac Sim Prerequisites

For NVIDIA Isaac Sim, you'll need to download it separately:

1. Go to [NVIDIA Developer Portal](https://developer.nvidia.com/isaac-sim)
2. Download Isaac Sim for your platform
3. Follow the installation instructions provided by NVIDIA

After installation, verify the setup:

```bash
# Navigate to Isaac Sim directory and run
cd /path/to/isaac-sim
./isaac-sim.sh
```

### 4. Install Isaac ROS Dependencies

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-perception
sudo apt install ros-humble-isaac-ros-nav2
sudo apt install ros-humble-isaac-ros-bi-connector
sudo apt install ros-humble-isaac-ros-ros-bridge

# Install additional dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard
```

## Project Setup

### 1. Clone Course Repository

```bash
# Navigate to your workspace
cd ~/robotics_ws/src

# Clone the course repository (or create your own structure)
git clone https://github.com/your-username/robotics-book.git

# Or if starting fresh, create the structure
mkdir -p robotics_course/{launch,config,models,worlds}
```

### 2. Install Course Dependencies

```bash
# Navigate to your workspace
cd ~/robotics_ws

# Install Python dependencies for the course
pip3 install -r src/robotics_course/requirements.txt

# Example requirements.txt content:
# numpy>=1.21.0
# opencv-python>=4.5.0
# transforms3d>=0.4.0
# pyquaternion>=0.9.0
# open3d>=0.16.0
```

### 3. Build the Workspace

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
cd ~/robotics_ws
colcon build --packages-select robotics_course

# Source the workspace
source install/setup.bash
```

## Testing Your Setup

### 1. Verify ROS 2 Installation

```bash
# Test basic ROS 2 functionality
ros2 topic list
ros2 service list
ros2 node list

# Test creating a simple publisher/subscriber
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
```

### 2. Test Gazebo Installation

```bash
# Launch Gazebo with a simple world
gz sim -r empty.sdf

# Or use ROS 2 Gazebo integration
ros2 launch gazebo_ros empty_world.launch.py
```

### 3. Test Python Environment

Create a simple test script to verify everything works:

```python
#!/usr/bin/env python3
# test_setup.py
import rclpy
from rclpy.node import Node
import numpy as np
import cv2

class SetupTestNode(Node):
    def __init__(self):
        super().__init__('setup_test')
        self.get_logger().info('ROS 2 setup test node started')
        self.get_logger().info(f'NumPy version: {np.__version__}')
        self.get_logger().info(f'OpenCV version: {cv2.__version__}')

def main(args=None):
    rclpy.init(args=args)
    node = SetupTestNode()

    # Test basic operations
    test_array = np.array([1, 2, 3, 4, 5])
    self.get_logger().info(f'Test array: {test_array}')

    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run the test:
```bash
python3 test_setup.py
```

## Common Issues and Troubleshooting

### 1. ROS 2 Environment Not Sourced
**Issue**: Commands like `ros2` not found
**Solution**: Ensure ROS 2 environment is sourced in your shell profile:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. GPU Not Detected
**Issue**: CUDA operations fail
**Solution**: Verify NVIDIA drivers and CUDA installation:
```bash
nvidia-smi
nvcc --version
```

### 3. Permission Issues
**Issue**: Cannot access serial ports or hardware
**Solution**: Add user to required groups:
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
newgrp dialout
```

### 4. Package Build Failures
**Issue**: `colcon build` fails with dependency errors
**Solution**: Ensure all dependencies are installed:
```bash
cd ~/robotics_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Next Steps

Once your environment is properly set up, proceed to:

1. **Module 1: ROS 2 Nervous System** - Learn the fundamentals of ROS 2 architecture
2. **Module 2: Simulation Environments** - Explore Gazebo and Isaac Sim
3. **Module 3: NVIDIA Isaac Brain** - Master AI-powered perception
4. **Module 4: Vision-Language-Action Systems** - Build multimodal AI systems

## Development Workflow

### Daily Development Routine
1. Open a new terminal
2. Source ROS 2 and your workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   cd ~/robotics_ws && source install/setup.bash
   ```
3. Navigate to your development directory
4. Start coding and testing

### Workspace Management
- Keep your source code in `~/robotics_ws/src/`
- Build products appear in `~/robotics_ws/build/` and `~/robotics_ws/install/`
- Always source the workspace after building: `source install/setup.bash`

Your development environment is now ready for the Physical AI & Humanoid Robotics course. Proceed to the first module to begin your journey into embodied intelligence!