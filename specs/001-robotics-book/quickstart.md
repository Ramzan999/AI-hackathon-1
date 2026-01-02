# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

Before starting with the Physical AI & Humanoid Robotics course materials, ensure you have:

- **Node.js** version 18 or higher
- **Python** version 3.8 or higher
- **ROS 2 Humble Hawksbill** installed and configured
- **Git** for version control
- Basic understanding of Python programming
- Familiarity with command-line tools

## Setup Development Environment

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Node.js Dependencies
```bash
npm install
```

### 3. Install Python Dependencies
```bash
pip install rclpy numpy matplotlib torch
```

### 4. Set Up ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # If you have built ROS 2 packages locally
```

## Local Development

### 1. Start Docusaurus Development Server
```bash
npm start
```
This will start a local server at `http://localhost:3000` with hot reloading.

### 2. Build Static Site
```bash
npm run build
```
This generates the static site in the `build` directory.

### 3. Serve Built Site Locally
```bash
npm run serve
```
This serves the built site at `http://localhost:3000` for testing.

## Project Structure Overview

The documentation site is organized into four main modules:

1. **ROS 2 Nervous System** - Core communication and architecture
2. **Simulation** - Gazebo and Unity environments
3. **NVIDIA Isaac Brain** - Perception and decision-making
4. **VLA Systems** - Vision-Language-Action integration

Each module contains:
- `concepts.md` - Theoretical foundations
- `hands-on-lab.md` - Practical exercises
- `troubleshooting.md` - Common issues and solutions

## Adding New Content

### 1. Create a New Page
Add a new markdown file in the appropriate module directory:
```
docs/ros2-nervous-system/new-topic.md
```

### 2. Update Sidebar
Add the new page to `sidebar.js` to make it appear in navigation:
```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'ROS 2 Nervous System',
      items: [
        'ros2-nervous-system/concepts',
        'ros2-nervous-system/hands-on-lab',
        'ros2-nervous-system/troubleshooting',
        'ros2-nervous-system/new-topic',  // Add your new page here
      ],
    },
    // ... other modules
  ],
};
```

### 3. Using Code Examples
Include code snippets with proper language annotation:
```md
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
<TabItem value="python" label="Python" default>

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
```

</TabItem>
<TabItem value="cpp" label="C++">

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  }
```

</TabItem>
</Tabs>
```

## Deployment

### GitHub Pages Setup
1. Ensure your repository has GitHub Pages enabled
2. Set the source to "GitHub Actions"
3. The deployment workflow is defined in `.github/workflows/deploy.yml`

### Manual Deployment
```bash
GIT_USER=<Your GitHub Username> USE_SSH=true npm run deploy
```

This command builds the site and pushes the static files to the `gh-pages` branch.

## Troubleshooting

### Common Issues

1. **Module not found errors**: Run `npm install` to install missing dependencies
2. **Python ROS 2 modules not found**: Ensure ROS 2 environment is sourced
3. **Port already in use**: Use `npm start -- --port 3001` to use a different port
4. **Build fails**: Check for syntax errors in markdown files and ensure all links are valid

### Getting Help
- Check the `troubleshooting.md` file in each module
- Review the ROS 2 documentation for ROS-specific issues
- Verify your environment matches the prerequisites