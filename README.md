# Feature 06: Robot Description Package (URDF)

This branch covers creating a robot description package with URDF and visualizing it in RViz.

---

## Prerequisites

Complete these branches first:
- `feature/01-turtlesim` - ROS 2 basics
- `feature/02-publisher-subscriber` - Nodes and topics
- `feature/04-launch-files` - Launch system

---

## Learning Objectives

By the end of this branch, you will:
- Understand what URDF is and why it's used
- Create a robot description package
- Define links and joints in URDF
- Visualize your robot in RViz
- Troubleshoot common URDF/RViz issues

---

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML format that describes your robot's:
- **Physical structure** - Links (rigid bodies) and joints (connections)
- **Visual geometry** - What the robot looks like (colors, shapes, meshes)
- **Collision properties** - Simplified geometry for collision detection
- **Inertial properties** - Mass and inertia for simulation

### The Robot Tree

ROS 2 robots are organized as a **tree of coordinate frames**:

```
base_link (root)
    ├── wheel_left_link
    └── wheel_right_link
```

- **Links** = rigid bodies (the "nodes" of the tree)
- **Joints** = connections between links (the "edges" of the tree)
- **TF transforms** = mathematical relationship between coordinate frames

---

## Creating the Robot Description Package

### Step 1: Create the Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_description
cd my_robot_description
```

### Step 2: Create Directory Structure

```bash
mkdir -p urdf meshes config launch
```

| Directory | Purpose |
|-----------|---------|
| `urdf/` | URDF robot description files |
| `meshes/` | 3D model files (STL, DAE) |
| `config/` | RViz configurations |
| `launch/` | Launch files |

### Step 3: Create the URDF File

Create `urdf/my_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.4 0.3 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="blue">
                <color rgba="0 0 0.8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.4 0.3 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>
    </link>

    <!-- Right wheel -->
    <link name="wheel_right_link">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.02"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.57 0 0"/>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1"/>
            </material>
        </visual>
    </link>

    <!-- Joint: base_link → wheel_right_link -->
    <joint name="wheel_right_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_right_link"/>
        <origin xyz="0 -0.15 0.05" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <!-- Left wheel -->
    <link name="wheel_left_link">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.02"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.57 0 0"/>
            <material name="gray"/>
        </visual>
    </link>

    <!-- Joint: base_link → wheel_left_link -->
    <joint name="wheel_left_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_left_link"/>
        <origin xyz="0 0.15 0.05" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

</robot>
```

### URDF Element Breakdown

#### Link Elements

| Element | Purpose |
|---------|---------|
| `<visual>` | What you see in RViz |
| `<collision>` | Simplified shape for physics |
| `<inertial>` | Mass properties for simulation |

#### Joint Types

| Type | Behavior |
|------|----------|
| `fixed` | No movement |
| `continuous` | Infinite rotation (like wheels) |
| `revolute` | Limited rotation (with limits) |
| `prismatic` | Sliding motion |

#### Origin Attributes

- `xyz` - Position offset (x, y, z in meters)
- `rpy` - Orientation (roll, pitch, yaw in radians)

---

### Step 4: Update package.xml

Add these dependencies:

```xml
<depend>urdf</depend>
<depend>xacro</depend>
<depend>joint_state_publisher</depend>
<depend>robot_state_publisher</depend>
<depend>rviz2</depend>
```

---

### Step 5: Update setup.py

Add URDF and launch files to `data_files`:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robot description package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
```

---

## Creating the Launch File

Create `launch/view_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Publish joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # Publish robot transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot_description'),
                'config',
                'urdf.rviz'
            )]
        )
    ])
```

### What Each Node Does

| Node | Purpose |
|------|---------|
| `joint_state_publisher` | Publishes joint positions (required for TF) |
| `robot_state_publisher` | Reads URDF and publishes TF transforms |
| `rviz2` | 3D visualization |

---

## Creating the RViz Configuration

Create `config/urdf.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1
      Name: Current View
      Pitch: 0.5
      Yaw: 0.8
```

---

## Build and Run

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch my_robot_description view_robot.launch.py
```

### In RViz

1. Verify **RobotModel** display is enabled (green checkbox)
2. Set **Fixed Frame** to `base_link` (top left)
3. You should see a blue box with two gray cylinders (wheels)

Use your mouse:
- **Left-click + drag** - Rotate camera
- **Right-click + drag** - Pan
- **Scroll wheel** - Zoom

---

## Visualizing the TF Tree

To see your robot's coordinate frame tree:

```bash
ros2 run tf2_tools view_frames.py
evince frames.pdf    # or open with any PDF viewer
```

This generates a diagram showing:
- All coordinate frames
- Parent-child relationships
- Transform values

---

## Troubleshooting

### Robot Not Showing in RViz

**Symptom:** Grid visible, but no robot

**Causes:**
1. **Wrong Fixed Frame** - Set it to `base_link`
2. **RobotModel not enabled** - Check the green checkbox
3. **robot_state_publisher not running** - Check launch output

**Debug:**
```bash
ros2 topic list              # Should see /robot_description
ros2 topic echo /tf          # Should see transform data
```

### Robot Appears at Origin Only

**Symptom:** Robot visible but can't move/rotate

**Cause:** `joint_state_publisher` not publishing

**Debug:**
```bash
ros2 node list               # Should see joint_state_publisher
ros2 topic echo /joint_states
```

### "Package not found" Error

**Cause:** Workspace not sourced after build

**Fix:**
```bash
source ~/ros2_ws/install/setup.bash
```

### RViz Shows "No Transform"

**Cause:** TF tree not publishing

**Fix:**
1. Check that both `joint_state_publisher` and `robot_state_publisher` are running
2. Verify URDF has valid link/joint hierarchy

---

## Using Meshes Instead of Primitives

Replace primitive shapes with 3D models:

```xml
<link name="base_link">
    <visual>
        <geometry>
            <mesh filename="package://my_robot_description/meshes/base.stl"
                  scale="0.001 0.001 0.001"/>
        </geometry>
    </visual>
</link>
```

**Important:** The `scale` attribute converts from millimeters (CAD default) to meters (ROS standard).

**Supported formats:** `.stl`, `.dae` (Collada), `.obj`

---

## Next: Xacro (XML Macros)

For complex robots, URDF becomes repetitive. The next branch (`feature/07-xacro`) covers:
- Defining reusable components (e.g., "wheel" macro)
- Using properties and math expressions
- Including sub-files for modular design

---

## Summary

You've learned:
- URDF describes robot structure (links + joints)
- Launch files start multiple nodes together
- RViz visualizes the robot model
- TF transforms connect coordinate frames

Proceed to `feature/07-xacro` to learn how to simplify complex URDFs.
