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

---

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML format that describes your robot's physical structure. It defines:
- **Links** - Rigid bodies with visual geometry, collision properties, and inertia
- **Joints** - Connections between links that define how they move relative to each other

The robot is organized as a **tree of coordinate frames**, where each link has its own coordinate system and joints define the parent-child relationships.

---

## Understanding Links

A **link** represents a rigid body in your robot. Each link can have:

| Element | Purpose |
|---------|---------|
| `<visual>` | What you see in RViz (geometry, color, origin) |
| `<collision>` | Simplified shape for physics/collision detection |
| `<inertial>` | Mass and inertia properties for simulation |

### Simple Link Example

```xml
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
</link>
```

This creates a blue box (0.4m x 0.3m x 0.1m) positioned 5cm above the origin.

---

## Understanding Joints

A **joint** connects two links and defines their relative motion:

| Joint Type | Behavior |
|------------|----------|
| `fixed` | No movement - links are rigidly attached |
| `continuous` | Infinite rotation (wheels) |
| `revolute` | Limited rotation with min/max limits |
| `prismatic` | Sliding linear motion |

### Simple Joint Example

```xml
<joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 -0.15 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
</joint>
```

This attaches `wheel_link` to `base_link`, positioned 15cm to the left, rotating around the Y-axis.

---

## The Robot

Open `my_robot_description/urdf/my_robot.urdf` to see the complete robot definition. It defines a differential drive robot with:
- A base link (box)
- Two wheels (cylinders) connected via continuous joints

---

## Launch File

To visualize the robot, you need three essential nodes running together. Create a launch file that starts:

| Node | Package | Purpose |
|------|---------|---------|
| `joint_state_publisher` | joint_state_publisher | Publishes joint positions |
| `robot_state_publisher` | robot_state_publisher | Reads URDF and publishes TF transforms |
| `rviz2` | rviz2 | 3D visualization |

See `my_robot_description/launch/view_robot.launch.py` for the complete implementation.

---

## Build and Run

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch my_robot_description view_robot.launch.py
```

---

## Using 3D Meshes (STL files)

Instead of primitive shapes (boxes, cylinders), you can use 3D models:

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

**Key points:**
- Supported formats: `.stl`, `.dae` (Collada), `.obj`
- The `scale` attribute converts from millimeters (CAD default) to meters (ROS standard)
- Place mesh files in the `meshes/` directory of your package

---

## What's Next: XACRO

URDF files can become verbose and repetitive. The next branch (`feature/07-xacro`) introduces **XACRO (XML Macro)**, which lets you:
- Define reusable macros (e.g., a wheel macro used twice)
- Use properties and variables
- Include other XACRO files for modular design

This simplifies complex robot descriptions significantly.
