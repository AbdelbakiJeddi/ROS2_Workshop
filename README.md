# ROS 2 Introduction Workshop

Welcome to the **ROS 2 Introduction Workshop** repository!

This repository contains all the material, code examples, and useful resources used during the workshop. The goal is to give you a **practical and beginner-friendly introduction to ROS 2**, with hands-on examples you can run on your own machine.

---

## Workshop Objectives

By the end of this workshop, you should be able to:

- Understand what ROS 2 is and why it is used
- Explain core ROS 2 concepts (nodes, topics, services, packages)
- Create and run your own ROS 2 nodes
- Build and run a ROS 2 workspace
- Use basic ROS 2 command-line tools

---

## Prerequisites

### Operating System

- Ubuntu 24.04 or 22.04

### ROS 2 Distribution

- **ROS 2 Jazzy or Humble** (LTS, recommended)

### Required Skills

- Basic Linux terminal usage
- Basic Python programming (helpful but not mandatory)

---

## Installation Guide

### Install ROS 2

Follow the official ROS 2 installation guide:
👉 <https://docs.ros.org/en/jazzy/Installation.html>

After installation, don't forget to source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

(Optional) Add it permanently:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

---

## Repository Structure

### ROS 2 Workspace Structure

```text
ros2_ws/                        # Workspace root
├── src/                        # Source space (your packages go here)
│   └── my_robot_package/
├── build/                      # Build space (auto-generated)
├── install/                    # Install space (auto-generated)
└── log/
```

### Python Package Structure

```text
my_robot_package/
├── package.xml                 # Package metadata and dependencies
├── setup.py                    # Python package setup (for Python packages)
├── setup.cfg                   # Python package configuration
├── my_robot_package/           # Python source code directory
│   └── __init__.py             # Makes it a Python package
```

---

## Getting Started

### Running Your First ROS 2 Nodes (Turtlesim)

Before diving into custom code, let's make sure your ROS 2 installation works by running a classic demo: **Turtlesim** 🐢

#### Run the Turtlesim Node 🐢

Open a terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

> You should see a small window with a turtle.

#### Control the Turtle

Open a new terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

> Use your keyboard arrows to move the turtle.

#### Explore What's Running

While Turtlesim is running, try the following commands in another terminal:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /turtle1/pose
```

#### Visualize the System (Optional)

```bash
rqt_graph
```

This will show a graph of nodes and topics.

---

## Hands-On Examples

### Overview

In this guide, you will create two ROS 2 nodes:

- A **publisher node** that sends messages on a topic
- A **subscriber node** that receives and prints those messages

This is the fundamental communication pattern in ROS 2.

---

### Package Structure

Your package should have this structure:

```text
py_example/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── py_example
└── py_example/
    ├── __init__.py
    ├── publisher_node.py
    └── subscriber_node.py
```

---

## Publisher Node

Create the file `py_example/publisher_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0
        self.get_logger().info('Publisher node started!')

    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS 2! {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Publisher: Code Explanation

**Imports:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
```

- `rclpy`: The ROS 2 Python client library
- `Node`: Base class for all ROS 2 nodes
- `String`: Message type we'll use for communication

**Node Initialization:**

```python
def __init__(self):
    super().__init__('simple_publisher')
```

- Creates a node with the name `simple_publisher`

**Creating the Publisher:**

```python
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```

- `String`: Message type to publish
- `'chatter'`: Topic name
- `10`: Queue size (how many messages to buffer)

**Creating a Timer:**

```python
self.timer = self.create_timer(1.0, self.publish_message)
```

- Calls `publish_message()` every 1.0 second

**Publishing Messages:**

```python
def publish_message(self):
    msg = String()
    msg.data = f'Hello ROS 2! {self.counter}'
    self.publisher_.publish(msg)
```

- Creates a message
- Sets the data field
- Publishes it to the topic

---

## Subscriber Node

Create the file `py_example/subscriber_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscriber node started!')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber: Code Explanation

**Creating the Subscription:**

```python
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10
)
```

- `String`: Message type to subscribe to
- `'chatter'`: Topic name (must match publisher)
- `self.listener_callback`: Function to call when message arrives
- `10`: Queue size

**Callback Function:**

```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```

- Automatically called whenever a message arrives
- `msg`: The received message object
- Access message data with `msg.data`

---

## Registering the Nodes

Edit `setup.py` and add the entry points:

```python
from setuptools import find_packages, setup

package_name = 'py_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 Workshop Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = py_example.publisher_node:main',
            'subscriber = py_example.subscriber_node:main',
        ],
    },
)
```

The `entry_points` section tells ROS 2 how to run your nodes.

---

## Package Dependencies

Make sure your `package.xml` includes these dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>py_example</name>
  <version>0.0.1</version>
  <description>ROS 2 Workshop Package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Key Concepts Summary

### Topics

- Named channels for communication
- Many-to-many: multiple publishers and subscribers can use the same topic
- Asynchronous: publisher doesn't wait for subscribers

### Messages

- Data structures sent over topics
- `std_msgs/String` has a single field: `data`
- Other message types: `Int32`, `Float64`, `Bool`, etc.

### Publishers

- Send messages to a topic
- Don't know who (if anyone) is listening

### Subscribers

- Listen to messages on a topic
- Use callback functions to process messages
- Don't know who is publishing

### Nodes

- Independent processes in your robot system
- Each node should do one specific task
- Nodes communicate via topics, services, or actions

---

## Expected Behavior

When you run both nodes:

1. Publisher sends a message every second
2. Subscriber receives and prints each message
3. Messages contain incrementing counter values

The publisher will show:

```text
[INFO] [simple_publisher]: Publisher node started!
[INFO] [simple_publisher]: Publishing: "Hello ROS 2! 0"
[INFO] [simple_publisher]: Publishing: "Hello ROS 2! 1"
[INFO] [simple_publisher]: Publishing: "Hello ROS 2! 2"
```

The subscriber will show:

```text
[INFO] [simple_subscriber]: Subscriber node started!
[INFO] [simple_subscriber]: I heard: "Hello ROS 2! 0"
[INFO] [simple_subscriber]: I heard: "Hello ROS 2! 1"
[INFO] [simple_subscriber]: I heard: "Hello ROS 2! 2"
```

---

## Next Steps

After mastering publisher and subscriber:

- Try using different message types
- Create custom message types
- Learn about services for request-response patterns
- Explore launch files to start multiple nodes together
- Add parameters to configure your nodes

---

## Learning Resources

### Official Documentation

- ROS 2 Docs: <https://docs.ros.org/en/jazzy/>
- ROS 2 Tutorials: <https://docs.ros.org/en/jazzy/Tutorials.html>
