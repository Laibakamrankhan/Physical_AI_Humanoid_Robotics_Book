# Module 1 Contract: The Robotic Nervous System (ROS 2)

**Module**: Module 1 | **Chapter Title**: The Robotic Nervous System (ROS 2)
**Focus**: Middleware for robot control
**Dependencies**: Foundational knowledge of Python programming.

## Section Breakdowns

### 1.1 Introduction to ROS 2
-   **1.1.1 What is ROS 2?**: Overview, history, key features, and advantages for robotics.
    -   *Content Contract*: Explain ROS 2 as a flexible framework for writing robot software. Discuss its evolution from ROS 1 and its benefits like real-time capabilities and security.
    -   *Required Diagram*: Diagram illustrating the high-level ROS 2 architecture.
-   **1.1.2 ROS 2 Concepts: Nodes, Topics, Services, Actions**: Detailed explanation of core communication mechanisms.
    -   *Content Contract*: Define each concept with clear analogies and examples. Emphasize their roles in distributed robot systems.
    -   *Required Diagram*: Diagram showing interaction between nodes, topics, services, and actions.
-   **1.1.3 ROS 2 vs. ROS 1**: Highlight key differences and improvements.
    -   *Content Contract*: Table comparing ROS 1 and ROS 2 features (DDS, security, real-time, multi-robot).

### 1.2 Setting Up Your ROS 2 Environment
-   **1.2.1 Installation Guide**: Step-by-step instructions for installing ROS 2 (e.g., Foxy/Galactic on Ubuntu).
    -   *Content Contract*: Provide commands and verification steps.
    -   *Required Checklist*: Installation verification checklist.
-   **1.2.2 Workspace and Package Creation**: How to set up a ROS 2 workspace and create a new Python package.
    -   *Content Contract*: Explain `colcon` build system. Step-by-step for `ros2 pkg create --build-type ament_python` command.
    -   *Required Code Snippet*: Basic Python ROS 2 package structure.

### 1.3 ROS 2 Communication Patterns in Python
-   **1.3.1 Publishers and Subscribers**: Implementing one-way data flow.
    -   *Content Contract*: Explain `rclpy.create_publisher` and `rclpy.create_subscription`.
    -   *Required Code Snippet*: Sample ROS 2 node (Talker/Listener) in Python (from spec).
    -   *Required Exercise*: Create a custom message type and use it in a pub/sub.
-   **1.3.2 Services and Clients**: Request-response communication.
    -   *Content Contract*: Explain `rclpy.create_service` and `rclpy.create_client`.
    -   *Required Code Snippet*: Simple Python ROS 2 service and client.
-   **1.3.3 Actions: Long-Running Tasks**: Implementing goal-feedback-result communication.
    -   *Content Contract*: Explain `rclpy.action.client.Client` and `rclpy.action.server.Server`.
    -   *Required Code Snippet*: Basic Python ROS 2 action client and server.

### 1.4 Describing Your Humanoid Robot (URDF)
-   **1.4.1 Introduction to URDF**: Unified Robot Description Format for robot modeling.
    -   *Content Contract*: Explain `link` and `joint` elements, kinematic chains.
    -   *Required Diagram*: Simple robot arm URDF model.
-   **1.4.2 Building a Humanoid URDF**: Step-by-step creation of a simple humanoid robot model.
    -   *Content Contract*: Explain how to define `origin`, `axis`, `limit` for joints.
    -   *Required Code Snippet*: Sample URDF snippet for a humanoid joint (from spec).
    -   *Required Exercise*: Extend a basic URDF with additional joints and links.

### 1.5 Launching and Debugging ROS 2 Systems
-   **1.5.1 Launch Files**: Orchestrating multiple ROS 2 nodes.
    -   *Content Contract*: Explain `ros2 launch` and Python launch files.
    -   *Required Code Snippet*: Simple Python ROS 2 launch file to start multiple nodes.
-   **1.5.2 Parameters**: Configuring ROS 2 nodes at runtime.
    -   *Content Contract*: Explain `ros2 param` and declaring parameters in nodes.
    -   *Required Code Snippet*: Python node demonstrating parameter usage.
-   **1.5.3 Debugging Tools**: `rqt_graph`, `ros2 topic echo`, `ros2 node info`.
    -   *Content Contract*: Practical usage of ROS 2 debugging tools.
    -   *Required Sidebar*: Common ROS 2 errors and troubleshooting tips.

### 1.6 Bridging Python Agents to ROS Controllers
-   **1.6.1 Agent-Controller Interaction**: Concepts of high-level Python AI agents interacting with low-level ROS 2 robot controllers.
    -   *Content Contract*: Discuss design patterns for integrating AI logic with ROS 2.
-   **1.6.2 Example Integration Workflow**: A simple example of an AI agent sending commands to a ROS 2 controller.
    -   *Content Contract*: Conceptual overview of the data flow and command execution.
    -   *Required Exercise*: Implement a simple Python agent that publishes velocity commands to a ROS 2 topic.

## Cross-Module References

-   **Module 2 (Digital Twin)**: URDF models created here will be used in Gazebo simulations. ROS 2 communication will be vital for controlling simulated robots.
-   **Module 3 (AI-Robot Brain)**: The ROS 2 foundation will be essential for integrating NVIDIA Isaac ROS and Nav2.
-   **Module 4 (VLA)**: ROS 2 actions will be the target for cognitive planning and voice commands.

## Required Assets

-   Diagrams: ROS 2 Architecture, Node/Topic/Service/Action Interaction, Simple Robot Arm URDF.
-   Tables: ROS 1 vs. ROS 2 comparison.
-   Code Snippets (Python, XML): ROS 2 Node (Pub/Sub), Custom Message Type, Service/Client, Action Client/Server, Basic URDF, Launch File, Parameter Usage, Python Agent to ROS Bridge.
-   Checklists: ROS 2 Installation Verification.
-   Exercises: Pub/Sub with custom message, Extend URDF, Python Agent velocity commands.
-   Sidebars: Common ROS 2 errors and troubleshooting.