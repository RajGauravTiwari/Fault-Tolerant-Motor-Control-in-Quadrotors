Install acaodos and acados python interface for running these interfaces.


# Running a ROS 2 Node for Drone Control with MAVSDK

This project explains how to set up and run a ROS 2 node to control a drone. The node includes steps to arm the drone, disable the control allocator to prevent interference, and then use the provided **nmp_controller** code to manage the drone's behavior.

---

## Prerequisites
- **ROS 2 Humble** (or compatible version) installed on your system.
- **MAVSDK** for Python installed.
- A drone simulator (e.g., PX4 SITL) or hardware drone connected via MAVSDK.
- The `nmp_controller` package properly built and sourced in your ROS 2 workspace.

---

## Installation

1. **Install Required ROS 2 Packages**:
   Ensure you have the necessary ROS 2 packages for building and running the node:
   ```bash
   sudo apt update
   sudo apt install ros-humble-mavros ros-humble-mavros-extras
   ```

2. **Set Up MAVSDK**:
   Install the MAVSDK Python bindings:
   ```bash
   pip install mavsdk
   ```

3. **Clone and Build the ROS 2 Workspace**:
   - Clone the repository containing the `nmp_controller` package:
     ```bash
     git clone <repository-url> ~/ros2_ws/src
     ```
   - Build the workspace:
     ```bash
     cd ~/ros2_ws
     colcon build
     ```
   - Source the workspace:
     ```bash
     source install/setup.bash
     ```

---

## Usage

1. **Start the ROS 2 Node**:
   Run the ROS 2 node for controlling the drone:
   ```bash
   ros2 run nmp_controller main
   ```

2. **Arm the Drone**:
   Ensure the drone is armed to enable control. You can arm it via MAVSDK or directly using a ROS 2 service:
   ```bash
   commander arm
   ```

3. **Disable the Control Allocator**:
   To prevent interference, disable the control allocator before running the code. This ensures the provided control logic directly manages the drone. Use the following command to stop the control allocator:
   ```bash
   ros2 service call /stop_allocator std_srvs/srv/Trigger
   ```

4. **Run the Code**:
   After the drone is armed and the control allocator is stopped, execute the control logic by running:
   ```bash
   ros2 run nmp_controller main
   ```

---

## Script Overview

1. **Starting the Node**:
   - Launches the ROS 2 node to interface with the drone and manage its behavior.

2. **Arming the Drone**:
   - Ensures the drone is ready for commands by sending an arm signal.

3. **Stopping the Control Allocator**:
   - Prevents PX4's internal control mechanisms from interfering with custom logic.

4. **Executing Custom Control**:
   - Uses the **nmp_controller** package to apply the control logic.

---

## Notes
- Ensure that the PX4 SITL or hardware drone is connected and accessible via ROS 2 and MAVSDK.
- Use a safe environment for testing drone controls to prevent accidents or damage.
- Verify that the `nmp_controller` package is configured correctly for your application.

---

## License
Include any licensing information here.

---
