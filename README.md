# 🚁 Fault-Tolerant Motor Control using PX4 and ROS2

This project focuses on **motor control fault tolerance** in drones using the **PX4 Autopilot** and **ROS2 Humble**. The system is capable of handling motor failures in real-time and reallocating control to maintain stable flight. A custom control allocator is implemented to support fault-tolerant behavior.

---

## 📦 Installation Guide

### 1️⃣ Clone the PX4-Autopilot Repository
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive
```

---

### 2️⃣ Setup PX4 Development Environment

Follow the official PX4 setup guide:  
[https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

Make sure you can build and run PX4 successfully.

---

### 3️⃣ Clone PX4 Custom Messages

PX4 uses `px4_msgs` for communication with ROS2. Clone it inside your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

---

### 4️⃣ Replace Control Allocator

For implementing fault-tolerant control:
- Replace the `control_allocator` module in the PX4 source with the **custom `control_allocator` module** provided in this repository.
- Location: `PX4-Autopilot/src/modules/control_allocator`

Make sure to re-build PX4 after replacing the module:
```bash
cd PX4-Autopilot
make px4_sitl gazebo
```

---

### 5️⃣ Setup ROS2 Workspace

Make sure **ROS2 Humble** is installed:  
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 6️⃣ Launch PX4 with Gazebo

Run PX4 in SITL with Gazebo:
```bash
make px4_sitl gazebo
```

In another terminal, start Gazebo with an Iris drone:
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

---

### 7️⃣ Simulate Motor Failure

1. In the PX4 terminal, you can **simulate a motor failure** using the following MAVLink command:
```bash
commander motor_test 1 4 -1 0.5 0
```
*(This will fail motor 4 in the simulation.)*

2. Observe real-time control reallocation and the system's fault-tolerant behavior in the terminal outputs.

---

## 📚 References

- **Fault-Tolerant Control Allocation for Multirotors**
  - Mahony, R., Hamel, T., Pflimlin, J. M. (2008). *Nonlinear Complementary Filters on the Special Orthogonal Group*. IEEE Transactions on Automatic Control.
  
- **NMPC-Based Fault-Tolerant Control**
  - Zhang, Y., Jiang, J. (2008). *Active Fault Tolerant Control System Against Actuator Failures*. IET Control Theory & Applications.

- PX4 Documentation: [https://docs.px4.io/](https://docs.px4.io/)
- ROS2 Documentation: [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)

---

## 🤝 Contributions

Feel free to open issues or submit pull requests for improvements or extensions of this project.

