# 00 — ROS 2 vs MAVLink vs pymavlink

## When to use each, and why

---

## The three layers

There are three distinct communication layers in our system:

```
┌─────────────────────────────────────────────────────┐
│  Application Layer (mission logic, detection, map)  │
│  Language: Python 3                                 │
│  Framework: ROS 2 (Humble or Jazzy)                 │
└─────────────────────┬───────────────────────────────┘
                      │ ROS 2 topics/services
┌─────────────────────▼───────────────────────────────┐
│  MAVLink Bridge                                     │
│  Options: pymavlink | MAVROS 2 | AP_DDS             │
└─────────────────────┬───────────────────────────────┘
                      │ MAVLink over UART/USB
┌─────────────────────▼───────────────────────────────┐
│  ArduCopter on SpeedyBee F405                       │
│  Flight control, sensor fusion, motor control       │
└─────────────────────────────────────────────────────┘
```

Each layer has a different job. Mixing their responsibilities causes bugs.

---

## Layer 1 — ROS 2

### What ROS 2 is (plain English)

ROS 2 (Robot Operating System 2) is a middleware framework for robotics. It provides:
1. **Topics:** publish/subscribe message passing between processes
2. **Services:** request/response calls between processes
3. **Actions:** long-running tasks with feedback (like "execute this mission")
4. **Parameter server:** shared configuration
5. **Bags:** recording and replaying all messages for debugging

Think of it as the "nervous system" of the robot — a standard way for different
software modules to talk to each other, even if they're in different languages or
running on different computers.

### When to use ROS 2 in our system

Use ROS 2 for data that flows between **independent software modules**:

| Data flow | ROS 2 topic | Rate |
|---|---|---|
| Raw camera frames | `/drone/camera/image_raw` | 30 Hz |
| Detected mine positions | `/drone/detections/mines` | As detected |
| Mine grid map | `/drone/map/grid` | 1 Hz |
| Drone pose from ArduPilot | `/drone/pose/local_ned` | 10–50 Hz |
| Mission state machine | `/drone/mission/state` | 1 Hz |
| Inter-drone mine reports | `/swarm/mines` | As detected |
| LED control commands | `/drone/leds/command` | As needed |

The key property: ROS 2 topics decouple the producer (camera node) from the consumer
(detection node). You can record all topics to a bag file and replay them offline for
debugging without a physical drone.

### When NOT to use ROS 2

Do not use ROS 2 for:
- **Flight control commands to ArduPilot** — use MAVLink directly for this
- **Emergency kill switch** — latency-critical, must be on hardware failsafe, not software
- **Sensor I2C/SPI reading** — handle in a ROS 2 node, but the actual hardware read
  is a system call, not a ROS 2 call

### ROS 2 version for us

**ROS 2 Jazzy (2024, LTS)** on Ubuntu 24.04 (or ROS 2 Humble on Ubuntu 22.04).
Raspberry Pi 4B: Jazzy is supported on ARM64. Install via official ROS 2 debs.

---

## Layer 2 — MAVLink

### What MAVLink is

MAVLink (Micro Air Vehicle Link) is a binary communication protocol used by ArduPilot
(and PX4) to send and receive flight control data. It's the language your RPi speaks
to the SpeedyBee F405.

MAVLink is NOT a general-purpose ROS 2 protocol. It's specific to flight controllers.

### Key MAVLink message types we use

| Message | Direction | Purpose |
|---|---|---|
| `SET_POSITION_TARGET_LOCAL_NED` | RPi → FC | Send target position/velocity in GUIDED mode |
| `COMMAND_LONG` | RPi → FC | Arm motors, change mode, takeoff |
| `LOCAL_POSITION_NED` | FC → RPi | Drone's current estimated position (x, y, z, vx, vy, vz) |
| `ATTITUDE` | FC → RPi | Roll, pitch, yaw angles |
| `SYS_STATUS` | FC → RPi | Battery voltage, CPU load |
| `EKF_STATUS_REPORT` | FC → RPi | EKF health, position/velocity variance |
| `OPTICAL_FLOW` | FC → RPi | Optical flow sensor readings |
| `HEARTBEAT` | Both | Alive signal — if missing for >1s, failsafe triggers |

### The three MAVLink interface options

**Option 1: pymavlink (current approach)**

Direct Python library. Most flexible, lowest level:

```python
from pymavlink import mavutil

# Connect to ArduPilot via serial
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
master.wait_heartbeat()

# Send GUIDED mode
master.set_mode('GUIDED')

# Send position target
master.mav.set_position_target_local_ned_send(
    0, master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111111000,   # position only
    x, y, -2.0, 0, 0, 0, 0, 0, 0, 0, 0
)
```

When to use pymavlink:
- When you want fine-grained control over MAVLink messages
- When ROS 2 overhead is unacceptable (latency < 10 ms needed)
- For simple scripts and testing
- When you don't need topic-based pub/sub from multiple consumers

**Option 2: MAVROS 2**

A ROS 2 package that bridges MAVLink to ROS 2 topics. MAVROS publishes ArduPilot
telemetry as standard ROS 2 topics:

```python
# ROS 2 + MAVROS: drone pose comes from ROS topic
import rclpy
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('mission_node')

pose_sub = node.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                     on_pose_update, 10)

# Send position command via MAVROS
pose_pub = node.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
```

When to use MAVROS 2:
- When your mission logic is already in ROS 2 and needs to access flight state
- When you want multiple ROS 2 nodes to consume drone telemetry (map node, detection
  node, logging node — all subscribe to the same topic without competing for serial port)

**Option 3: AP_DDS (native ROS 2 from ArduPilot)**

ArduPilot 4.5+ has built-in Micro XRCE-DDS support (library: AP_DDS). This allows
ArduPilot to publish/subscribe ROS 2 topics **without** any bridge software on the RPi.
ArduPilot IS a native ROS 2 node.

Published topics include:
- `/ap/pose/filtered` — PoseStamped with EKF-filtered position
- `/ap/navsat` — NavSatFix (GPS position, if GPS is used)
- `/ap/imu/experimental` — IMU data

Subscribed topics:
- `/ap/cmd_vel` — TwistStamped for velocity control
- `/ap/joy` — Joystick for manual override

Services: `/ap/arm_motors`, `/ap/mode_switch`, `/ap/takeoff`

Connection: Serial UART or Ethernet (ArduPilot 4.5+ supports Ethernet to RPi).

```bash
# On RPi: start the XRCE-DDS agent (bridges DDS to ROS 2)
microxrceddsagent serial --dev /dev/ttyAMA0 -b 921600
```

When to use AP_DDS:
- For newest SpeedyBee F405 firmware (check if AP_DDS is compiled in for F405)
- When you want the cleanest ROS 2 architecture with no extra bridge layer
- When team is comfortable with ROS 2 services for arming/mode switching

---

## Layer 3 — ArduPilot Lua scripting

### What it is

ArduPilot supports onboard Lua scripting (AP_Scripting). Scripts run directly on the
flight controller CPU — no companion computer needed for simple automation tasks.

Lua scripts can:
- Read sensor values (GPS, baro, IMU, rangefinder)
- Execute conditional flight logic (if altitude < 1.5m: add hover hold)
- Send MAVLink messages to companion computer
- Trigger servo outputs (for LED control, chalk dropping)

### When to use Lua scripts vs RPi

| Task | Lua (FC onboard) | RPi (companion) |
|---|---|---|
| Low-level failsafe (altitude hold) | ✓ Better (no serial latency) | ✗ |
| Complex mission logic | ✗ Limited compute | ✓ Better |
| Mine detection | ✗ No camera | ✓ |
| LED/servo for path marking | ✓ Simple, reliable | ✓ (more flexibility) |
| Watchdog for comms loss | ✓ Runs even if RPi crashes | ✗ |

**Recommended split:** Use Lua for safety functions (altitude floor, comms-loss RTL).
Use RPi for everything else.

---

## Recommended integration for Robofest 6.0

```
RPi 4B running Ubuntu 22.04 + ROS 2 Humble
│
├── camera_node.py       → Reads RPi Cam → publishes /drone/camera/image_raw
├── detection_node.py    → Subscribes image, runs YOLO → publishes /drone/detections
├── thermal_node.py      → Reads MLX90640 via I2C → publishes /drone/thermal
├── map_node.py          → Subscribes detections → maintains mine grid → A* path
├── mission_node.py      → State machine: SCAN → AGGREGATE → GUIDE
└── mavlink_bridge.py    → pymavlink bridge:
                            Subscribes /drone/mission/cmd → sends MAVLink GUIDED cmds
                            Publishes LOCAL_POSITION_NED → /drone/pose
```

The mavlink_bridge.py handles the serial port exclusively. All other nodes communicate
via ROS 2 topics. No other node touches the serial port.

```python
# mavlink_bridge.py — simplified
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil

class MAVLinkBridge(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')
        self.master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.cmd_sub = self.create_subscription(PoseStamped, '/drone/cmd', self.on_cmd, 10)
        self.create_timer(0.02, self.read_mavlink)  # 50 Hz read loop

    def read_mavlink(self):
        msg = self.master.recv_match(blocking=False)
        if msg and msg.get_type() == 'LOCAL_POSITION_NED':
            pose = PoseStamped()
            pose.pose.position.x = msg.x
            pose.pose.position.y = msg.y
            pose.pose.position.z = -msg.z  # flip NED z to altitude
            self.pose_pub.publish(pose)

    def on_cmd(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, x, y, -z, 0, 0, 0, 0, 0, 0, 0, 0
        )
```

---

## References

1. ArduPilot native ROS 2 (AP_DDS) documentation:
   https://ardupilot.org/dev/docs/ros2.html

2. ArduPilot ROS 2 on Raspberry Pi guide:
   https://ardupilot.org/dev/docs/ros2-pi.html

3. ArduROSPi: ROS 2 ↔ pymavlink bridge example:
   https://discuss.ardupilot.org/t/ardurospi-ros-2-pymavlink-bridge/142058

4. MAVROS 2 GitHub:
   https://github.com/mavlink/mavros

5. AP_DDS XRCE-DDS guide (Raspberry Pi 4):
   https://discuss.ardupilot.org/t/guide-for-using-dds-over-ppp-with-a-raspberry-pi-4-companion-computer/126128
