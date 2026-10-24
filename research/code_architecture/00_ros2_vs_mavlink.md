# 00 — ROS 2 vs MAVSDK vs MAVLink (pymavlink / MAVROS)

## When to use each, and why

---

## Decision status (read this first)

> **Chosen for Robofest 6.0: MAVSDK-Python as the primary flight API, pymavlink as a
> secondary channel for ArduPilot-specific gaps, MAVProxy as the MAVLink router.
> MAVROS is rejected. ROS 2 is optional and not part of the core flight stack.**

This file was first written as a general survey and originally assumed a ROS 2 node graph
with a pymavlink bridge. That layout is kept below as the **alternative architecture**, because
the reasoning is still useful if the project ever adds VIO (which is ROS-heavy). The sections
marked *(chosen)*, *(secondary)*, *(rejected)* and *(not chosen)* show where each option landed.

### Why MAVSDK is the primary API here

1. **It is async-native, and the whole codebase is asyncio.** MAVSDK-Python exposes
   `await drone.action.arm()`, `async for pos in drone.telemetry.position()`, and so on. That
   drops straight into the asyncio architecture (thermal reader, detection, comms, mission
   state machine all as coroutines). pymavlink is blocking and callback/poll based; ROS 2
   brings its own executor and callback model that has to be bridged to asyncio with threads.
2. **Small memory footprint on a Pi 4 (4 GB).** MAVSDK costs roughly 60-80 MB of RAM
   (a thin Python wrapper plus the `mavsdk_server` process). MAVROS costs roughly 550-750 MB,
   plus the ROS 2 runtime, on a board that is also running YOLO/NCNN inference, thermal
   processing, Flask and Vosk.
3. **The frame convention matches our code.** Our local coordinates are NED, the same as
   ArduPilot's local frame and MAVSDK's `PositionNedYaw` offboard setpoints. MAVROS publishes
   ENU following the ROS convention (REP-103), which means silent X/Y swaps and Z sign flips at
   every boundary. In a GPS-denied system where mine positions depend on getting the frame right,
   a hidden conversion layer is a bug source we do not want.
4. **Small API surface, fast to rewrite around.** The team is small and a full codebase rewrite
   is planned. `arm`, `offboard`/GUIDED setpoints, telemetry streams and `land` cover most of
   what flight code needs, with no workspace, colcon build or launch-file layer to maintain.
5. **MAVROS fit with ArduPilot/EKF3 is weaker than with PX4.** MAVROS support for the
   ArduPilot + EKF3 + optical-flow path has gaps, and debugging them through a ROS bridge adds a
   layer between us and the flight controller.

### Why pymavlink stays as a secondary channel

MAVSDK is designed around PX4 and covers only part of ArduPilot. Things MAVSDK does not
expose well on ArduPilot, such as EKF3 status/health (`EKF_STATUS_REPORT`) and some parameter
and message-level work, go through pymavlink on a second MAVProxy UDP output. MAVProxy routes
the same MAVLink stream to both. This is a gap-filler, not a second flight controller: only
MAVSDK sends flight commands.

### Why ROS 2 is not in the core stack

ROS 2's strengths (topic decoupling, rosbag replay, ready-made packages) matter most with many
independent nodes or with ROS-native perception stacks. For a small team on a Pi 4 with one
asyncio process per drone, the cost (RAM, build/launch overhead, the asyncio bridge) outweighs
the benefit. It becomes worth revisiting only if VIO (OpenVINS/VINS) is added, since those
stacks are ROS-native; in that case ROS 2 would sit beside the flight code with a one-way
bridge (see the Alternative section at the bottom), not replace MAVSDK.

### Quick comparison

| | MAVSDK-Python | pymavlink | MAVROS 2 | AP_DDS / ROS 2 |
|---|---|---|---|---|
| Role here | **Primary (chosen)** | Secondary | Rejected | Not chosen |
| asyncio fit | Native | Blocking, needs executor | Needs thread bridging | Needs thread bridging |
| RAM on Pi 4 | ~60-80 MB | Negligible | ~550-750 MB | ROS 2 runtime + DDS agent |
| Frame | NED (matches ArduPilot) | NED (raw MAVLink) | ENU (conversion needed) | ROS REP-103 conventions |
| ArduPilot coverage | Partial (PX4-centric) | Complete (raw messages) | Gaps with EKF3/flow | Newer, firmware-dependent |
| Best for | Arm, offboard/GUIDED, telemetry, land | EKF health, params, custom messages | ROS-native pipelines | Pure-ROS 2 stacks |

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

### The four MAVLink interface options

**Option 1: pymavlink (secondary: gap filler)**

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

**Option 2: MAVSDK-Python (chosen: primary flight API)**

An async Python wrapper over a `mavsdk_server` process that speaks MAVLink to the flight
controller. On ArduPilot, MAVSDK's `offboard` plugin sends position/velocity setpoints that
ArduPilot handles in GUIDED mode.

```python
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

async def fly():
    drone = System()
    await drone.connect(system_address="udp://:14540")   # MAVProxy output

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # Follow the project's arm-and-offboard sequence (see technical-learnings):
    # validate EKF health, send a zero-offset hold setpoint, then arm and start offboard,
    # so the NED origin is not reset by a mode transition.
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.action.arm()
    await drone.offboard.start()
    await asyncio.sleep(2.0)                               # stabilisation

    # NED: x = North, y = East, z = Down (so -2.0 = 2 m up)
    await drone.offboard.set_position_ned(PositionNedYaw(10.0, 0.0, -2.0, 0.0))

asyncio.run(fly())
```

When to use MAVSDK (this is our default):
- Arming, mode/offboard control, position and velocity setpoints, landing
- Telemetry streams (position, attitude, battery, in-air state) consumed as async generators
- Anywhere the code is already asyncio

Limits to know about:
- Built around PX4, so some ArduPilot behaviours need workarounds (for example, the NED origin
  can reset on an AUTO to GUIDED transition; avoid `action.takeoff()` and use the hold-setpoint
  sequence above)
- EKF3 health and parameter-level work are not covered well: use pymavlink for those
- Setpoints must keep streaming; ArduCopter holds position if GUIDED targets stop arriving

**Option 3: MAVROS 2 (rejected for this project)**

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

When MAVROS 2 would make sense (not our case; see Decision status for why we rejected it):
- When your mission logic is already in ROS 2 and needs to access flight state
- When you want multiple ROS 2 nodes to consume drone telemetry (map node, detection
  node, logging node — all subscribe to the same topic without competing for serial port)

**Option 4: AP_DDS (native ROS 2 from ArduPilot, not chosen)**

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

## Chosen architecture for Robofest 6.0 (MAVSDK-first, single asyncio process)

```
RPi 4B (4 GB), one asyncio process per drone
│
├── thermal_task      → MLX90640 reader (blocking pipe read via run_in_executor) → asyncio.Queue
├── detection_task    → RGB + thermal detectors, confidence fusion, persistence filter
├── map_task          → local NED grid, mine list, exclusion zones
├── mission_task      → state machine: SCAN -> AGGREGATE -> GUIDE
├── comms_task        → inter-drone UDP/TCP (WiFi), heartbeats, mine reports
├── flight (MAVSDK)   → arm, offboard setpoints, telemetry streams, land   [only sender of flight commands]
└── ardupilot_aux     → pymavlink on 2nd MAVProxy UDP output: EKF3 health, params, extra messages [read/aux only]

SpeedyBee F405 (ArduCopter) <-- serial --> MAVProxy (router) --> UDP :14540 MAVSDK
                                                           --> UDP :14550-style out for pymavlink
```

Rules that keep this stable:
- Blocking I/O and heavy numpy work go through `run_in_executor` and `asyncio.Queue`, so the
  event loop (and therefore the GUIDED setpoint stream) never stalls.
- Exactly one component sends flight commands (MAVSDK). pymavlink is used for reads and for
  parameter or message gaps.
- Safety functions that must survive a companion-computer crash (altitude floor, comms-loss
  RTL) still belong in ArduPilot Lua or parameters, as described in Layer 3.

---

## Alternative: ROS 2-based integration (not chosen)

The layout below was the original recommendation of this file. It is retained for reference and
for a possible future VIO add-on. If ROS 2 is ever added, keep MAVSDK as the flight path and use
ROS 2 only for perception data, with a one-way bridge, rather than routing flight control
through ROS.


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

6. MAVSDK-Python documentation (primary flight API):
   https://mavsdk.mavlink.io/main/en/python/

7. ArduPilot GUIDED mode (what MAVSDK offboard maps to on ArduCopter):
   https://ardupilot.org/copter/docs/ac2_guidedmode.html
