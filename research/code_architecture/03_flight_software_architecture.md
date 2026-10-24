# 03 — Full Flight Software Architecture

## The complete stack, data flow, and failure handling

---

## Decision status (read this first)

> **Project decision (Robofest 6.0): MAVSDK-Python is the primary flight API, with pymavlink
> as a secondary channel and MAVProxy as the router. MAVROS is rejected, and ROS 2 is not part
> of the core flight stack.** See `research/code_architecture/00_ros2_vs_mavlink.md` for the
> reasons. The ROS 2 node design in this file is kept as the **alternative architecture** (useful if VIO is ever added). The section directly below shows how the same modules map onto the chosen design.

### Chosen design: the same modules as asyncio tasks in one process per drone

| ROS 2 node (alternative below) | Chosen implementation |
|---|---|
| `camera_node` | `camera_task`: picamera2 capture loop, frames pushed to an `asyncio.Queue` |
| `thermal_node` | `thermal_task`: reads the existing C++ `mlx_stdout` pipe via `run_in_executor` |
| `detection_node` | `detection_task`: consumes both queues, heavy CPU work in an executor |
| `map_node` | `map_task`: grid + A* over in-memory NumPy arrays |
| `mission_node` | `mission_task`: the same state machine, written as `async` code |
| `mavlink_bridge_node` | **MAVSDK** (flight commands + telemetry) + pymavlink on a 2nd MAVProxy output (EKF3 health, params) |
| `comms_node` | `comms_task`: asyncio UDP/TCP sockets for swarm traffic |
| ROS 2 topics | `asyncio.Queue` / shared state object |

```python
# Sketch only (untested): the shape of the chosen design.
import asyncio, numpy as np
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

TOL = 0.4  # m, waypoint arrival tolerance

class State:
    pose = (0.0, 0.0, 0.0)          # NED metres, updated by pose_task

async def thermal_task(proc, out_q: asyncio.Queue):
    loop = asyncio.get_running_loop()
    while True:
        # blocking pipe read must not run on the event loop (it would stall setpoints)
        raw = await loop.run_in_executor(None, proc.stdout.read, 768 * 4)
        await out_q.put(np.frombuffer(raw, dtype=np.float32).reshape(24, 32))

async def pose_task(drone: System, state: State):
    async for pv in drone.telemetry.position_velocity_ned():
        state.pose = (pv.position.north_m, pv.position.east_m, pv.position.down_m)

async def goto(drone: System, state: State, n, e, d):
    # ArduCopter holds position if GUIDED targets stop arriving, so keep re-sending
    while (sum((a - b) ** 2 for a, b in zip(state.pose, (n, e, d))) ** 0.5) > TOL:
        await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
        await asyncio.sleep(0.2)

async def mission_task(drone: System, state: State, waypoints):
    for n, e, d in waypoints:       # SCAN state
        await goto(drone, state, n, e, d)
    # AGGREGATE -> GUIDE -> LAND follow the same pattern

async def main(drone, proc, waypoints):
    state, q = State(), asyncio.Queue(maxsize=4)
    await asyncio.gather(
        thermal_task(proc, q), pose_task(drone, state),
        mission_task(drone, state, waypoints),
        # detection_task, map_task, comms_task, ardupilot_aux (pymavlink) ...
    )
```

---

## Overview (ROS 2 alternative design, not chosen)


The software stack spans two processors:

```
┌──────────────────────────────────────────────────────────────────┐
│  RASPBERRY PI 4B / 5                                             │
│  Ubuntu 22.04 ARM64 + ROS 2 Humble                              │
│                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐   │
│  │ camera   │  │ thermal  │  │detection │  │  map_node    │   │
│  │ _node    │  │ _node    │  │ _node    │  │  (grid+A*)   │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────┬───────┘   │
│       │              │              │                │           │
│       └──────────────┴──────────────┘                │           │
│              ROS 2 topics                            │           │
│  ┌───────────────────────────────────┐               │           │
│  │         mission_node              │←──────────────┘           │
│  │  (state machine, orchestration)   │                           │
│  └──────────────────┬────────────────┘                           │
│                     │                                             │
│  ┌──────────────────▼────────────────┐  ┌──────────────────┐   │
│  │      mavlink_bridge_node          │  │  comms_node      │   │
│  │  (pymavlink ↔ ROS 2 translation) │  │  (WiFi UDP swarm)│   │
│  └──────────────────┬────────────────┘  └──────────────────┘   │
│                     │ UART @ 921600 baud                         │
└─────────────────────│────────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────────┐
│  SPEEDYBEE F405 WING V3 — ArduCopter 4.5+                       │
│                                                                  │
│  EKF3: fuses IMU + Optical Flow + Barometer + Compass           │
│  GUIDED mode: accepts position/velocity targets from RPi         │
│  Lua scripts: altitude floor, comms-loss LAND, kill switch       │
│  Outputs: 4× ESC PWM, servo (chalk/LED), buzzer                 │
└──────────────────────────────────────────────────────────────────┘
```

---

## ROS 2 node details (alternative design, not chosen)

### 1. camera_node

**File:** `slave/src/camera_node.py`

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import picamera2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Image, '/drone/camera/raw', 10)
        self.bridge = CvBridge()

        self.cam = picamera2.Picamera2()
        config = self.cam.create_video_configuration(
            main={"size": (640, 480), "format": "BGR888"}
        )
        self.cam.configure(config)
        self.cam.start()

        self.create_timer(1/30.0, self.capture_and_publish)  # 30 Hz

    def capture_and_publish(self):
        frame = self.cam.capture_array()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
```

**Topics published:** `/drone/camera/raw` (Image, 30 Hz)

### 2. detection_node

**File:** `slave/src/detection_node.py`

Subscribes to `/drone/camera/raw`, runs the three-detector pipeline (YOLO + HSV +
Hough), subscribes to `/drone/thermal` for fusion, publishes confirmed mine positions.

```python
from drone_msgs.msg import MineDetection, MineList  # custom messages

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.image_sub = self.create_subscription(
            Image, '/drone/camera/raw', self.on_image, 10)
        self.thermal_sub = self.create_subscription(
            Float32MultiArray, '/drone/thermal', self.on_thermal, 5)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.on_pose, 10)
        self.mine_pub = self.create_publisher(MineList, '/drone/detections', 10)

        self.yolo = load_ncnn_model(PARAM_PATH, BIN_PATH)
        self.latest_thermal = None
        self.latest_pose = None
        self.persistence = PersistenceFilter(confirm_frames=3)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        dets = []
        dets += yolo_detect(self.yolo, frame)
        dets += hsv_detect(frame)
        dets += hough_detect(frame)
        fused = fuse_detections(dets, self.latest_thermal)
        confirmed = self.persistence.update(fused, time.time())

        if confirmed and self.latest_pose:
            world_mines = [project_to_world(d, self.latest_pose)
                           for d in confirmed]
            self.mine_pub.publish(build_mine_list_msg(world_mines))
```

**Topics:** sub `/drone/camera/raw`, `/drone/thermal`, `/drone/pose` → pub `/drone/detections`

### 3. thermal_node

**File:** `slave/src/thermal_node.py`

Reads MLX90640 via I2C at 8 Hz. Publishes raw temperature array.

```python
import board
import busio
import adafruit_mlx90640

class ThermalNode(Node):
    def __init__(self):
        super().__init__('thermal_node')
        i2c = busio.I2C(board.SCL, board.SDA, frequency=1000000)  # 1 MHz I2C
        self.sensor = adafruit_mlx90640.MLX90640(i2c)
        self.sensor.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ
        self.pub = self.create_publisher(Float32MultiArray, '/drone/thermal', 5)
        self.create_timer(0.125, self.read_and_publish)  # 8 Hz

    def read_and_publish(self):
        frame = [0.0] * 768
        self.sensor.getFrame(frame)
        msg = Float32MultiArray()
        msg.data = frame
        self.pub.publish(msg)
```

### 4. map_node

**File:** `slave/src/map_node.py`

Maintains the mine grid. When new mine detections arrive, updates grid and recomputes
A* path. Publishes path as a list of waypoints.

```python
class MapNode(Node):
    def __init__(self):
        self.mine_grid = np.zeros((60, 240), dtype=np.uint8)
        self.mine_sub = self.create_subscription(
            MineList, '/drone/detections', self.on_detections, 10)
        self.swarm_sub = self.create_subscription(
            MineList, '/swarm/mines', self.on_swarm_detections, 10)
        self.path_pub = self.create_publisher(Path, '/drone/safe_path', 5)
        self.map_pub = self.create_publisher(OccupancyGrid, '/drone/map', 1)

    def on_detections(self, msg):
        for mine in msg.mines:
            mark_mine_on_grid(mine.x, mine.y, self.mine_grid)
            # Re-broadcast to swarm
            self.swarm_broadcast(mine)
        self.recompute_path()

    def recompute_path(self):
        start_cell = (0, 30)    # (col=0m, row=7.5m → row=30 at 0.25m/cell)
        goal_cell  = (239, 30)  # (col=60m, row=7.5m)
        path_cells = astar(self.mine_grid, start_cell, goal_cell)
        if path_cells:
            path_msg = cells_to_path_msg(path_cells)
            self.path_pub.publish(path_msg)
```

### 5. mission_node (state machine)

**File:** `slave/src/mission_node.py`

The heart of the system. Drives the drone through mission phases:

```python
from enum import Enum

class MissionState(Enum):
    PREFLIGHT    = "preflight"      # System checks, sensor validation
    TAKEOFF      = "takeoff"        # Arm + climb to 2m
    SCAN         = "scan"           # Execute lawnmower pattern
    AGGREGATE    = "aggregate"      # Wait for swarm mine sharing + path compute
    GUIDE        = "guide"          # Fly along safe path with LEDs active
    ESCORT       = "escort"         # Follow person across field
    LAND         = "land"           # Return to start + land

class MissionNode(Node):
    def __init__(self):
        self.state = MissionState.PREFLIGHT
        self.waypoint_index = 0
        self.scan_waypoints = compute_strip_waypoints(DRONE_ID)
        self.safe_path = None

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.on_pose, 20)
        self.path_sub = self.create_subscription(
            Path, '/drone/safe_path', self.on_path, 5)
        self.gesture_sub = self.create_subscription(
            String, '/drone/gesture', self.on_gesture, 5)

        # Publishers
        self.cmd_pub = self.create_publisher(PoseStamped, '/drone/cmd', 10)
        self.led_pub = self.create_publisher(String, '/drone/leds', 10)

        # State machine timer (10 Hz)
        self.create_timer(0.1, self.tick)

    def tick(self):
        {
            MissionState.PREFLIGHT: self.tick_preflight,
            MissionState.TAKEOFF:   self.tick_takeoff,
            MissionState.SCAN:      self.tick_scan,
            MissionState.AGGREGATE: self.tick_aggregate,
            MissionState.GUIDE:     self.tick_guide,
            MissionState.ESCORT:    self.tick_escort,
            MissionState.LAND:      self.tick_land,
        }[self.state]()

    def tick_scan(self):
        # Send next waypoint when close to current one
        if self.waypoint_index < len(self.scan_waypoints):
            wp = self.scan_waypoints[self.waypoint_index]
            if distance(self.current_pose, wp) < WAYPOINT_TOLERANCE:
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.scan_waypoints):
                    self.state = MissionState.AGGREGATE
                    return
            self.send_waypoint(wp)

    def on_gesture(self, msg):
        if msg.data == 'START' and self.state == MissionState.PREFLIGHT:
            self.state = MissionState.TAKEOFF
        elif msg.data == 'PAUSE':
            self.hold_position()
```

### 6. mavlink_bridge_node

**File:** `slave/src/mavlink_bridge_node.py`

Already shown in `00_ros2_vs_mavlink.md`. Handles all serial communication exclusively.

### 7. comms_node

**File:** `slave/src/comms_node.py`

Handles WiFi UDP inter-drone communication.

```python
class CommsNode(Node):
    BROADCAST_IP = '192.168.99.255'
    PORT = 5005

    def __init__(self):
        self.drone_id = int(os.environ.get('DRONE_ID', 1))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(('', self.PORT))
        self.recv_sock.setblocking(False)

        # Forward received swarm mines to map_node
        self.swarm_mine_pub = self.create_publisher(MineList, '/swarm/mines', 10)
        # Subscribe to local mine detections for broadcasting
        self.mine_sub = self.create_subscription(
            MineList, '/drone/detections', self.broadcast_mines, 10)
        self.create_timer(0.1, self.receive_messages)  # 10 Hz receive loop

    def broadcast_mines(self, msg):
        packet = {'type': 'MINES', 'drone_id': self.drone_id,
                  'mines': [(m.x, m.y, m.confidence) for m in msg.mines]}
        self.sock.sendto(json.dumps(packet).encode(), (self.BROADCAST_IP, self.PORT))
```

---

## Failure handling and watchdogs

### Comms loss watchdog (Lua on SpeedyBee)

ArduPilot Lua runs even if the RPi crashes. This script implements LAND on comms loss (not RTL: RTL needs a trustworthy home/position, which a GPS-denied drone does not have):

```lua
-- scripts/comms_watchdog.lua
local last_heartbeat = millis()
local HEARTBEAT_TIMEOUT_MS = 5000  -- 5 seconds

local function update()
    -- Check if companion computer is sending heartbeats
    local now = millis()
    if now - last_heartbeat > HEARTBEAT_TIMEOUT_MS then
        gcs:send_text(4, "Companion comms lost — LAND")
        vehicle:set_mode(9)   -- 9 = LAND on ArduCopter
    end
    return update, 500  -- check every 500 ms
end

-- Listen for heartbeat from companion via MAVLink
function heartbeat_received()
    last_heartbeat = millis()
end

return update()
```

> **Caution:** this script is a sketch. `heartbeat_received()` is never connected to actual MAVLink
> reception, so as written `last_heartbeat` is never updated and the script would trigger LAND
> about 5 seconds after start. Wire it to the AP_Scripting MAVLink receive API, decide which
> message/component counts as the companion heartbeat (MAVSDK's server also sends heartbeats),
> and test in SITL before relying on it.

The RPi sends a heartbeat every 1 second via pymavlink:
```python
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0
)
```

### Altitude floor (Lua)

> Untested sketch: confirm that the Lua binding used below exists for your ArduCopter version and behaves as intended in SITL.
> GPS-denied review (2026-10-24): `ahrs:get_position()` returns a global (lat/lon) location that does not exist without GPS/an EKF origin. Use the downward rangefinder (height above ground) instead.

Prevents the drone from descending below 1.0 m (safety margin above mines/person):

```lua
-- scripts/altitude_floor.lua
local FLOOR_M = 1.0

local function update()
    local pos = ahrs:get_position()
    if pos and pos:alt() < FLOOR_M * 100 then  -- alt in cm
        -- Override with altitude hold at floor
        vehicle:set_target_altitude_cm(FLOOR_M * 100)
    end
    return update, 100  -- 10 Hz check
end
return update()
```

---

## Data flow diagram

```
[Person gesture] ──→ [gesture_node] ──→ /drone/gesture ──→ [mission_node]
                                                                   │
[RPi Camera] ──→ [camera_node] ──→ /drone/camera/raw            TAKEOFF ──→ [mavlink_bridge]
                                              │                    │                │
[MLX90640] ──→ [thermal_node] ──→ /drone/thermal                 SCAN               │
                                              │                    │                 ▼
                              [detection_node]◄──────────────────  │         SpeedyBee F405
                                    │ /drone/detections            │         (ArduCopter)
                                    ▼                              │              │
                              [map_node] ──→ /drone/map           │         LOCAL_POS_NED
                                    │ /drone/safe_path            │              │
                                    ▼                              │              ▼
                              [mission_node] ◄────────────── /drone/pose ◄── [mavlink_bridge]
                                    │ /drone/cmd
                                    ▼
                              [mavlink_bridge] ──→ UART ──→ SpeedyBee

[WiFi] ◄──→ [comms_node] ◄──→ /swarm/mines (bidirectional)
                    └──────────→ [map_node] (receives other drones' mine detections)
```

---

## References

1. ArduPilot companion computer documentation:
   https://ardupilot.org/dev/docs/companion-computers.html

2. ArduPilot Lua scripting:
   https://ardupilot.org/plane/docs/common-lua-scripts.html

3. ROS 2 publisher/subscriber tutorial:
   https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

4. picamera2 Python library:
   https://github.com/raspberrypi/picamera2

5. AP_DDS ROS 2 native interface (ArduPilot 4.5+):
   https://ardupilot.org/dev/docs/ros2.html
