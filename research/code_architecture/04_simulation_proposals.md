# 04 — Simulation and Testing Proposals

## How to test before flying

---

## Why simulation matters for this PS

You have 3 drones, each ≤750 g. A crash costs:
- Competition score: -20 points per crash
- Hardware: ₹8,000–15,000 per drone replacement
- Time: 1–2 days to rebuild and re-test

A solid simulation workflow means discovering and fixing bugs in software before
they become crashes in hardware.

The goal is to test each subsystem in increasing complexity:

```
Level 1: Unit tests on laptop (no drone, no simulator)
Level 2: Detection pipeline on recorded footage
Level 3: ArduPilot SITL (simulated flight controller)
Level 4: SITL + Gazebo (simulated physics + simulated field)
Level 5: HITL (hardware flight controller in the loop)
Level 6: Indoor hover test (real drone, controlled environment)
Level 7: Outdoor field test (real field, real conditions)
```

Do not skip levels. Each level catches a different class of bug.

---

## Proposal A: ArduPilot SITL + Gazebo Simulation

### What SITL is

Software-In-The-Loop (SITL) runs ArduCopter entirely as a Linux process on your
laptop. The flight controller firmware runs in software, talking to simulated sensors.

SITL speaks the same MAVLink protocol as real hardware. Your RPi code cannot tell
the difference between SITL and a real SpeedyBee — they speak the same messages.

### Setup

```bash
# Install ArduPilot and SITL dependencies
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build SITL for ArduCopter
cd ArduCopter
sim_vehicle.py -v ArduCopter --console --map

# This starts:
# - ArduCopter SITL process (UART on TCP port 5760)
# - MAVProxy ground station GUI
# - Map display showing simulated drone position
```

### Connect your RPi code to SITL

Your RPi mission code connects to SITL's TCP port instead of a serial port:

```python
# Instead of:
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

# Use SITL:
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
```

Everything else in your code runs unchanged. You can test:
- Mode switching (GUIDED, LAND, RTL)
- Arming sequence
- Takeoff to 2.0 m
- Position target commands (does the simulated drone fly to waypoints?)
- State machine transitions
- EKF failure injection (parameter override)

### SITL + Gazebo: simulated physics

Gazebo adds a 3D physics simulation with realistic aerodynamics, inertia, and
sensor noise.

```bash
# Install Gazebo + ArduPilot plugin
sudo apt-get install gazebo ros-humble-gazebo-ros-pkgs
# Clone the ArduPilot Gazebo plugin
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo && mkdir build && cd build && cmake .. && make

# Launch Gazebo with ArduCopter model
gazebo --verbose worlds/iris_ardupilot.world &
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

### Custom mine field in Gazebo

Create a SDF (Simulation Description Format) model for the mine field:

```xml
<!-- mine_field.world -->
<world name="mine_field">
  <include><uri>model://ground_plane</uri></include>
  <include><uri>model://sun</uri></include>

  <!-- Field boundary markers -->
  <model name="field_boundary">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>60 15 0.01</size></box></geometry>
        <material>
          <ambient>0.4 0.7 0.4 1</ambient>  <!-- green grass -->
        </material>
      </visual>
    </link>
  </model>

  <!-- Mine disc (repeat 40 times with varied positions/colours) -->
  <model name="mine_01">
    <pose>5.3 3.1 0.005 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.125</radius><length>0.01</length></geometry>
        <material><ambient>1 0 0 1</ambient></material>  <!-- red mine -->
      </visual>
    </link>
  </model>
  <!-- ... repeat for all 40 mines -->
</world>
```

### What Gazebo simulation lets you test

- **Coverage completeness:** After simulated scan, which mine positions were detected?
  Count detected vs actual → detection completeness metric.
- **Navigation drift:** How far does the simulated drone drift from its commanded
  position over a 10-minute mission?
- **Collision avoidance:** Run all 3 simulated drones simultaneously and check if ORCA
  or altitude separation prevents collisions.
- **Safe path generation:** Given simulated mine map, does A* generate a valid 1 m
  clearance path?
- **Time budget:** Does the full mission complete within 10 minutes in simulation?

### Limitations

- Gazebo camera images won't look like a real field (no realistic grass texture)
- Can't test real YOLOv8 detection on simulated images (colours are wrong)
- Wind simulation is simplified vs real outdoor wind
- Optical flow sensor behaviour in Gazebo is approximate

---

## Proposal B: Hardware-In-The-Loop (HITL) with SpeedyBee

### What HITL is

HITL runs the real ArduCopter firmware on the real SpeedyBee F405 board, but with
simulated sensor inputs from Gazebo (via USB). The physical ESCs and motors are NOT
powered. The FC processes simulated IMU, GPS, barometer values and outputs PWM signals
that go to the simulation instead of real motors.

### Why HITL over SITL

- Tests the actual firmware binary on the actual hardware
- Catches firmware-specific bugs (specific to SpeedyBee's hardware abstraction)
- Tests the actual UART serial link between RPi and SpeedyBee

### Setup

```
Gazebo (laptop) ──USB──→ SpeedyBee F405 (HITL firmware) ──UART──→ RPi 4B
                          (real flight controller, simulated sensors)      (real mission code)
```

Requires: ArduCopter HITL build for F405, Gazebo with HITL plugin.

### Limitation

SpeedyBee F405 HITL support: verify with ArduPilot HITL documentation for the F405
target. Not all F405 firmware builds include HITL. May need a custom build.

### When to use

After SITL testing, before first outdoor flight. Gives confidence that the physical
RPi-to-FC serial link works correctly.

---

## Proposal C: Detection Pipeline Testing on Recorded Footage

### The most important test to run first

You don't need any drone hardware to test mine detection. Film test footage and run
the detection pipeline on it.

### Setup

```bash
# Record 30-second video at your test location
# Place plastic plates/discs on grass at various distances
# Film from 2m height (stand on a chair or use a pole mount)

# Save as footage/test_field_01.mp4

# Run detection pipeline on recorded footage
python3 slave/src/detection_test.py --video footage/test_field_01.mp4
```

```python
# detection_test.py
import cv2
from detection_node import yolo_detect, hsv_detect, hough_detect, fuse_detections

def test_on_video(video_path):
    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    detections_per_frame = []

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        dets = []
        dets += yolo_detect(yolo_model, frame)
        dets += hsv_detect(frame)
        dets += hough_detect(frame)
        fused = fuse_detections(dets, thermal=None)
        detections_per_frame.append(len(fused))

        # Visualise detections
        for d in fused:
            cx, cy = d['pixel']
            cv2.circle(frame, (cx, cy), 50, (0, 255, 0), 2)
        cv2.imshow('Detections', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Summary
    print(f"Frames with ≥1 detection: {sum(1 for d in detections_per_frame if d > 0)}")
    print(f"Total frames: {total_frames}")
    cap.release()

test_on_video('footage/test_field_01.mp4')
```

### What you can learn from recorded footage testing

1. Does YOLO detect your specific test discs (calibrate confidence threshold)?
2. How many false positives on clean grass areas?
3. Does the persistence filter (3-frame confirmation) reduce noise effectively?
4. What happens to detection at different angles (drone tilted by wind)?
5. Can the pipeline handle multiple mines in frame simultaneously?

### Typical test cases to record

| Test | Setup | What you measure |
|---|---|---|
| Single red disc on green grass | 1 disc, overhead | Baseline detection rate |
| Single brown disc on brown soil | 1 disc, hard case | Hardest case detection |
| Multiple discs, various colours | 5–10 discs | Multi-mine frame handling |
| Disc at 45° camera tilt | Tilt camera sideways | Attitude-affected detection |
| Moving camera (simulated flight) | Film while walking) | Detection at motion blur |
| Buried marker: peg on ground | Small coloured peg | Buried mine marker detection |

### Cost and time

**Zero cost.** Any plastic plates or frisbees serve as mine mockups.
**2–3 hours** of filming and iteration gives 80% of the detection quality insight
you'd get from a full flight test.

**Do this before writing a single line of drone control code.**

---

## Proposal D: Indoor Test with Cardboard/Plastic Disc Mockups

### Scope

Fly the real drone indoors over a simulated mine field. Use a room with ≥3 m ceiling.
Place coloured cardboard discs on the floor. Fly at 1.5–2 m altitude.

### What to test

- Does the drone maintain position at 2 m altitude (optical flow / barometer)?
- Does the camera capture clear images at the frame rate needed?
- Do the mine discs appear the same size as expected from the math (~61 pixels)?
- Does the detection pipeline trigger on real drone footage (noise, vibration, blur)?
- Does ArduPilot waypoint navigation follow the planned lawnmower correctly?

### Key differences from outdoor test

- No wind → easier position hold
- Different lighting (indoor artificial light) → colours look different
- No GPS at all → pure optical flow + barometer
- Much smaller space → test short strip, not full 60 m

### Minimal viable indoor setup

```
Room: ≥4 m × 8 m minimum (even a hallway)
Altitude: 1.5 m (lower than outdoor for ceiling clearance)
Discs: 10× cardboard circles, 25 cm diameter, various colours
Pattern: Single lawnmower strip, 4 m × 8 m
Duration: 3–4 minutes per test
Safety: Prop guards mandatory, remove bystanders
```

### What breaks indoors that won't tell you about outdoor

- Optical flow works beautifully on a flat floor tile → may not predict grass performance
- No wind → no attitude perturbation → no angle-based detection errors
- Lighting is controllable → won't reveal time-of-day issues

**Treat indoor tests as "does the code work in principle" tests.** They don't validate
the full system.

---

## Testing priority order

| Priority | Test | Time | What it finds |
|---|---|---|---|
| 1 | Detection on recorded footage | 2–3 hours | Detection accuracy, threshold tuning |
| 2 | SITL mission logic | 4–8 hours | State machine bugs, waypoint logic |
| 3 | Detection on live RPi camera (stationary) | 1–2 hours | Camera integration, latency |
| 4 | Indoor hover + lawnmower | 1–2 hours | Navigation, waypoint following |
| 5 | Indoor detection in flight | 2–3 hours | End-to-end pipeline, vibration effects |
| 6 | Outdoor short-range test (15 × 15 m) | 4–6 hours | Real conditions, optical flow outdoors |
| 7 | Full-scale outdoor (15 × 60 m) | 4–8 hours | Complete competition rehearsal |

---

## Automated test suite

Create automated tests that run on every code change:

```python
# tests/test_detection.py
import pytest
import cv2
import numpy as np

def test_hsv_detects_red_disc():
    # Create synthetic frame: red circle on green background
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (34, 139, 34)  # Green background (BGR)
    cv2.circle(frame, (320, 240), 60, (0, 0, 200), -1)  # Red disc

    dets = hsv_detect(frame)
    assert len(dets) == 1, f"Expected 1 detection, got {len(dets)}"
    cx, cy = dets[0]['pixel']
    assert abs(cx - 320) < 20, f"X offset too large: {cx}"
    assert abs(cy - 240) < 20, f"Y offset too large: {cy}"

def test_persistence_filter_rejects_single_frame():
    pf = PersistenceFilter(confirm_frames=3)
    det = {'pixel': (320, 240), 'world_x': 15.0, 'world_y': 5.0, 'confidence': 0.9}
    # Single detection in one frame should NOT confirm
    result = pf.update([det], time.time())
    assert len(result) == 0

def test_astar_finds_path_around_mine():
    grid = np.zeros((60, 240), dtype=np.uint8)
    # Mine at centre of field (blocks straight path)
    mark_mine_on_grid(30.0, 7.5, grid)

    path = astar_safe_path(grid, (0, 30), (239, 30))
    assert path is not None, "No path found"
    # Verify path stays clear of mine exclusion zone
    for (xi, yi) in path:
        assert grid[yi, xi] == 0, f"Path goes through danger zone at ({xi}, {yi})"
```

Run these tests before every outdoor flight:
```bash
pytest tests/ -v
```

---

## References

1. ArduPilot SITL documentation:
   https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html

2. ArduPilot Gazebo plugin:
   https://github.com/khancyr/ardupilot_gazebo

3. AirSim (alternative simulator, Microsoft):
   https://arxiv.org/pdf/2308.10671 (referenced in detection research)

4. Gazebo classic documentation:
   https://gazebosim.org/docs/latest/getstarted

5. ArduPilot HITL testing:
   https://ardupilot.org/dev/docs/hitl-simulators.html
