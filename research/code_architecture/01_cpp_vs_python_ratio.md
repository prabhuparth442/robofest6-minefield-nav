# 01 — C++ vs Python: What to Write in Which Language

## For this specific PS and hardware stack

---

## The fundamental trade-off

**Python:**
- Faster to write and iterate
- Better library support (pymavlink, ultralytics, OpenCV Python, numpy, scipy)
- Slower at runtime (CPython: interpreted, GIL, no native parallelism per thread)
- Typical loop rate cap: ~100–300 Hz for simple processing loops
- Fine for logic that runs at 1–30 Hz

**C++:**
- Slower to write, harder to debug
- Runs at native CPU speed
- True parallelism (pthreads, no GIL)
- Needed for: >300 Hz control loops, real-time camera capture at 60+ fps, direct
  hardware register access, operations that must complete in <1 ms

---

## What actually needs what language

### Latency analysis

| Component | Required rate | Python achievable? | Notes |
|---|---|---|---|
| pymavlink position command | 10 Hz | ✓ Yes | ~0.5 ms per send |
| Camera capture + publish | 30 Hz | ✓ Yes (Pi Cam CSI-2) | Use `picamera2` library |
| YOLOv8n-ncnn inference | 8–10 Hz | ✓ Yes | ncnn runs in C++, Python calls it via wrapper |
| HSV detection (OpenCV) | 30 Hz | ✓ Yes | cv2 is C++ under the hood |
| Hough circle detection | 30 Hz | ✓ Yes | Same as above |
| Thermal sensor read (I2C) | 8 Hz | ✓ Yes | RPi I2C is slow; 8 Hz is within Python |
| Mine map update (grid write) | 10 Hz | ✓ Yes | 14 KB numpy array write is ~0.1 ms |
| A* path planning | Once | ✓ Yes | ~50–100 ms in Python for 14K node grid |
| ORCA velocity computation | 10 Hz | ✓ Yes | ~5 ms in Python for 3 drones |
| WiFi UDP send/receive | 10 Hz | ✓ Yes | Socket operations are OS calls |
| Optical flow relay (if software) | 30 Hz | ⚠️ Marginal | Lucas-Kanade in C++ via cv2 is fast |
| EKF data relay (AP_DDS bridge) | 50 Hz | ⚠️ Marginal | Use C++ if AP_DDS needed at 50 Hz |
| High-frequency servo PWM | 50–100 Hz | ✗ No | Use Lua on FC or direct PWM library in C++ |

**Conclusion:** For 2 m/s scan speed and 30 Hz camera, Python is sufficient for every
component EXCEPT hardware-level PWM control and high-frequency sensor relay if needed.

---

## Proposal A: 100% Python

### Architecture

```
main_orchestrator.py     (mission state machine, pymavlink calls)
camera_capture.py        (picamera2, publishes frames)
detection_pipeline.py    (YOLOv8n-ncnn, HSV, Hough — all Python)
thermal_reader.py        (MLX90640 via I2C, scikit-mlx90640)
map_manager.py           (numpy grid, A* pathfinding)
comms_manager.py         (UDP socket, WiFi broadcast/receive)
led_controller.py        (RPi.GPIO for LED control)
```

### Real-world performance on RPi 4B

At 2 m/s scan speed, we need detections every ~130 ms (to ensure ≥3 frames per mine
at 30 fps, with 100 ms between confirmations). Python can reliably achieve this.

Measured task times on RPi 4B (4GB RAM):

| Task | Measured time |
|---|---|
| YOLOv8n-ncnn inference (320×320) | ~100–120 ms |
| OpenCV HSV + blob | ~5 ms |
| Hough circles | ~20–30 ms |
| MLX90640 frame read (I2C) | ~125 ms (8 Hz = 125 ms per frame) |
| pymavlink send position | ~0.5 ms |
| A* on 14,400 node grid | ~50 ms |
| UDP broadcast | <1 ms |

**The bottleneck is YOLOv8n-ncnn at 100–120 ms → ~8–10 FPS.**

The detection runs in a separate thread, so it doesn't block mission logic:

```python
import threading
import queue

frame_queue = queue.Queue(maxsize=2)     # Camera → detection
detection_queue = queue.Queue(maxsize=10)  # Detection → map

def camera_thread():
    cam = picamera2.Picamera2()
    cam.start()
    while True:
        frame = cam.capture_array()
        if frame_queue.full():
            frame_queue.get_nowait()  # Drop oldest frame
        frame_queue.put(frame)

def detection_thread():
    while True:
        frame = frame_queue.get()
        detections = yolo_detect(frame)
        detections += hsv_detect(frame)
        detections += hough_detect(frame)
        detection_queue.put(fuse(detections))

def mission_thread():
    while True:
        if not detection_queue.empty():
            dets = detection_queue.get()
            map_manager.update(dets)
        send_next_waypoint()
        time.sleep(0.05)  # 20 Hz mission loop
```

### When 100% Python is NOT enough

- You need camera at 60+ fps (Python overhead may cause frame drops)
- You want EKF data relay at 100+ Hz
- You add computer vision that isn't wrapped in a C++ library

### Verdict
**Use 100% Python for everything in Proposal A.** The system runs at safe margins for
our 2 m/s, 8–10 FPS, 30 Hz camera requirements. YOLOv8n-ncnn is implemented in C++
internally — you just call it from Python. OpenCV is also C++ internally.

---

## Proposal B: Python mission logic + C++ sensor nodes

### What moves to C++

**C++ ROS 2 node: optical_flow_relay**

If you're implementing software optical flow (Lucas-Kanade on RPi camera instead of
Matek 3901-L0X hardware), the flow computation must run at 30+ Hz:

```cpp
// ros2_optical_flow/src/optical_flow_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/video/tracking.hpp>

class OpticalFlowNode : public rclcpp::Node {
public:
    OpticalFlowNode() : Node("optical_flow") {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/drone/camera/image_raw", 10,
            std::bind(&OpticalFlowNode::on_image, this, std::placeholders::_1));
        flow_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/drone/optical_flow_vel", 10);
    }

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "mono8")->image;

        if (!prev_frame_.empty()) {
            std::vector<cv::Point2f> prev_pts, curr_pts;
            // Harris corners on previous frame
            cv::goodFeaturesToTrack(prev_frame_, prev_pts, 200, 0.01, 10);
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prev_frame_, frame, prev_pts, curr_pts,
                                      status, err);
            // Compute mean flow vector → publish as velocity
            publish_velocity(prev_pts, curr_pts, status);
        }
        prev_frame_ = frame;
    }
    // ...
};
```

This runs at 30 Hz with ~5 ms per frame in C++. Python equivalent: ~30–50 ms per frame.

**C++ ROS 2 node: high-frequency MAVLink relay (if using AP_DDS)**

AP_DDS bridge runs natively in C++ on the SpeedyBee. The microxrceddsagent
(Micro XRCE-DDS Agent) is a C++ executable. You don't write this — you run the
prebuilt binary:

```bash
# Install on RPi
pip install microxrceddsagent  # or build from source
# Run the bridge
microxrceddsagent serial --dev /dev/ttyAMA0 -b 921600
```

### When Proposal B is worth the effort

Only if you measure actual Python performance and hit a specific bottleneck. Don't
prematurely optimise.

### Verdict
**Implement Proposal A first.** Add C++ optical flow node only if you decide to go
software optical flow instead of Matek 3901-L0X hardware. If you use the hardware
sensor, Proposal A is completely sufficient.

---

## Proposal C: C++ core + Python glue

### What this means

Heavy C++ ROS 2 nodes for all perception and control. Python only for high-level
mission orchestration.

```
C++ nodes:
  camera_node       → 30 Hz image capture
  detection_node    → YOLOv8 C++ API directly (no Python wrapper)
  thermal_node      → MLX90640 I2C read in C++
  mavlink_bridge    → Direct serial comms in C++

Python nodes:
  mission_node      → State machine (SCAN/AGGREGATE/GUIDE)
  map_node          → A* (numpy) — fine in Python for once-per-mission use
  swarm_comms_node  → UDP socket
```

### Is this worth it?

Only if you need:
- Strict determinism (real-time OS scheduling guarantees)
- Sub-10 ms latency through the entire pipeline
- >30 FPS detection

For a drone flying at 2 m/s, none of these are required. Python at 10 Hz detection
is provably sufficient.

**C++ is a time investment of ~3× over Python** for the same functionality. Do not
choose C++ unless Python performance is a measured bottleneck.

### Verdict
**Don't implement Proposal C for Robofest 6.0.** The hardware speed requirement doesn't
justify the development time cost.

---

## Recommendation matrix

| Task | Language | Reason |
|---|---|---|
| Mission state machine | Python | Simplicity, easy to modify |
| YOLO inference | Python (calls C++ ncnn) | ncnn handles the heavy lifting |
| OpenCV detection | Python (calls C++ cv2) | Same — C++ under the hood |
| pymavlink control | Python | Fine at 10 Hz |
| UDP comms | Python | Socket library is C under hood |
| A* path planning | Python (numpy) | Runs once — latency unimportant |
| MLX90640 read | Python | 8 Hz is within Python capability |
| Software optical flow | **C++** if needed | 30 Hz flow computation in Python is marginal |
| LED PWM control | Lua on FC (or RPi.GPIO) | Hardware-level timing |
| Chalk servo control | ArduPilot Lua or RPi.GPIO | Deterministic servo timing |

---

## Summary

For a Robofest 6.0 drone flying at 2 m/s:
- **Write everything in Python** (Proposal A)
- Add C++ only for software optical flow if you go that route
- Trust that OpenCV, ncnn, and pymavlink are already C/C++ under the hood
- The development velocity advantage of Python far outweighs the marginal
  performance difference at our operating speeds

The goal is a working, debuggable system on competition day — not a perfectly
optimised one. Python gets you there faster.
