# 01 — Dead Reckoning & EKF-Based Navigation

> **GPS-denied review (2026-10-24):** Robofest 6.0 flies GPS-denied for the whole mission. Nothing in flight may depend on a GPS fix, a 'last known GPS position', lat/lon waypoints, or GPS-based failsafes. Position comes from EKF3 (optical flow + rangefinder + compass, local NED frame). The old GPS-averaged origin lock and `local_to_gps()` conversion were removed from this file.

## What we currently use, and why it drifts

---

## The layman version

Imagine closing your eyes and walking across a room. You count your steps, keep track of
turns. When you stop, you can estimate roughly where you are — but the longer you walk,
the less confident you are. Every step introduces a tiny error. After 50 steps, errors
from all 50 steps have added up.

Dead reckoning is exactly this: **estimate where you are now by integrating how you've
been moving since a known starting point.**

For a drone, "steps" are replaced by acceleration measurements from an IMU.

---

## The physics

An IMU (Inertial Measurement Unit) gives you:
- **Accelerometers** — measure linear acceleration in X, Y, Z (in m/s²)
- **Gyroscopes** — measure angular rotation rate in roll, pitch, yaw (in rad/s)

To get position from an accelerometer:

```
acceleration (m/s²)
  → integrate once → velocity (m/s)
  → integrate again → position (m)
```

This is called **double integration**. Each integration step amplifies noise.

### The error cascade

If your accelerometer has a bias error of just 1 mg (0.001 × 9.81 m/s²):

```
After 1 second:    velocity error  = 0.00981 m/s
                   position error  = 0.004 m  (barely noticeable)

After 60 seconds:  velocity error  = 0.589 m/s
                   position error  = 17.7 m   ← unusable

After 10 minutes:  position error  ≈ hundreds of metres
```

This is why **pure IMU dead reckoning is unusable after ~10 seconds** without corrections.

---

## How ArduPilot EKF3 makes it survivable

ArduPilot's Extended Kalman Filter doesn't use the IMU alone. It fuses:

| Sensor | What it measures | Update rate |
|---|---|---|
| Accelerometer (IMU) | Linear acceleration | 400 Hz |
| Gyroscope (IMU) | Angular rate | 400 Hz |
| Barometer (DPS310 on SpeedyBee) | Altitude (Z-axis only) | 25 Hz |
| Magnetometer (compass) | Heading direction | 10 Hz |
| GPS (when available) | Absolute XY position | 5–10 Hz |

**Key insight:** the barometer provides absolute altitude, which breaks the Z-axis
double-integration chain. Without GPS, XY position must still be dead-reckoned — but
at least altitude is bounded.

### What EKF3 does (simplified)

EKF3 maintains two things simultaneously:
1. **State estimate** — best current guess of position, velocity, attitude
2. **Uncertainty covariance** — how confident it is in each state variable

Every time a sensor reading arrives, EKF3 updates:

```
Prediction step (runs at 400 Hz with IMU):
  new_state = model(old_state, imu_reading)
  new_uncertainty = grows (more time = more drift)

Update step (runs when other sensors arrive):
  innovation = sensor_reading - predicted_sensor_reading
  kalman_gain = uncertainty / (uncertainty + sensor_noise)
  corrected_state = predicted + kalman_gain × innovation
  corrected_uncertainty = shrinks
```

The barometer and magnetometer constrain Z and heading continuously. But **without
GPS or another XY reference, horizontal position uncertainty grows without bound**.

---

## Measured drift rates in our system

Based on ArduPilot EKF3 field tests with SpeedyBee F405 + compass + barometer only
(no GPS, no optical flow):

| Duration | Expected XY drift | Practical effect |
|---|---|---|
| 0–60 s | 0.1–0.3 m | Detection accuracy still good |
| 1–3 min | 0.3–1.0 m | Mine position errors acceptable |
| 3–7 min | 1.0–3.0 m | Detection locations getting unreliable |
| 7–10 min | 3.0–5.0 m | ±5 m error on a 15 m wide field is critical |

Drift rate is roughly **0.3–0.5 m/minute** for a well-calibrated SpeedyBee with
magnetometer and barometer, rising to 0.8–1.2 m/min in the second half of the mission
as the filter's uncertainty covariance grows.

The 10-minute mission window is exactly where dead reckoning starts to fail seriously.

---

## What "EKF3" means vs older EKF2

ArduPilot has had two generations of EKF:

| Feature | EKF2 (older) | EKF3 (current) |
|---|---|---|
| Optical flow | ✗ not supported | ✓ fused natively |
| VIO input | ✗ | ✓ via VISION_POSITION_ESTIMATE MAVLink |
| Range finders | Limited | ✓ multiple |
| GSF (GPS Fix and Go) | ✗ | ✓ — can switch between GPS and non-GPS mid-flight |
| Affine noise model | ✗ | ✓ — better models sensor noise |

SpeedyBee F405 ships with ArduCopter 4.4+ which uses EKF3 exclusively.

---

## Our origin strategy (GPS-free)

The mission is GPS-denied, so the flight code never uses a GPS fix. The origin is simply where the drone arms: EKF3 defines
local NED (0, 0, 0) there, and every waypoint is a local NED setpoint. The only thing worth capturing before takeoff is
**heading**, from the compass:

```python
COMPASS_SAMPLES = 5

# Average compass headings with a circular mean
# (a plain average fails near 0/360 degrees)
angles_rad = [deg_to_rad(h) for h in compass_samples]
sin_mean = mean([sin(a) for a in angles_rad])
cos_mean = mean([cos(a) for a in angles_rad])
origin_heading = atan2(sin_mean, cos_mean)

save_json("origin_state.json", {"heading_deg": rad_to_deg(origin_heading)})   # no lat/lon
```

**Why circular mean for compass?** If headings are 358, 1, 2 degrees, a regular average gives 120 degrees (wildly wrong).
The sin/cos mean correctly gives about 0.33 degrees.

### Field frame to local NED

Define the field frame with x along the 60 m length and y across the 15 m width, from the start point. If the field's long
axis points along compass bearing psi (measured at start), a field point (x_f, y_f) maps to local NED (north, east) by:

```python
north = x_f * cos(psi) - y_f * sin(psi)
east  = x_f * sin(psi) + y_f * cos(psi)
# send as PositionNedYaw(north, east, -altitude, yaw)
```

Any heading error multiplies into position error (a 5 degree error at 60 m is about 5.2 m sideways), which is why compass
calibration matters and why marker or landmark re-anchoring is valuable.

**Pre-flight sanity check:** confirm on the bench or in SITL that arming, offboard/GUIDED and LAND work with GPS disabled
and flow providing velocity. Some ArduPilot features (RTL, SmartRTL, polygon fences) assume a global home/EKF origin and
may not behave as expected without GPS, so do not rely on them (see `swarm/03_collision_avoidance.md`).

---

## Weaknesses and mitigations

| Weakness | Current mitigation | Better mitigation |
|---|---|---|
| XY drift ~0.5 m/min | Thermal detection re-hover (12 frames) reduces false-positive impact | Add optical flow (see 02_optical_flow.md) |
| Compass disturbance from motors | Motor-off compass calibration during preflight | Keep compass away from power cables |
| Barometer affected by prop wash | FC placed away from prop downwash | Barometric reference taken before arming |
| Heading error multiplies XY error | Circular-mean averaging over 5 samples | Better magnetometer (external) |

---

## Key papers and references

1. **Thrun, S., Burgard, W., Fox, D. (2005).** *Probabilistic Robotics.* MIT Press.
   Chapters 3–4 cover Kalman Filters and dead reckoning drift mathematically.
   (Copy in `research/papers/`)

2. **Jarraya et al. (2025).** *GNSS-Denied UAV Navigation: Analyzing Computational
   Complexity, Sensor Fusion, and Localization Methodologies.*
   Satellite Navigation, 6:9. https://link.springer.com/article/10.1186/s43020-025-00162-z

3. **ArduPilot EKF3 documentation:**
   https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html

4. **ArduPilot EKF3 optical flow fusion issue (real field data):**
   https://github.com/ArduPilot/ardupilot/issues/31964
