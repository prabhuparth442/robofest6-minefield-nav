# GPS-Denied Navigation — Complete Overview

> **GPS-denied review (2026-10-24):** Robofest 6.0 flies GPS-denied for the whole mission. Nothing in flight may depend on a GPS fix, a 'last known GPS position', lat/lon waypoints, or GPS-based failsafes. Position comes from EKF3 (optical flow + rangefinder + compass, local NED frame). Parts 3 and 5 below were corrected in this review.

## Start here if you're new to drones or navigation

---

## Part 1 — What does "GPS-denied" even mean?

Your phone always knows where it is. It uses GPS — a network of satellites orbiting
Earth that send precise timing signals. Your phone listens to several satellites at once
and uses the differences in arrival times to triangulate: "I am at these coordinates."

GPS works great outdoors, in open sky. But it breaks in several situations:
- **Indoors** — satellite signals can't penetrate buildings well
- **Tunnels, forests, urban canyons** — signals are blocked or reflected off surfaces
- **Competition fields with rules** — some competitions (including Robofest 6.0) forbid
  using GPS inside the mission area, specifically to make the problem harder

**What happens to a drone that loses GPS?**

Without GPS, the drone has no external reference telling it where it is. It has to figure
out its own position using only what's on board — sensors, cameras, its own motion history.

This is the GPS-denied navigation problem.

---

## Part 2 — Why is this hard?

Imagine you're blindfolded in a field. You know your starting point. Someone walks you
around for 10 minutes. Can you return to the start?

You could try counting steps and keeping track of direction (dead reckoning). But small
errors in each step accumulate. After 200 steps, you might be 3 metres off your intended
path even if you were very careful.

Drones face exactly this problem. Sensors have noise. Every measurement is slightly wrong.
Over time, those small wrongs add up. This is called **drift**.

The central challenge of GPS-denied navigation is: **how do you stop drift?**

---

## Part 3 — The Robofest 6.0 context

| Constraint | What it means for navigation |
|---|---|
| 15 m × 60 m field | Moderate size — 60 m of travel in the long axis |
| No GPS inside boundary | No satellite fix once the mission starts |
| No GPS dependency at all | Treat GPS as unavailable for the whole mission, including start-up. The origin is the local NED point where the drone arms. (If the PS allows a pre-mission fix it may be logged as metadata only; flight must never require it.) |
| 10-minute mission window | Drift must stay small over ~10 min of flight |
| 3 drones, each ≤750 g | Every gram of sensor hardware costs something |
| Drones fly at ~2 m altitude | Low — close to ground, good for optical sensors |
| Outdoor environment | Wind, lighting variation, no structured features |

The key insight: **there is no absolute reference, so everything is relative.** The origin is the local NED point where
the drone arms (EKF3 defines it as 0,0,0), heading comes from the compass, and the drone navigates entirely in that local
frame for the mission duration. Absolute fixes can only come from things we put or find in the field (ArUco markers, the
Pole/Statue landmarks).

---

## Part 4 — The landscape of solutions

There are six broad families of GPS-denied navigation:

| # | Approach | Core idea | Needs extra hardware? |
|---|---|---|---|
| 1 | Dead reckoning (IMU) | Integrate acceleration to get velocity, integrate velocity to get position | No — IMU already on FC |
| 2 | Optical flow | Watch how the ground texture moves in a downward camera | Small flow sensor (~20 g) |
| 3 | Visual Inertial Odometry (VIO) | Fuse a camera + IMU together to estimate motion | Stereo/mono camera on Pi |
| 4 | SLAM | Build a map while navigating; use map landmarks to correct position | Camera or LiDAR |
| 5 | UWB ranging | Measure time-of-flight distances to fixed beacons on the ground | UWB anchors + tags |
| 6 | Marker-based | Detect known visual markers (ArUco/AprilTag) to get absolute position | Printed markers on field |

Each approach has fundamentally different accuracy, weight, cost, compute, and robustness
profiles. The right answer for your problem depends on your constraints.

---

## Part 5 — How our system works (GPS-free)

```
Before mission (no GPS needed):
  1. Drone sits at the start zone, level, motors off
  2. Compass heading: average 5 samples with a circular mean -> origin_heading
  3. Save {heading_deg} to origin_state.json (heading only; no lat/lon)
  4. Arm. EKF3 defines local NED (0, 0, 0) at this point

During mission:
  5. EKF3 on the SpeedyBee F405 fuses:
       - Optical flow (horizontal velocity)       EK3_SRC1_VELXY = 5
       - Rangefinder / barometer (altitude)
       - Magnetometer (heading)
       - Accelerometer + gyroscope
     with GPS disabled for position (EK3_SRC1_POSXY = 0)
  6. Our code works in the local frame only and sends local NED setpoints
     (MAVSDK PositionNedYaw / SET_POSITION_TARGET_LOCAL_NED). There is no lat/lon conversion.
  7. Absolute corrections, when available, come from ArUco markers or landmarks (see 05_marker_based.md)
```

**If the position estimate degrades (flow lost, EKF variance high):** there is no "last known GPS position" to fall back on.
The EKF's estimate is the only position the drone has, and it is drifting. The safe response is to hold briefly if the
estimate is still usable, then LAND in place (see `swarm/05_swarm_failure_modes.md`), not to fly back to a remembered
coordinate.

**The known weakness:** EKF3 dead reckoning without flow drifts ~0.3-0.5 m per minute. Over 10 minutes at 60 m range, position
error could reach 3-5 m. For a 15 m wide field, that is significant: a detection reported at (X=2, Y=20) could actually be at
(X=4, Y=21). The following documents explore every alternative and how to fix this.

---

## Reading order

| File | What you'll learn |
|---|---|
| `01_dead_reckoning.md` | What we do now — IMU integration, EKF, and why it drifts |
| `02_optical_flow.md` | Using a downward camera to measure ground movement |
| `03_vio_slam.md` | Computer vision fused with IMU — the modern approach |
| `04_uwb_ranging.md` | Radio time-of-flight ranging — like GPS but local |
| `05_marker_based.md` | ArUco / AprilTag visual absolute references |
| `06_lidar_slam.md` | LiDAR-based 2D/3D SLAM |
| `07_comparison_vs_ps.md` | Head-to-head comparison against Robofest 6.0 constraints |
| `08_upgrade_path.md` | Concrete implementation plan: what to add and how |

---

## Key vocabulary

| Term | Plain English |
|---|---|
| **Dead reckoning** | Estimating current position by integrating known velocities from a known start |
| **Drift** | Accumulated position error over time due to sensor noise |
| **EKF** | Extended Kalman Filter — a mathematical algorithm that fuses multiple noisy sensors optimally |
| **IMU** | Inertial Measurement Unit — accelerometers + gyroscopes, measures linear and rotational motion |
| **Optical flow** | How pixels in a camera image shift frame-to-frame — reveals the camera's lateral motion |
| **VIO** | Visual-Inertial Odometry — combines camera feature tracking with IMU readings |
| **SLAM** | Simultaneous Localization and Mapping — builds a map and estimates position in it at the same time |
| **UWB** | Ultra-Wideband radio — measures time-of-flight between transceivers to ≤10 cm accuracy |
| **ArUco** | A type of square fiducial marker (like a QR code) that a camera can localise precisely in 3D |
| **Odometry** | Estimating position from motion measurements (the word comes from "odometer") |
| **6-DoF** | Six degrees of freedom — X, Y, Z position + roll, pitch, yaw orientation |
| **Fusion** | Combining multiple sensor readings into one estimate, weighted by their reliability |
