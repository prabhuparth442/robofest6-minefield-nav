# 02 — GPS-Denied Navigation

> **GPS-denied review (2026-10-24):** Robofest 6.0 flies GPS-denied for the whole mission. Nothing in flight may depend on a GPS fix, a 'last known GPS position', lat/lon waypoints, or GPS-based failsafes. Position comes from EKF3 (optical flow + rangefinder + compass, local NED frame). This is a legacy 5.0-era note; the sections below were corrected, but treat `gps_denied/` as the current reference.

## Why this matters for us

The Robofest PS explicitly bans GPS during the 10-minute mission window. Our approach:
lock the start heading from the compass (→ `origin_state.json`), take the arming point as local NED (0, 0, 0), and send
local NED setpoints while ArduPilot EKF3 fuses optical flow + rangefinder/barometer + compass. This works, but drift accumulates. Understanding the alternatives helps us
harden the system and reduce positional errors over a 60 m field.

---

## Our Approach (corrected for GPS-denied 6.0)

> The original text of this section described the 5.0 design: a GPS fix saved to `origin_state.json`, waypoints converted to
> GPS coordinates and sent as `SET_POSITION_TARGET_GLOBAL_INT`, and a coverage grid painted from "real GPS feedback". That
> does not work when GPS is unavailable for the whole mission, so it was replaced.

**Local-frame navigation with ArduPilot EKF3:**

1. Pre-flight: compass heading averaged over 5 samples (circular mean) -> `origin_state.json` (heading only)
2. Arm: EKF3 sets local NED (0, 0, 0) at the arming point
3. In flight: all waypoints are local NED setpoints (MAVSDK `PositionNedYaw`); no lat/lon conversion exists
4. EKF3 fuses optical flow (velocity) + rangefinder/barometer (altitude) + compass (heading)
5. `grid_map.py` paints coverage from the EKF's local position estimate (not planned waypoints), so position drift is absorbed
   into the coverage map passively

**Weakness:** without absolute references, drift accumulates (about 0.3-0.5 m/min on EKF3 alone; less with flow). The
mitigations are below and in `gps_denied/`.

---

## Key Paper: Relative Positioning for Aerial Robot Path Planning in GPS-Denied Environment

**Reference:** arXiv:2409.10193 (2024)  
**URL:** https://arxiv.org/abs/2409.10193

### What it says

- Target application: bushfire monitoring swarms where GPS is unreliable
- Solution: **relative positioning** — each drone establishes its position relative to a
  known base of operations (not absolute world coordinates)
- Core idea: use structured landmarks visible to the drone cameras as anchors
- Achieves localisation good enough for search-and-reconnaissance without GPS

### Connection to our approach

Our "origin-lock" is conceptually similar — we establish a fixed reference at the start
point and everything is relative to it. The paper validates that this approach is viable.

**For Robofest 6.0:** The competition field has two fixed visual landmarks: the Pole and
the Statue. Their positions are pre-surveyed in `fieldmap.py`. A computer-vision step that
*detects and re-localises against these landmarks* mid-flight could correct accumulated drift.
This would be a significant improvement worth implementing.

---

## Key Paper: Autonomous Navigation for Drone Swarms in GPS-Denied Environments

**Reference:** PMC7256583  
**URL:** https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7256583/

### What it says

- Uses **structured learning** (a supervised ML approach) to train navigation policies in GPS-denied environments
- Key insight: when GPS is absent, the swarm needs a shared reference frame — typically
  established by the lead drone which has the most reliable localisation
- Demonstrates that master–slave architectures (our exact topology) handle GPS denial
  well because only the master needs reliable position; slaves can follow relative paths

### Connection to our architecture

We already use this pattern: the master drone is the reference (it holds `origin_state.json`
and the authoritative grid). Slaves operate relative to master-issued waypoints.

**For Robofest 6.0:** When a slave loses its ArduPilot EKF3 position estimate (EKF diverges), the
there is no GPS to fall back on, and a "last known position" from the master is just as drifted. A slave that loses its estimate should hold briefly and then LAND; re-anchoring should use absolute references (ArUco markers, Pole/Statue landmarks) rather than a remembered coordinate. Currently no such
re-anchor command exists in `tcp_channel.py`. Worth adding as a fallback.

---

## Key Paper: UAV Autonomous Navigation System Based on Air-Ground Collaboration

**Reference:** MDPI Drones 9(6):442 (2025)  
**URL:** https://www.mdpi.com/2504-446X/9/6/442

### What it says

- Combines ground-based reference station with aerial drones for GPS-denied navigation
- The ground station provides a stable coordinate anchor; drones triangulate relative to it
- Achieves 0.1–0.2 m positional accuracy without GPS

### Relevance to our setup

Our "master drone" acts as the ground reference — it has the fixed `origin_state.json`
and broadcasts the authoritative field coordinate system via UDP telemetry.

**For Robofest 6.0:** Consider whether the master drone should hover stationary over the
origin point for the entire mission (acting purely as coordinator / reference station)
rather than also scanning. This would improve overall accuracy at the cost of one scanner.

---

## Coordinate System — Implementation Details

Our coordinate system is defined in `04_coordinate_math.py` and `fieldmap.py`:

```
Origin:   Arming point = local NED (0, 0, 0) from EKF3; only the start heading is saved in origin_state.json
+X axis:  East  (compass 90°)
+Y axis:  North (compass 0°)
+Z axis:  Up

All mine positions have NEGATIVE Y (they are South of the start line).
Field runs: X = [-2, 32] m,  Y = [-60, 2] m
```

Mine and drone positions are kept directly in this local frame; no GPS-to-metres conversion is needed.

Camera intrinsics used for pixel → physical offset:
```
fx = 614.5  (55° horizontal FOV at 640 px wide)
fy = 761.2  (35° vertical FOV at 480 px tall)
```

These intrinsics match the 55°×35° FOV MLX90640 lens. If you change lens, recalculate:
```
fx = (width_px / 2) / tan(hfov_rad / 2)
```

---

## Drift Mitigation Techniques Used

| Technique | Where in code | Benefit |
|-----------|--------------|---------|
| Compass averaging (5 samples, circular mean) | `00_preflight_calib.py` | Accurate initial heading |
| Passive grid from the real EKF position (not planned path) | `grid_map.mark_position()` | Drift-tolerant coverage map |
| Fuzzy footprint disc (r=0.78m) | `grid_map.py` | Position noise fills coverage gaps |
| Persistence gate (re-hover + 12 frames) | `03_persistence.py` | Rejects false positives from drift |
| Mine deduplication radius 1.5 m | `05_map_verifier.py` | Fuses near-duplicate reports |

---

## For Robofest 6.0 — Recommended Improvements

1. **Optical flow sensor** (e.g. ArduPilot-compatible optical flow sensor (PX4Flow or Pi cam + Lucas-Kanade)) — can cut EKF drift
   from ~0.3 m/min to ~0.05 m/min over flat ground
2. **UWB anchor beacons** at field corners — provide cm-level indoor positioning with no GPS;
   check if PS rules permit placing anchors at field boundary
3. **Landmark detection** — detect Pole/Statue via camera and correct drone position mid-flight
4. **EKF re-anchor TCP command** — master can send corrected position to drifting slave
