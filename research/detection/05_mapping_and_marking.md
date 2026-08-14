# 05 — Mine Mapping and Safe Path Marking

## After detection: where is the mine, and how do we mark the safe path?

---

## Part 1 — The coordinate chain problem

When Drone 1 detects a mine, here is what it actually knows:

```
1. The mine appeared at pixel (cx=320, cy=240) in the image
2. The drone thinks it is at local NED position (x=14.3, y=2.1, z=-2.0)
3. The drone's heading (yaw) is approximately 0° (pointing down the X axis)
```

To place the mine on the world map, we need to convert:
```
pixel (cx, cy) → camera frame (Xc, Yc, Zc) → body frame (Xb, Yb, Zb) → NED frame (Xn, Yn, Zn)
```

Each step introduces error. The final mine position error is the sum of errors from:
1. Navigation error (how accurately does the drone know its own position?)
2. Altitude error (how accurately does it know its height above ground?)
3. Camera pointing angle error (is the camera truly pointing straight down?)
4. Pixel localisation error (is the mine centre at exactly that pixel?)

**Target position accuracy:** ±0.3 m or better (to ensure 1 m exclusion zone is safe)

---

## The coordinate transform

### Image → camera frame

For a camera with focal length f (pixels) pointing down, a pixel at (cx, cy) in an
image of size (W, H) maps to camera-frame direction:

```python
def pixel_to_camera_ray(cx, cy, W, H, fov_h_deg):
    """Returns unit direction vector in camera frame"""
    f_px = W / (2 * np.tan(np.radians(fov_h_deg / 2)))

    # Camera frame: X=right, Y=down, Z=forward (optical axis pointing down for nadiral)
    dx = (cx - W/2) / f_px
    dy = (cy - H/2) / f_px
    dz = 1.0

    norm = np.sqrt(dx**2 + dy**2 + dz**2)
    return np.array([dx/norm, dy/norm, dz/norm])
```

### Camera ray → ground intersection

The camera points down (nadir). The ray intersects the ground at altitude 0 (z=0),
given drone altitude h:

```python
def ray_to_ground(camera_ray, drone_altitude_m):
    """
    Assumes camera pointing straight down (nadir mount, no gimbal)
    camera_ray: unit vector in camera/body frame
    Returns offset from drone nadir point in metres
    """
    # Scale ray so its Z component = h (ground is h metres below drone)
    scale = drone_altitude_m / camera_ray[2]
    dx_m = camera_ray[0] * scale   # X offset from drone position (m)
    dy_m = camera_ray[1] * scale   # Y offset from drone position (m)
    return dx_m, dy_m
```

### Body → NED (accounting for drone attitude)

The drone may not be perfectly level. Roll and pitch angles from the IMU rotate the
camera's pointing direction:

```python
def body_to_ned_offset(dx_body, dy_body, roll_rad, pitch_rad, yaw_rad):
    """
    Rotates body-frame offset into NED frame using drone's euler angles
    """
    # Rotation matrix: ZYX euler convention (yaw, pitch, roll)
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)

    # Full rotation matrix R_body_to_NED
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr           ]
    ])

    body_vec = np.array([dx_body, dy_body, 0.0])
    ned_offset = R @ body_vec
    return ned_offset[0], ned_offset[1]  # North, East offsets in metres
```

### Full pipeline

```python
def pixel_to_world(cx, cy, W, H, fov_h,
                   drone_x, drone_y, drone_alt,
                   roll, pitch, yaw):
    """Returns world position of mine in local NED frame"""
    ray = pixel_to_camera_ray(cx, cy, W, H, fov_h)
    dx_body, dy_body = ray_to_ground(ray, drone_alt)
    dn, de = body_to_ned_offset(dx_body, dy_body, roll, pitch, yaw)

    mine_x = drone_x + dn   # NED North
    mine_y = drone_y + de   # NED East
    return mine_x, mine_y
```

---

## Proposal A: Optical Flow Position Log

### How it works

Our current system uses optical flow (Matek 3901-L0X) to estimate the drone's local
NED position. This is the only position reference in GPS-denied mode.

Typical optical flow accuracy: ±0.3–0.8 m drift over 10 minutes (see
`gps_denied/02_optical_flow.md`). Mine position accuracy = drone position accuracy +
pixel projection error.

For a mine at pixel centre (cx = W/2, cy = H/2): no pixel offset → mine is directly
below drone → mine position = drone position ± navigation error.

For a mine at the image corner: offset is ~1.3 m from nadir → adds ~0.05 m of attitude-
induced error (at 2° typical roll/pitch). Total error still dominated by navigation error.

**Achievable mine position accuracy with optical flow: ±0.4–0.9 m**

This is marginal for a 1 m exclusion zone requirement. A ±0.9 m mine position error
means the actual mine could be 0.9 m from where we map it. Our 1 m exclusion zone
might only provide 0.1 m true clearance.

**Mitigation:** Mark mines with a 1.5 m exclusion zone in the software map to account
for position uncertainty. This reduces path finding options slightly but increases safety.

### Implementation

```python
# In main_orchestrator.py
def log_mine_detection(detection, mavlink_conn):
    msg = mavlink_conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
    if msg:
        drone_x = msg.x    # NED North in metres
        drone_y = msg.y    # NED East in metres
        drone_z = -msg.z   # altitude (NED z is negative up)

        # Get attitude for projection
        att = mavlink_conn.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
        mine_x, mine_y = pixel_to_world(
            cx=detection['cx'], cy=detection['cy'],
            W=640, H=480, fov_h=66,
            drone_x=drone_x, drone_y=drone_y, drone_alt=drone_z,
            roll=att.roll, pitch=att.pitch, yaw=att.yaw
        )
        detection['world_x'] = mine_x
        detection['world_y'] = mine_y
        return detection
```

---

## Proposal B: Grid-Cell Map

### How it works

Represent the field as a 2D grid. When a mine is detected, mark the mine's grid cell
and all cells within 1 m (4 cells at 0.25 m resolution) as occupied.

```python
GRID_RESOLUTION = 0.25   # metres per cell
FIELD_W = 15.0           # Y axis
FIELD_L = 60.0           # X axis
GRID_H = int(FIELD_W / GRID_RESOLUTION)   # 60 rows
GRID_W = int(FIELD_L / GRID_RESOLUTION)   # 240 cols

mine_grid = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
EXCLUSION_RADIUS_CELLS = int(1.5 / GRID_RESOLUTION)  # 6 cells = 1.5m safety margin

def mark_mine_on_grid(world_x, world_y, grid):
    xi = int(world_x / GRID_RESOLUTION)
    yi = int(world_y / GRID_RESOLUTION)
    for dy in range(-EXCLUSION_RADIUS_CELLS, EXCLUSION_RADIUS_CELLS + 1):
        for dx in range(-EXCLUSION_RADIUS_CELLS, EXCLUSION_RADIUS_CELLS + 1):
            if dx*dx + dy*dy <= EXCLUSION_RADIUS_CELLS**2:
                nx, ny = xi + dx, yi + dy
                if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                    grid[ny, nx] = 1   # 1 = mine/danger zone
```

**Memory:** 60 × 240 = 14,400 bytes = 14 KB. Trivial for RPi.
**A* on this grid:** <100 ms for full 15 × 60 m field.

---

## Proposal C: Optical Flow Coordinate Transform

**Already described above (Proposal A)** — the pixel-to-world transform is the
core of this approach. The key is using LOCAL_POSITION_NED from ArduPilot (which
incorporates optical flow) as the drone position.

For accuracy improvement, fuse multiple detections of the same mine:
```python
def fuse_mine_estimates(detections, position_uncertainty=0.5):
    """
    Weighted average of multiple detections of the same mine
    Inverse-variance weighting (detections near image centre have less projection error)
    """
    if not detections:
        return None

    # Detections near image centre (small projection offset) are more accurate
    weights = [1 / (d.get('projection_error', position_uncertainty)**2)
               for d in detections]
    total_w = sum(weights)

    fused_x = sum(d['world_x'] * w for d, w in zip(detections, weights)) / total_w
    fused_y = sum(d['world_y'] * w for d, w in zip(detections, weights)) / total_w
    return fused_x, fused_y
```

---

## Proposal D: ArUco Marker Absolute Reference

### How it works

Place ArUco markers at known positions on the field boundary. The drone's downward
camera detects these markers and computes its absolute position in the field frame
(not just relative to the start).

ArUco marker detection gives position accuracy of ~5–10 cm.

```python
import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# Known marker positions (absolute world coordinates)
MARKER_POSITIONS = {
    0: (0.0, 0.0),     # Start corner
    1: (0.0, 15.0),    # Far corner of start zone
    2: (60.0, 0.0),    # End corner
    3: (60.0, 15.0),   # Far end corner
}

def correct_position_with_aruco(frame, camera_matrix, dist_coeffs, current_pos):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return current_pos  # No marker visible, keep current estimate

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in MARKER_POSITIONS:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], 0.3, camera_matrix, dist_coeffs)

            # Compute drone position from marker pose
            corrected = aruco_to_ned(tvec[0], MARKER_POSITIONS[marker_id])
            return corrected  # Much more accurate than dead reckoning

    return current_pos
```

### Limitations

- **Requires pre-placing markers on the field boundary** before the mission. Is this
  allowed? The PS doesn't prohibit field markers — only prohibits "external computers"
  and GPS. A printed paper marker is not a computer.
- Drone must be in line-of-sight of a marker. At 60 m field length, markers at the
  far end are ~60 m away — ArUco detection at 60 m requires large markers (>1 m).

**Practical use:** Place markers every 15 m along field edges. Each drone sees at
least one marker per 15 m of travel. Resets position estimate to correct value every
15 m → maximum accumulated drift between corrections ≈ 0.3 m (optical flow at 15 m).

---

## Physical mine marking

### After detecting mines and computing safe path: physically mark the path

**Option 1: LED array on drone**
Drone flies along the safe path at 1 m altitude with bright green/white LEDs pointing
down. Person follows the lit corridor.

```python
# LED pattern: solid green = safe path, flashing red = mine exclusion zone
def activate_path_leds(safe_path_cells, all_mine_cells):
    for cell in safe_path_cells:
        gpio_set_led(GREEN, ON)
    for cell in all_mine_cells:
        gpio_set_led(RED, FLASH_2HZ)
```

**Option 2: Chalk/powder drop mechanism**
A servo-actuated hatch releases coloured chalk powder at path waypoints.
Chalk marks persist on the ground → person sees the path even after drone moves away.

Weight: servo (~8 g) + chalk container (~50 g) = ~58 g. Within budget.

**Option 3: Laser pointer**
Drone hovers above each safe waypoint and illuminates the ground with a low-power
visible laser. Person follows the dot.

**Recommendation:** LED corridor (Option 1) for real-time guidance. Chalk drop
(Option 2) if LEDs need to be sustained while drone is elsewhere.

---

## Summary

| Issue | Recommendation |
|---|---|
| Mine position accuracy | Optical flow + multi-frame averaging → ±0.5 m |
| Map representation | 0.25 m grid, 1.5 m exclusion radius |
| Coordinate transform | pixel → body → NED using attitude from ArduPilot |
| Absolute position fix | ArUco markers every 15 m if permitted by TAC |
| Physical marking | LED corridor during guidance phase |

---

## References

1. ArduPilot LOCAL_POSITION_NED MAVLink message:
   https://ardupilot.org/dev/docs/mavlink-commands.html

2. OpenCV ArUco marker detection:
   https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

3. ArduPilot optical flow position accuracy data:
   https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html
