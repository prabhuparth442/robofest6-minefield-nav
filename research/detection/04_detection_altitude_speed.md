# 04 — Detection Physics: Altitude, Speed, and Miss Probability

## Worked mathematics for optimal flight profile

---

## Variables defined

```
h     = drone altitude above ground (metres)
v     = drone horizontal scan speed (m/s)
f_cam = camera frame rate (fps)
F     = camera horizontal FOV (degrees)
W_cam = image width in pixels
H_cam = image height in pixels
d_mine = mine diameter (metres), range 0.20–0.30 m
f_IR  = MLX90640 frame rate (Hz), configurable up to 64 Hz
```

---

## Part 1 — Camera ground coverage

### Ground footprint at altitude h

For a camera with horizontal FOV = F degrees:

```
Ground width (GW) = 2 × h × tan(F/2)
Ground height (GH) = 2 × h × tan(F_vertical/2)
```

For RPi Camera Module 3 (66° × 49° FOV) at various altitudes:

| Altitude | GW (m) | GH (m) | Area (m²) |
|---|---|---|---|
| 1.5 m | 1.96 m | 1.44 m | 2.82 m² |
| 2.0 m | 2.62 m | 1.92 m | 5.03 m² |
| 2.5 m | 3.27 m | 2.40 m | 7.85 m² |
| 3.0 m | 3.93 m | 2.88 m | 11.3 m² |

### Pixel footprint (ground resolution)

```
px/m = W_cam / GW
mm/pixel = (GW × 1000) / W_cam
```

For 640 × 480 camera at 2 m altitude (GW = 2.62 m):
```
px/m (horizontal) = 640 / 2.62 = 244 px/m = 4.1 mm/pixel
px/m (vertical)   = 480 / 1.92 = 250 px/m = 4.0 mm/pixel
```

For 1920 × 1080 at 2 m:
```
1920 / 2.62 = 733 px/m = 1.36 mm/pixel
```

### Mine pixel size

A 25 cm mine at 2 m altitude, 640×480 camera:
```
Mine diameter in pixels = 0.25 × 244 = 61 pixels
Mine area in pixels ≈ π × (30.5)² ≈ 2,922 pixels²
```

At 1920×1080:
```
Mine diameter = 0.25 × 733 = 183 pixels — very large
```

**At 640×480, 2 m altitude: mine = 61 px diameter. Clearly detectable.**

---

## Part 2 — Time over a mine

### Scenario

Drone flies at speed v along the X axis. Mine is at a fixed position. The camera FOV
sweeps across the mine's position.

Time the mine is in the visual FOV:

```
t_visible = GW / v = (2 × h × tan(F/2)) / v
```

At h=2m, F=66°, v=2 m/s:
```
t_visible = 2.62 / 2.0 = 1.31 seconds
```

Frames during which mine is visible:
```
N_frames = t_visible × f_cam = 1.31 × 30 = 39 frames
```

The mine is in frame for 39 consecutive frames. Excellent — plenty for YOLO to detect.

What if we scan faster?

| Speed (m/s) | t_visible (s) | N_frames (30fps) | Verdict |
|---|---|---|---|
| 0.5 | 5.24 | 157 frames | Overkill |
| 1.0 | 2.62 | 79 frames | Very safe |
| 2.0 | 1.31 | 39 frames | Safe |
| 3.0 | 0.87 | 26 frames | Marginal |
| 4.0 | 0.65 | 20 frames | Risky |
| 5.0 | 0.52 | 16 frames | Too fast |

**Maximum safe scan speed for 30fps camera: ~3 m/s**

At 3 m/s with persistence filter requiring 3 frames for confirmation:
- Mine in view for 26 frames
- 3 frames out of 26 must be confirmed → probability = ~100% if each frame has >50% detection rate

---

## Part 3 — MLX90640 thermal coverage math

### Ground footprint at altitude h

MLX90640-D55 (55° × 35° FOV):

```
Thermal GW = 2 × h × tan(27.5°) = 2 × h × 0.5206
Thermal GH = 2 × h × tan(17.5°) = 2 × h × 0.3153
```

| Altitude | Thermal GW (m) | Thermal GH (m) | Area (m²) |
|---|---|---|---|
| 1.5 m | 1.56 m | 0.95 m | 1.48 m² |
| 2.0 m | 2.08 m | 1.26 m | 2.62 m² |
| 2.5 m | 2.60 m | 1.58 m | 4.11 m² |
| 3.0 m | 3.12 m | 1.89 m | 5.90 m² |

### Thermal pixel footprint

At 2 m, 32 × 24 array, thermal GW = 2.08 m, GH = 1.26 m:

```
X resolution = 2.08 / 32 = 6.5 cm/pixel
Y resolution = 1.26 / 24 = 5.25 cm/pixel
```

A 25 cm mine:
```
X pixels = 0.25 / 0.065 = 3.8 pixels ≈ 4 pixels
Y pixels = 0.25 / 0.0525 = 4.8 pixels ≈ 5 pixels
```

**Thermal frame rate:** MLX90640 at default setting = 8 Hz. At 2 m/s scan:
```
t_visible_thermal = 2.08 / 2.0 = 1.04 seconds
N_thermal_frames = 1.04 × 8 = 8.3 frames
```

8 thermal frames per mine. With a 3-frame persistence filter: confirms mine if detected
in 3 of 8 frames. Fine.

---

## Part 4 — Miss probability as function of speed and altitude

### Model

We model the probability of detecting a mine in a single frame as P_frame.
P_frame depends on:
- Mine pixel size relative to image (larger is easier)
- Motion blur (higher speed = more blur)
- Sensor noise

With N frames per pass and a required k-frame confirmation:

```
P_miss_per_pass = P(confirmed in fewer than k frames)
               = Σ_{i=0}^{k-1} C(N,i) × P_frame^i × (1-P_frame)^(N-i)
```

For simplicity, assume k=3 (3-frame confirmation), and various P_frame:

```python
from scipy.stats import binom

def miss_probability(N_frames, k_confirm, p_frame):
    """P(miss) = P(getting fewer than k_confirm detections in N_frames)"""
    p_miss = binom.cdf(k_confirm - 1, N_frames, p_frame)
    return p_miss

# Worked examples:
# At 2 m/s, 30 fps: N=39, k=3
# If P_frame = 0.50 (hard case): P_miss = binom.cdf(2, 39, 0.50) ≈ 0.0000001 → near-zero
# If P_frame = 0.20 (difficult): P_miss = binom.cdf(2, 39, 0.20) ≈ 0.0007 → 0.07%
# If P_frame = 0.05 (very hard): P_miss = binom.cdf(2, 39, 0.05) ≈ 0.35 → 35%
```

### Practical P_frame estimates

| Condition | P_frame estimate | Miss rate at 2m/s, k=3 |
|---|---|---|
| Bright mine, good lighting, trained YOLO | 0.80 | ~0% |
| Medium mine, partial shadow, YOLO | 0.50 | ~0% |
| Same-colour mine, HSV only | 0.15 | ~2.5% |
| Very light mine, morning light, HSV only | 0.05 | ~35% |

**Conclusion:** At 2 m/s with YOLO running at 8–10 FPS (on RPi4 ncnn), the per-frame
detection doesn't need to be high. Even P_frame = 0.20 gives ~0.07% miss rate.

The main risk is when P_frame drops below ~0.10 — this happens only for mines that
genuinely look identical to the background.

---

## Part 5 — Overlap between adjacent passes

To avoid missing mines at pass boundaries, adjacent passes should overlap:

```
Pass 1 centre: y = 0.75 m
Pass 2 centre: y = 2.25 m
Spacing: 1.5 m
Camera GH: 1.92 m (at 2 m altitude, 49° vertical FOV)

Overlap at boundary: (1.92 - 1.5) / 2 = 0.21 m on each side
```

A mine at the boundary (y = 1.5 m) is:
- 0.21 m from the top of Pass 1's view → in Pass 1 for ~0.1 seconds at 2 m/s = 3 frames
- 0.21 m from the bottom of Pass 2's view → in Pass 2 for ~3 frames

**Boundary mines get only 3 frames per pass** (vs 39 for centre-strip mines).
With k=3 confirmation, a mine at the exact boundary needs ALL 3 boundary frames to detect.
This is achievable only if the mine is clearly visible.

**Fix:** Increase overlap. Use 1.2 m pass spacing instead of 1.5 m:
- Overlap = (1.92 - 1.2) / 2 = 0.36 m per side
- Boundary mine in view: 0.36 / 2 = 0.18 s → 5.4 frames at 30fps
- More margin for boundary detection

Cost: more passes needed. Passes per 5 m strip: 5 / 1.2 = 4.2 → round up to 5 passes.
Time per drone: 5 × 60 m / 2 m/s = 150 s → still within budget.

---

## Part 6 — Recommended flight profile

Based on the analysis:

| Parameter | Value | Justification |
|---|---|---|
| Altitude | 2.0 m | Best resolution vs coverage trade-off |
| Speed | 1.5–2.0 m/s | 39+ frames per mine, safe for RPi4 NCNN |
| Pass spacing | 1.2 m | 5-frame overlap at boundaries |
| Camera resolution | 640 × 480 | 8–10 FPS at NCNN; higher res = fewer FPS |
| Thermal sensor | D55 (narrow FOV) | 4–5 pixels per mine; better resolution |
| YOLO frame rate | 8–10 fps (RPi4) | Adequate at 2 m/s; increase at RPi5 |
| Persistence filter | 3 frames | Balances speed vs false negative risk |

### At higher altitude (2.5 m or 3.0 m)

| Advantage | Disadvantage |
|---|---|
| Wider FOV → fewer passes | Mine appears smaller in image |
| Less risky for obstacle avoidance | Thermal resolution worse (mine = 3 pixels) |
| Easier GPS-denied hover (less turbulence near ground) | Detection P_frame drops |

**Stay at 2.0 m for best detection performance.**

---

## Summary

The physics favours us:
- 39 frames per mine at 2 m/s → very low miss probability even with imperfect detection
- 2 m altitude gives excellent visual resolution (61 px mine diameter at 640×480)
- 8 fps thermal with 8 frames per mine is sufficient for 3-frame confirmation
- 1.2 m pass spacing with 1.92 m FOV gives 0.36 m boundary overlap — adequate

The weak point is mines with P_frame < 0.10 (indistinguishable from background).
Sensor fusion (thermal + visual) addresses this by providing a second independent
chance to detect the mine.
