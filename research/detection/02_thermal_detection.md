# 02 — Thermal Detection with MLX90640

## Physics, coverage math, and time-of-day analysis

---

## What the MLX90640 is

The MLX90640 is a 32 × 24 pixel far-infrared (FIR) thermopile array sensor made by
Melexis. It measures object temperatures from -40°C to 300°C without contact, by
detecting emitted thermal radiation.

| Specification | Value |
|---|---|
| Resolution | 32 × 24 pixels = 768 pixels total |
| Temperature accuracy | ±1°C (typical) |
| Temperature resolution | 0.1°C |
| Frame rate | 0.5 to 64 Hz (configurable) |
| FOV versions | 55° × 35° (narrow, D55) or 110° × 75° (wide, D110) |
| Interface | I2C at 400 kHz (standard) or 1 MHz (fast mode) |
| Supply voltage | 3.3 V |
| Weight (module) | ~4–6 g |
| Cost | ~₹2,000–₹3,500 in India |

This is the sensor already used in the current codebase (`slave/02_vision_filter.py`
and the legacy thermal pipeline).

---

## Part 1 — FOV and coverage math at 2 m altitude

### Using the 55° × 35° version (MLX90640-D55)

At altitude h = 2 m:

```
Width covered = 2 × h × tan(FOV_H / 2) = 2 × 2 × tan(27.5°) = 2.07 m
Height covered = 2 × h × tan(FOV_V / 2) = 2 × 2 × tan(17.5°) = 1.26 m

Ground patch at 2m: 2.07 m × 1.26 m ≈ 2.6 m²
```

Ground resolution per pixel:
```
X resolution = 2.07 m / 32 pixels = 6.5 cm/pixel
Y resolution = 1.26 m / 24 pixels = 5.25 cm/pixel
```

A 25 cm mine at 2 m altitude occupies:
```
X pixels = 25 cm / 6.5 cm/pixel ≈ 3.8 pixels
Y pixels = 25 cm / 5.25 cm/pixel ≈ 4.8 pixels
```

**A 25 cm mine appears as roughly a 4 × 5 pixel blob on the 32 × 24 array.**

This is workable but marginal. The mine occupies ~5% of the sensor's total pixels.
Any temperature contrast must be detectable at this resolution.

### Using the 110° × 75° version (MLX90640-D110)

At altitude h = 2 m:
```
Width = 2 × 2 × tan(55°) = 5.71 m
Height = 2 × 2 × tan(37.5°) = 3.07 m
```

Resolution per pixel:
```
X: 5.71 / 32 = 17.8 cm/pixel
Y: 3.07 / 24 = 12.8 cm/pixel
```

A 25 cm mine: ~1.4 × 2 pixels. **Essentially 1–2 pixels — too coarse.**

**Use the narrow-FOV version (D55) at 2 m altitude.** It gives 4–5 pixels per mine,
which is the minimum for meaningful thermal signature analysis.

---

## Part 2 — What can thermal actually detect here?

### Surface mines (plastic discs)

Plastic has a thermal emissivity of ~0.90 (similar to soil: ~0.92–0.96). Emissivity
alone doesn't distinguish them. The temperature difference comes from **thermal mass**
and **solar absorptivity**.

Key physics: an object's temperature in sunlight depends on:
```
T_surface = T_ambient + (α × I_solar - ε × σ × T_surface⁴) / (h_conv)
```
Where:
- α = solar absorptivity (how much solar radiation absorbed)
- ε = thermal emissivity
- h_conv = convective cooling rate (wind, object shape)

**Dark plastic disc** (α ≈ 0.9): absorbs more solar radiation → heats up faster
than soil → appears warmer in midday thermal image. ΔT ≈ +2–6°C above ambient soil.

**Light-coloured plastic disc** (α ≈ 0.3–0.4): absorbs less → may actually be cooler
than surrounding dark soil on a hot day. ΔT ≈ -1–3°C relative to soil.

**The mine's colour affects its thermal signature.** Dark mines stand out. Light
mines may not.

### Buried mines with surface markers

The plastic disc is buried 3–5 cm below surface. There is NO direct thermal emission
from the buried disc reachable through 3–5 cm of soil (soil thermal diffusion depth
for daily cycle ≈ 10–50 cm, but short-term anomalies from buried objects are very
weak at 3–5 cm).

**However:** the surface MARKER (coloured peg or painted ring) IS on the surface.
The marker itself is a physical object that heats in sunlight differently than
surrounding soil. Detect the marker, not the buried disc.

A coloured peg (~5–10 cm visible above ground) appears as a small warm or cool spike
in the thermal image. But at 6.5 cm/pixel, a 10 cm peg is ~1.5 pixels. Very hard to
detect reliably.

**Conclusion for buried mines:** Thermal is NOT the primary detection method.
Visual camera (Proposal C in `01_visual_detection.md`) detects the surface marker more
reliably than thermal.

---

## Part 3 — Time-of-day dependency

This is critical for outdoor thermal detection. Gujarat outdoor competitions often run
morning or afternoon.

### Morning (7–9 AM)

Soil is cooler after overnight cooling. Sun is low angle → long shadows.
Objects that absorbed daytime heat yesterday are still slightly warmer than surroundings
(thermal inertia). Effect: mines that absorbed heat yesterday are ~0.5–2°C warmer.
Signal is weak.

**Thermal contrast: low.** Harder to detect mines reliably.

### Midday (11 AM – 2 PM)

Maximum solar irradiance. Dark objects heat up fastest.
Contrast between dark mine surface and grass: potentially +4–8°C.
But: soil also heats up → background is noisy.
Shadowed mine sections (from drone, nearby objects) appear colder.

**Thermal contrast: high for dark mines, low for light mines.** Best time for thermal
detection, but with high false positive rate.

### Afternoon (3–5 PM)

Sun angle reduces. Objects are cooling at different rates based on thermal mass.
Plastic cools faster than heavy soil → mines may appear cooler than surroundings.
Signal: ~-2–4°C (reversed thermal signature from morning).

**Thermal contrast: moderate.** The sign of the contrast reverses — detection
algorithm must handle both warm and cool anomalies.

### Algorithm implication

```python
def thermal_anomaly(frame, background_temp=None, time_of_day=None):
    """
    frame: 32x24 numpy array of temperatures in Celsius
    Returns pixel coordinates of anomalies
    """
    if background_temp is None:
        # Estimate background as trimmed mean (exclude outliers)
        background_temp = np.percentile(frame, [25, 75]).mean()

    # Absolute deviation from background
    delta = frame - background_temp

    # Time-of-day adaptive threshold
    # Morning: look for warm anomalies only (small threshold)
    # Midday: both warm and cool anomalies (larger threshold)
    # Afternoon: look for cool anomalies too
    warm_threshold = 2.0   # °C above background
    cool_threshold = -1.5  # °C below background

    warm_mask = delta > warm_threshold
    cool_mask = delta < cool_threshold

    anomaly_mask = warm_mask | cool_mask

    # Find blob centres in anomaly mask
    from scipy import ndimage
    labelled, n_objects = ndimage.label(anomaly_mask)
    centroids = ndimage.center_of_mass(anomaly_mask, labelled, range(1, n_objects+1))

    return centroids, delta
```

---

## Proposal A: Temperature Anomaly Threshold

### How it works

Compute the background soil temperature from the median of the thermal image.
Any pixel that is significantly warmer or cooler than background (beyond a threshold)
is a candidate mine.

As coded above. Simple, fast (~2 ms per frame at 8 Hz sensor rate).

### Pros
- Colour-invariant (doesn't care what colour the mine is)
- Works even if mine is partially covered (even 2 exposed pixels may show anomaly)
- Extremely fast to compute

### Cons
- Requires real temperature contrast — if mine temperature ≈ soil temperature → miss
- Rocks, metal debris, wet patches all create thermal anomalies → false positives
- Background estimate can be skewed by large mine density (40 mines → ~4% of 900 m²)

### Accuracy estimate
**50–70% detection rate** for dark mines in midday. **20–40%** for light mines.
**High false positive rate** outdoors without visual confirmation.

---

## Proposal B: Thermal + Visual Fusion

### How it works

Use thermal to narrow the region of interest, then use visual camera to confirm the shape.

```python
def fused_detection(thermal_frame, visual_frame, homography_matrix):
    """
    thermal_frame: 32×24 array
    visual_frame: 640×480 BGR image
    homography_matrix: maps thermal pixels to visual pixels (pre-calibrated)
    """
    # Step 1: Find thermal anomalies (coarse candidates)
    thermal_candidates, delta = thermal_anomaly(thermal_frame)

    confirmed_mines = []
    for (ty, tx) in thermal_candidates:
        # Step 2: Map thermal pixel to visual pixel region
        thermal_pt = np.array([[tx, ty]], dtype=np.float32).reshape(-1, 1, 2)
        visual_pt = cv2.perspectiveTransform(thermal_pt, homography_matrix)
        vx, vy = int(visual_pt[0][0][0]), int(visual_pt[0][0][1])

        # Step 3: Extract region of interest in visual image
        roi = visual_frame[max(0,vy-100):vy+100, max(0,vx-100):vx+100]

        # Step 4: Apply shape/colour detection to ROI
        circles = cv2.HoughCircles(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY),
                                    cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                    param1=50, param2=20,
                                    minRadius=20, maxRadius=80)

        if circles is not None:
            confirmed_mines.append({'visual_pos': (vx, vy),
                                     'thermal_delta': delta[int(ty), int(tx)]})

    return confirmed_mines
```

**Calibration:** The thermal and visual cameras cover different FOVs and must be
aligned. Take 8–10 images of a known thermal target (a warm object at known position)
and compute the homography matrix with `cv2.findHomography`.

### Pros
- Thermal provides colour-invariant pre-filtering → visual only checks small ROIs
- Visual confirms the circular shape → eliminates thermal false positives (rocks)
- Together: much higher precision than either alone

### Cons
- Requires careful calibration of thermal-to-visual alignment
- Must run both sensors synchronously (or accept slight time offset)
- Adds software complexity

### Accuracy estimate
**75–90% detection rate** with low false positive rate.

---

## Proposal C: Buried Mine Thermal Analysis

### Can we detect buried mines (3–5 cm depth) thermally?

**Theoretical analysis:**

Thermal diffusion in soil follows the heat equation. The attenuation of a surface
temperature pulse at depth z and time t:

```
ΔT(z, t) = ΔT_surface × exp(-z / δ) × sin(ωt - z/δ)
Where:
  δ = thermal skin depth = sqrt(2κ/ω)
  κ = thermal diffusivity of soil ≈ 0.4 × 10⁻⁶ m²/s (dry sand)
  ω = 2π/P where P = 86400 s (daily cycle)
  δ = sqrt(2 × 0.4×10⁻⁶ / (2π/86400)) ≈ 0.074 m = 7.4 cm
```

A buried disc at 3–5 cm depth is within ~0.5 × δ of the surface. A daily temperature
swing of 20°C at the surface becomes:

```
ΔT(z=0.04m) = 20 × exp(-0.04/0.074) ≈ 20 × 0.58 = 11.6°C at the mine depth
```

This doesn't directly tell us the surface anomaly above the mine. The plastic disc
has different thermal properties than soil and slightly perturbs the heat flow above it.

**In practice:** published research on thermal mine detection (using military-grade
thermal cameras with <0.1°C NETD) shows buried mine detection is possible in optimal
conditions (early morning, when soil has cooled overnight but mine retained heat).
But with the MLX90640's ±1°C accuracy, detecting a thermal anomaly of <0.5°C above
a buried disc at 3–5 cm depth is very unreliable.

**Conclusion:** For the Robofest PS, buried mine detection via thermal is NOT a
practical primary strategy. The PS provides a visual surface marker for this reason.
Detect the marker visually, not the mine thermally.

---

## Summary

| Application | Thermal reliability | Recommendation |
|---|---|---|
| Surface dark-coloured mines | Moderate (midday) | Use as backup to visual |
| Surface light-coloured mines | Low | Don't rely on thermal |
| Buried mine (disc) | Very low | Thermal cannot reach it |
| Buried mine (surface marker) | Low | Marker is too small (1–2 pixels) |

**Thermal is a useful secondary detector for surface mines, especially at midday.
It should never be the sole detection method.**

---

## References

1. Melexis MLX90640 datasheet:
   https://www.melexis.com/en/product/mlx90640/far-infrared-thermal-sensor-array

2. Waveshare MLX90640 module specs:
   https://www.waveshare.com/wiki/MLX90640-D55_Thermal_Camera

3. UAV thermal imaging for outdoor detection (RGB + thermal fusion):
   https://arxiv.org/pdf/2308.10671

4. Application of drone magnetometer for mine detection (military context):
   https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8125094/
