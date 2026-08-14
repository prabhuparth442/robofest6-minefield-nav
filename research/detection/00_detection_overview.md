# Mine Detection — Complete Overview

## Start here if you're new to the detection problem

---

## Part 1 — What exactly needs to be detected

The PS describes two types of mines:

### Type 1: Surface mines (75% of all mines, ~30 of 40)

> "Rest ~75% of the 'mines' will be on-ground (terrestrial), having different
> non-reflecting opaque colours."

- Physical form: plastic discs, 20–30 cm diameter, circular (some elliptical)
- Colour: non-reflecting opaque — meaning matte plastic, no shiny surfaces
- Colour variation: "different non-reflecting opaque colours" — multiple colours,
  none predictable in advance
- On the ground surface, fully visible from above

### Type 2: Buried mines (25% of all mines, ~10 of 40)

> "Nearly 25% of the 'mines' will be buried slightly inside the earth within
> ~30–50 mm depth, i.e. beneath the soil surface. However, those won't be visible
> on the surface."
>
> "Each buried mine shall be indicated by a standardized surface marker (e.g., a
> coloured peg, painted ring, disturbed soil pattern, or predefined visual cue)
> placed directly above or adjacent to the buried mine."

- The buried mine itself is NOT directly visible
- But a surface MARKER is placed above it — you detect the marker, not the mine
- The marker is standardized but the exact type isn't disclosed before competition
- Possible forms: coloured peg, painted ring, or disturbed soil pattern

### Key implication

For Type 2 mines, you're detecting a **visual cue** (marker), not the mine itself.
This means visual detection (camera-based) is valid for ALL mines, including buried ones.
Thermal or subsurface scanning is NOT required by the PS — the marker is intentionally
surface-visible.

---

## Part 2 — Operating conditions

| Parameter | Value | Implication |
|---|---|---|
| Field | Outdoor, 15 × 60 m | Variable lighting, wind, grass/soil |
| Altitude | ~2 m AGL | Limited time over each ground patch |
| Speed | ~1–2 m/s during scan | Frame rate must match speed |
| Mine size | 20–30 cm diameter | Must be detectable at 2 m altitude |
| Ground texture | Grass / soil / sand | Background varies across the field |
| Time of day | Unknown (competition schedule) | Lighting angle varies |
| Compute | RPi 4 or 5 onboard | Limited inference budget |

**Angular size of a mine at 2 m altitude:**
- 25 cm mine at 2 m: subtends arctan(0.25/2) = 7.1° of angle
- In a 90° FOV camera (640×480), this occupies 640 × (7.1/90) ≈ 50 pixels across
- Plenty of pixels for shape detection or classification

**Time over a mine at 2 m altitude, 2 m/s:**
- Camera FOV at 2 m (90° lens): ~4 m × 3 m ground patch
- Mine in FOV for: 4 m / 2 m/s = **2 seconds** at typical scan speed
- At 30 fps: 60 frames while mine is in view
- Enough frames for confident detection even with some missed frames

---

## Part 3 — The challenge hierarchy

From easiest to hardest:

1. **Detect a brightly coloured matte disc on grass** — Easiest. Colour contrast
   against green/brown grass. HSV thresholding works.

2. **Detect a disc whose colour matches the grass** — Harder. Need shape detection
   (circle Hough) or texture difference.

3. **Detect a disc at an angle** (drone tilting in wind) — Camera perspective changes
   apparent disc shape from circle to ellipse.

4. **Detect a partially occluded disc** (tall grass edge over mine) — Partial detection
   only. Need confidence thresholding.

5. **Detect a buried mine's surface marker** — Marker may be a small peg (a few cm),
   painted ring, or just disturbed soil. Requires fine detail recognition.

6. **Detect multiple mines simultaneously in one frame** — Normal. Mine density: 40
   mines / 900 m² = 1 mine per 22 m² average. At 2 m altitude (4 m × 3 m footprint
   = 12 m²), expect 0–1 mine per frame on average.

---

## Part 4 — Scoring context

**Mine detection and correct mapping: 25 points** (highest single task in Grand Finale)

"Correct mapping" implies:
- Not just detecting mines, but placing them in the correct map position
- Position accuracy matters — a mine detected but mapped 2 m away from its actual
  position may fail to generate a correct 1 m exclusion zone

**Position accuracy requirement:**
The path requires 1 m clearance from each mine. If we map a mine 0.8 m away from its
actual position, the "safe" path may actually pass within 0.2 m of the mine. This is
dangerous and may be penalised by the TAC.

**Target detection position accuracy:** ±0.3 m or better.

This position accuracy depends on:
- Navigation position accuracy (where is the drone when it detects?)
- Camera angle accuracy (exactly where in the image is the mine?)
- Image-to-ground transform accuracy

All three feed into final mine position. See `05_mapping_and_marking.md`.

---

## Part 5 — Current detection approach (existing codebase)

From `slave/02_vision_filter.py` (legacy detection pipeline):

The existing code uses:
1. HSV colour thresholding to isolate mine-coloured regions
2. Blob detection on the thresholded image
3. Size filtering to reject objects too small or too large to be mines

This is Proposal A in `01_visual_detection.md`. It works for brightly coloured mines
but fails on mines whose colour blends with the ground.

The thermal camera (MLX90640) pipeline is in `slave/03_persistence.py`. It uses
temperature anomaly detection against the mean background temperature. It works for
buried mines (if there's a thermal signature) but struggles at certain times of day.

---

## Reading order

| File | What you'll learn |
|---|---|
| `01_visual_detection.md` | Camera-based proposals: HSV → YOLO, with benchmarks |
| `02_thermal_detection.md` | MLX90640 physics, FOV calculations, time-of-day effects |
| `03_sensor_fusion_detection.md` | How to combine visual + thermal + other sensors |
| `04_detection_altitude_speed.md` | Physics of detection: coverage math, miss probability |
| `05_mapping_and_marking.md` | After detection: how to place mines on the map accurately |

---

## Key vocabulary

| Term | Plain English |
|---|---|
| **False positive** | Detecting a mine that isn't there |
| **False negative** | Missing a mine that is there — the dangerous failure |
| **Confidence score** | Model's 0–1 certainty that a detection is real |
| **Threshold** | The minimum confidence required to record a detection |
| **HSV** | Hue-Saturation-Value: colour space useful for colour-based detection |
| **Blob** | A connected region of pixels identified as a single object |
| **Hough transform** | Algorithm to detect geometric shapes (circles, lines) in images |
| **Inference** | Running a trained neural network on an input image |
| **FOV** | Field of View — the angular cone of what a camera or sensor can see |
| **Pixel footprint** | Physical area in the real world covered by one pixel |
