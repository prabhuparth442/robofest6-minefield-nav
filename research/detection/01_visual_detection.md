# 01 — Visual (Camera) Mine Detection

## Five proposals from classical to neural network, with real benchmarks

---

## Camera setup baseline

**Assumed hardware:** Raspberry Pi Camera Module 3 (Sony IMX708)
- Resolution: 12 MP (4608 × 2592), used at 1920 × 1080 or 640 × 480
- FOV: 66° horizontal (wide angle version: 120°)
- Frame rate: 30 fps at 1080p, up to 120 fps at 640 × 480
- Weight: ~6 g
- Interface: MIPI CSI-2 (direct RPi connection)

**Altitude:** 2.0 m AGL  
**Sensor FOV at ground:** 66° → ground width = 2 × 2 × tan(33°) = **2.6 m**  
**Ground resolution at 1920 × 1080:** 2.6 m / 1920 = **1.35 mm/pixel**  
**25 cm mine occupies:** 250 mm / 1.35 mm/px ≈ **185 pixels** across → huge, easily detectable

If using a wide-angle lens (120° FOV):  
**Ground width:** 2 × 2 × tan(60°) = **6.9 m** → at 3 passes per drone, covers 5 m strip in 2 passes  
**Resolution:** 6.9 m / 1920 px = 3.6 mm/pixel. Mine = 70 pixels → still detectable.

---

## Proposal A: Classical CV — HSV Thresholding + Blob Detection

### How it works

The simplest approach: convert each frame to HSV colour space and threshold to isolate
mine-coloured pixels. Connected components (blobs) in the thresholded image are
candidate mines. Filter blobs by size and circularity.

```python
import cv2
import numpy as np

def detect_mines_hsv(frame, color_profiles):
    """
    color_profiles: list of (lower_hsv, upper_hsv) tuples for each known mine color
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in color_profiles:
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        combined_mask = cv2.bitwise_or(combined_mask, mask)

    # Morphological cleanup
    kernel = np.ones((5, 5), np.uint8)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

    # Find blobs
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # At 2m alt, 1.35mm/px: 25cm mine = 185px wide → area ~ π*(92.5)² ≈ 26,900 px²
        # Accept mines 15–40 cm diameter
        if 10000 < area < 100000:
            perimeter = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity > 0.7:  # circles have circularity = 1.0
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                detections.append({'pixel': (cx, cy), 'area': area, 'method': 'hsv'})

    return detections
```

### Hardware required
Raspberry Pi Camera Module 3 only. No extra hardware.

### Inference time on RPi4
Pure Python + OpenCV: **~5–15 ms per frame at 640×480** → effectively real-time.

### Accuracy estimate
**Good (70–85%)** for mines with distinct colours (red, yellow, blue) against green grass.
**Poor (20–40%)** for mines that match ground colour (brown disc on brown soil).
**Zero** for buried mine markers (unless they're brightly coloured pegs).

### Training data needed
None — algorithm is rule-based. But colour profiles must be calibrated for the specific
mine colours used at this competition.

**Problem:** The PS says mine colours are undisclosed until competition day. You can't
pre-calibrate for unknown colours.

### Failure modes
1. Mine colour similar to grass → blends in, not detected
2. Direct sunlight creates glare on surrounding surface → false positives
3. Drone shadow falls on mines → apparent colour changes
4. Late afternoon sunlight at low angle → entire image colour-shifted

### Verdict
Use as a fast pre-filter. Not sufficient as sole detector. Run in parallel with
Proposal B (shape detection) and Proposal C (YOLO) for redundancy.

---

## Proposal B: Shape Detection — Hough Circle Transform

### How it works

Rather than looking for colour, detect the **circular geometry** of the disc.
Hough Circle Transform finds all circular structures in an image, regardless of colour.

```python
def detect_circles_hough(frame, altitude_m=2.0):
    """
    altitude_m: current altitude for computing expected mine radius in pixels
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Expected mine radius in pixels at given altitude
    # FOV = 66°, image width = 1920px → scale = 2*tan(33°)*altitude / 1920
    px_per_meter = 1920 / (2 * 2.0 * np.tan(np.radians(33)))  # ~740 px/m at 2m
    min_r = int(0.10 * px_per_meter)  # 10 cm radius → ~74 px
    max_r = int(0.15 * px_per_meter)  # 15 cm radius → ~111 px

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1,           # inverse ratio of accumulator resolution
        minDist=200,    # minimum distance between circles (px)
        param1=50,      # upper Canny threshold
        param2=30,      # accumulator threshold — lower = more detections
        minRadius=min_r,
        maxRadius=max_r
    )

    detections = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for c in circles[0]:
            detections.append({
                'pixel': (c[0], c[1]),
                'radius_px': c[2],
                'method': 'hough'
            })

    return detections
```

### Hardware required
Raspberry Pi Camera + OpenCV (included in RPi OS).

### Inference time
**~20–50 ms per frame** at 640×480. Still real-time.

### Accuracy estimate
**Good (60–80%)** for circular mines on textured ground.
**Poor** on elliptical mines (PS says "a few can be elliptical").
**Moderate (50–60%)** for partially visible mines at frame edges.

### Advantage over HSV
**Colour-invariant.** Detects brown mines on brown soil IF the mine has distinct
circular edges. Works on any colour mine.

### Failure modes
1. Grass blades or shadows create noise → spurious circles detected
2. Elliptical mines → Hough circle misses them (need Hough ellipse — much slower)
3. Mine partially covered by grass → circle is incomplete → detection fails
4. Overlapping shadows from multiple objects → creates false circular arcs

### Verdict
Combine with HSV as complementary — where HSV fails (same-coloured mine), Hough
may succeed. Where Hough fails (ellipse), YOLO handles it.

---

## Proposal C: YOLOv8-nano — Trained on Mine Images

### What YOLO is

YOLO (You Only Look Once) is a neural network that detects objects in images in a single
forward pass. YOLOv8n (nano) is the smallest variant — fast and light.

Architecture: CSPDarknet backbone → C2f neck → Detect head  
Weights size: ~6.3 MB (fp32) or ~3.4 MB (int8 quantized)  
Classes: we train on 2 classes: `surface_mine`, `buried_marker`

### Benchmark data (real measured numbers)

**YOLOv8n on Raspberry Pi 4B (CPU, PyTorch):**
~1 FPS at 640×640 resolution — too slow.

**YOLOv8n on RPi 4B with NCNN (int8 quantized):**
~8–10 FPS at 320×320 — usable for 2 m/s scan speed.

**YOLOv8n on RPi 5 with NCNN:**
~30–40 FPS at 320×320 — excellent.

**YOLOv8n with Google Coral USB TPU (Edge TPU quantized):**
~200 FPS at 320×320 — extremely fast, but requires specific edge-tpu-compiled model.

Source: Ultralytics official benchmarks, Qengineering RPi4 YOLOv8-ncnn tests, MDPI 2025
benchmark paper (Benchmarking YOLOv8–YOLOv12 on Single-Board Computers).

### Does 8–10 FPS work for our scan speed?

At 2 m/s scan speed, ground under drone moves at 2 m/s.  
At 10 FPS: one frame every 0.1 s → drone moves 0.2 m between frames.  
With 2.6 m wide FOV: a mine spends ~1.3 s in frame = **13 frames** with it visible.  
Even if we only detect it in 5 of those 13 frames, confidence is high.

**YES — 8–10 FPS is adequate for 2 m/s scan.**

For >2 m/s scans, use the RPi 5 or Coral TPU.

### Training data

The PS doesn't disclose exact mine colours. You need a dataset that generalises across
colours and textures.

**Strategy:**
1. Use synthetic data: render plastic discs in Blender at varying colours, angles,
   altitudes, lighting conditions (2,000–5,000 images)
2. Use real data: photograph physical plastic plates/discs on grass outdoors at 1–3 m
   distance (500–1,000 images)
3. Augment: random colour shifts, rotation, perspective distortion, blur

**Minimum viable dataset:** 1,000 images with bounding box labels. Use Roboflow
(free tier) for labelling and augmentation.

```python
# Training command
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
results = model.train(
    data='mine_dataset/data.yaml',
    epochs=100,
    imgsz=320,
    batch=32,
    augment=True,
    hsv_h=0.3,     # strong hue augmentation (random mine colours)
    hsv_s=0.5,
    degrees=180,   # full rotation invariance (mines are circular)
    flipud=0.5,
    fliplr=0.5,
    perspective=0.001,  # slight perspective distortion (altitude variation)
)
```

### Export to NCNN for RPi 4 deployment

```bash
# Export to NCNN
yolo export model=mine_yolov8n.pt format=ncnn imgsz=320

# This creates:
#   mine_yolov8n_ncnn_model/model.ncnn.param
#   mine_yolov8n_ncnn_model/model.ncnn.bin
```

```python
# Runtime inference with NCNN on RPi
import ncnn

def load_ncnn_model(param_path, bin_path):
    net = ncnn.Net()
    net.load_param(param_path)
    net.load_model(bin_path)
    return net

def infer_ncnn(net, frame):
    mat_in = ncnn.Mat.from_pixels_resize(frame, ncnn.Mat.PixelType.PIXEL_BGR,
                                          320, 320)
    ex = net.create_extractor()
    ex.input('images', mat_in)
    _, mat_out = ex.extract('output0')
    return parse_yolo_output(mat_out)
```

### Accuracy estimate
**High (85–95%)** for trained classes, given diverse training data.
Degrades with out-of-distribution colours not seen during training.

### Failure modes
1. Training data doesn't include the specific colours used at competition → misses mines
2. At 320×320, a 25 cm mine at 2 m altitude = ~185px / (1920/320) = ~31 px → marginal
   (Use 640×640 if RPi5 or Coral — gives 62 px for mine)
3. Motion blur at scan speed >3 m/s → blurred mine detection fails
4. Drone vibration → blurry frames → detection drops

### Verdict
**Primary detector for Grand Finale.** Train aggressively with colour augmentation.
Deploy at 320×320 with NCNN on RPi 4, or 640×640 with NCNN on RPi 5.

---

## Proposal D: YOLOv8n + Depth (Stereo Camera)

### How it works

Add a stereo camera pair (e.g., OAK-D Lite, 38g) to get depth estimates. Use depth
to:
1. Reject false positives above ground level (plants, poles)
2. More accurately convert pixel coordinates to world coordinates (depth-assisted backprojection)

**OAK-D Lite specs:**
- Weight: 38 g (including housing)
- Depth range: 0.08–35 m
- Compute: built-in MyriadX VPU for inference
- Can run YOLOv8n onboard at 30+ FPS without RPi

### Pros
- Depth eliminates most false positives (a plant 30 cm tall is NOT at ground level)
- Accurate 3D position of mine without needing separate altitude sensor
- Onboard VPU offloads inference from RPi

### Cons
- Weight: 38 g is significant for a 750 g AUW budget
- Outdoor depth estimation in bright sunlight: stereo struggles (structured light
  and active IR don't work well in direct sun)
- Passive stereo (what OAK-D uses) works in sunlight but has ~2 cm error at 2 m range

### Verdict
Investigate if weight budget allows. The OAK-D Lite is attractive for its onboard
inference, but test depth accuracy on your specific outdoor field first.

---

## Proposal E: Template Matching with Rotational Invariance

### How it works

Pre-build templates of mine appearances at different colours and take the best matching
region of each frame.

```python
def rotational_template_match(frame, template, scales):
    """
    template: reference image of a mine
    scales: list of scale factors to account for altitude variation
    """
    best_val, best_loc = -1, None
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_tmpl = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

    # Try multiple rotations (mines can be at any angle — but circles are rotation-invariant)
    for scale in scales:
        resized_tmpl = cv2.resize(gray_tmpl, None, fx=scale, fy=scale)
        result = cv2.matchTemplate(gray_frame, resized_tmpl, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if max_val > best_val:
            best_val = max_val
            best_loc = max_loc

    return best_loc if best_val > 0.7 else None
```

### Problems
- Template must match the specific mine's colour — you don't know colours in advance
- Template matching is O(W × H × template_W × template_H) — slow on large images
- Robust rotation invariance requires many templates (0–360° if discs are not true circles)

### Verdict
**Don't use as primary.** Too brittle to unknown colours. Use only as a last resort
if YOLO fails and you know the exact mine colour from TAC pre-inspection.

---

## Recommended pipeline

```
Frame acquisition (30 fps, 640×480)
         │
         ├──→ HSV thresholding [5ms] → colour candidates
         │
         ├──→ Hough circles [30ms] → shape candidates
         │
         └──→ YOLOv8n-ncnn [100ms at 320×320] → learned candidates

         Fusion: take union of all detections, weighted by confidence
         │
         └──→ Persistence filter [03_persistence.py]:
               Only record detections confirmed in ≥3 frames within 1s window
               (eliminates single-frame noise)
```

The three detectors run in parallel (separate threads on RPi's 4 cores).
The fusion step takes the union weighted by confidence per method.

---

## References

1. YOLOv8–YOLOv12 benchmark on single-board computers (MDPI 2025):
   https://www.mdpi.com/2504-4990/8/7/204

2. YOLOv8 ncnn Raspberry Pi 4/5 (Qengineering):
   https://github.com/Qengineering/YoloV8-ncnn-Raspberry-Pi-4

3. Ultralytics YOLOv8 on Raspberry Pi guide:
   https://docs.ultralytics.com/guides/raspberry-pi

4. Edge deployment benchmarks (arxiv 2024):
   https://arxiv.org/html/2409.16808v1

5. YOLOv4-Tiny INT8 on RPi5 — 45 FPS reference:
   https://arxiv.org/pdf/2506.09300
