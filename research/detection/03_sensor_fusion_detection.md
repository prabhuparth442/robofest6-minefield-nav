# 03 — Sensor Fusion for Detection

## Combining multiple sensors to improve reliability

---

## Why fusion matters

No single sensor is good enough for all mine types in all conditions:

| Sensor | Strong on | Weak on |
|---|---|---|
| Visual (HSV) | Colourful mines, bright conditions | Same-colour mines, shadows |
| Visual (YOLO) | All mine shapes and colours (if trained) | Novel colours not in training data |
| Thermal (MLX90640) | Dark mines at midday | Light mines, buried markers, morning |
| Hough circles | All colours, shape-based | Grass noise, elliptical mines |

Fusion means: run multiple sensors in parallel and combine their outputs. A mine
confirmed by two independent sensors is much more reliably detected than a mine
from one sensor alone.

---

## Proposal A: Visual Primary + Thermal Secondary (Confirm-then-Classify)

### Architecture

```
Visual detection pipeline (YOLO + HSV)
         │
         ├──→ Detects candidate at pixel (cx, cy) with confidence 0.65
         │
         │    Is confidence ≥ HIGH_THRESHOLD (0.85)?
         ├──→ YES: Record as confirmed mine immediately
         │
         │    Is confidence MEDIUM (0.5–0.85)?
         └──→ CHECK THERMAL:
               Map visual pixel → thermal pixel (homography)
               Is there a thermal anomaly (|ΔT| > 1.5°C) at that position?
               YES: Confirm mine. NO: Reject as false positive.
```

This is a **cascade architecture**:
- High-confidence visual detections pass immediately (fast path)
- Medium-confidence visual detections need thermal confirmation (slow path)
- Low-confidence visual detections (<0.5) are rejected without thermal check

```python
HIGH_CONF = 0.85
MED_CONF = 0.50
THERMAL_CONFIRM_DELTA = 1.5  # °C

def fused_decision(visual_detections, thermal_frame, visual_to_thermal_H):
    confirmed = []
    for det in visual_detections:
        conf = det['confidence']

        if conf >= HIGH_CONF:
            confirmed.append({**det, 'source': 'visual_high'})

        elif conf >= MED_CONF:
            # Check thermal at corresponding position
            t_pixel = map_visual_to_thermal(det['pixel'], visual_to_thermal_H)
            ty, tx = int(t_pixel[1]), int(t_pixel[0])

            if 0 <= ty < 24 and 0 <= tx < 32:
                background = np.median(thermal_frame)
                delta = thermal_frame[ty, tx] - background

                if abs(delta) >= THERMAL_CONFIRM_DELTA:
                    confirmed.append({**det,
                                       'source': 'visual_thermal_confirmed',
                                       'thermal_delta': delta})
                # else: reject medium-confidence detection

    return confirmed
```

### When thermal isn't available (morning, overcast)

Fall back to a higher visual confidence threshold alone:

```python
if thermal_available():
    threshold = MED_CONF  # 0.50 — thermal confirms ambiguous cases
else:
    threshold = HIGH_CONF  # 0.85 — only high-confidence visual passes
```

### Pros
- Reduces false positives significantly
- Thermal confirmation costs <2 ms (32×24 array, one lookup)
- Gracefully degrades if thermal sensor fails

### Cons
- Misses mines where visual confidence is medium AND thermal has no contrast
  (this is the hardest case: same-colour mine in morning conditions)
- Requires calibrated homography between visual and thermal FOVs

---

## Proposal B: Magnetometer Sweep

### Is this relevant for plastic mines?

The PS explicitly says mines are plastic. Plastic has no magnetic signature.
**A magnetometer cannot detect the mines themselves.**

However, the buried mine's containment (if any) might have metal — but the PS says
"plastic discs." The surface markers might be plastic pegs. No metal component
mentioned.

**Conclusion: Magnetometer is irrelevant for this specific PS.** Do not add weight
for a magnetometer.

For reference: the military drone magnetometer systems used for real-world demining
(e.g., the Springer Nature 2021 paper on DMZ mine detection) target metallic anti-
personnel mines. The Robofest plastic discs are specifically designed to avoid thermal
and magnetic signatures to make the problem a pure vision challenge.

---

## Proposal C: Ground-Penetrating Radar (GPR)

### What GPR is

GPR sends microwave pulses (100 MHz – 2.6 GHz) into the ground and detects reflections
from buried objects. Military drones have used GPR for actual landmine detection.

### Weight analysis for Robofest 6.0

The smallest airborne GPR systems commercially available:

| System | Weight | Frequency | Detection depth |
|---|---|---|---|
| IDS GeoRadar (multi-drone) | ~2 kg antenna + ~1 kg electronics | 400 MHz – 2.6 GHz | 0–30 cm |
| Custom lightweight GPR (research) | ~500 g | 1 GHz | 0–20 cm |
| Minimal academic GPR (single antenna) | ~200 g | 2.4 GHz | 0–10 cm |

Our AUW limit: 750 g per drone. RPi + SpeedyBee + props + frame + battery ≈ 600–650 g.
**No weight budget for even the lightest GPR system.**

Additionally, GPR at 2 m altitude has a footprint of ~1 m × 1 m per pulse — adequate
for 3–5 cm buried mines. But at 2 m altitude with 2 GHz frequency, penetration depth
is ~5–10 cm in dry sand — barely enough for 3–5 cm buried mines.

**Conclusion: GPR is ruled out by weight constraints.** 750 g AUW makes any meaningful
GPR impossible without exceeding the limit. The PS designers knew this.

---

## Proposal D: Acoustic Detection of Buried Mines

### Theory

A buried object changes ground resonance when mechanically excited. If a drone emits
a sound wave (or generates ground vibration through landing gear contact), a buried
object responds differently than undisturbed soil.

**Problems:**
1. Drone must fly close to ground (10–20 cm) to generate detectable acoustic excitation
2. Propeller noise (80–100 dB at drone level) overwhelms any acoustic return signal
3. Requires ground contact or near-contact — flight at 2 m altitude makes this impossible
4. Processing acoustic returns while flying in wind is research-grade work

**Conclusion: Not feasible for a flying platform.** Acoustic mine detection requires
contact or near-contact systems. Ruled out.

---

## Fusion architectures

### Early fusion (sensor-level)

Combine raw sensor outputs before classification:

```
Visual pixels (640×480×3) ─┐
Thermal pixels (32×24×1) ──┤ → Concatenated tensor → Single neural network → Mine/no mine
```

**Problem:** Thermal and visual have very different resolutions. Upscaling thermal
from 32×24 to 640×480 loses all spatial information. **Not recommended.**

### Late fusion (decision-level)

Each sensor produces its own detection list. Combine the lists:

```
Visual detector → [(x1,y1,conf1), (x2,y2,conf2), ...]  ─┐
Thermal detector → [(xa,ya,confA), ...]  ─────────────────┤ → Merge by proximity
                                                            └→ Unified mine list
```

This is Proposal A above. **Recommended** — easy to implement, each sensor operates
independently.

### Middle fusion (feature-level)

Visual CNN extracts features from the penultimate layer. Thermal processing extracts
a feature vector. Concatenate, pass to a small classifier head.

```python
class FusedDetector(nn.Module):
    def __init__(self):
        self.visual_backbone = YOLOv8nBackbone()
        self.thermal_encoder = nn.Sequential(
            nn.Conv2d(1, 8, 3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4))  # compress 32x24 to 4x4
        )
        self.fusion_head = nn.Linear(256 + 8*4*4, 2)  # 2 classes

    def forward(self, visual, thermal):
        vf = self.visual_backbone(visual)     # (B, 256)
        tf = self.thermal_encoder(thermal)    # (B, 8, 4, 4)
        tf = tf.flatten(1)                    # (B, 128)
        fused = torch.cat([vf, tf], dim=1)   # (B, 384)
        return self.fusion_head(fused)        # (B, 2)
```

**Pros:** Learns optimal fusion weights.
**Cons:** Requires paired training data (visual + thermal of same mines simultaneously).
Harder to debug. **Advanced — implement only if time allows.**

---

## Recommended fusion stack

For competition:

```
Priority 1 (always): YOLOv8n visual detection (primary)
Priority 2 (if thermal available): Thermal anomaly confirmation of medium-confidence
Priority 3 (always): Hough circle detection on the same frame (secondary, fast)

Merge rule:
  - Any detection with conf > 0.85 from YOLO → confirmed mine
  - Any detection conf 0.5–0.85 confirmed by thermal → confirmed mine
  - Any Hough circle with no YOLO match → add with conf = 0.5 (may be real mine)
  - Deduplicate: merge detections within 0.3 m of each other
```

---

## Persistence filter (critical)

Single-frame detections are noisy. A persistent detection across multiple frames is
much more reliable. The existing `slave/03_persistence.py` implements this:

```python
class PersistenceFilter:
    def __init__(self, confirm_frames=3, timeout_frames=10, distance_threshold=0.3):
        self.candidates = {}  # candidate_id → list of (position, timestamp, confidence)

    def update(self, detections, timestamp):
        confirmed = []
        for det in detections:
            matched = self._find_matching_candidate(det['position'])

            if matched is not None:
                self.candidates[matched].append(det)
                if len(self.candidates[matched]) >= self.confirm_frames:
                    best = max(self.candidates[matched], key=lambda d: d['confidence'])
                    confirmed.append(best)
            else:
                new_id = len(self.candidates)
                self.candidates[new_id] = [det]

        self._expire_old_candidates(timestamp)
        return confirmed
```

With 30 fps video and 3-frame confirmation: a mine is only recorded once it appears
in 3 consecutive frames. At 2 m/s, 3 frames = 0.1 seconds = 0.2 m of travel. Mine
in FOV for ~1.3 s = 39 frames. **Persistence filter easily confirms real mines and
rejects noise.**

---

## References

1. RGB + thermal fusion for UAV outdoor surveillance:
   https://arxiv.org/pdf/2308.10671

2. Human detection in UAV thermal imagery (embedded platform benchmarks):
   https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12733663/

3. Drone magnetometer for mine detection (DMZ application):
   https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8125094/
