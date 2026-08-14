# 02 — Onboard Compute: What Runs Where

## Comparing RPi 4, RPi 5, Coral TPU, and Jetson Nano

---

## The compute budget

Each drone is a 750 g AUW system. Our baseline frame (SpeedyBee F405 + motor + props +
battery + ESC) weighs approximately:

| Component | Weight |
|---|---|
| SpeedyBee F405 Wing V3 FC | ~12 g |
| 4× BLDC motors (e.g. 2306 2400Kv) | 4 × ~30 g = 120 g |
| 4× ESCs (integrated or separate) | ~20–40 g |
| Frame (carbon fibre quad) | ~80–120 g |
| Props (5" set × 4) | ~20 g |
| Battery (4S 2200 mAh LiPo) | ~170 g |
| Wiring + connectors | ~20 g |
| **Subtotal (flight hardware)** | **~442–522 g** |

Remaining for mission payload (RPi + camera + thermal + sensors):
```
750 - 522 = 228 g (conservative)
750 - 442 = 308 g (optimistic)
```

Target compute budget: **~150–200 g**

---

## Proposal A: Raspberry Pi 4B (4 GB RAM)

### Specs

| Spec | Value |
|---|---|
| CPU | Broadcom BCM2711, quad-core Cortex-A72 @ 1.8 GHz |
| RAM | 4 GB LPDDR4-3200 |
| Weight | ~46 g (board only) |
| With heatsink + case | ~70–80 g |
| Idle power | ~2.8 W |
| Full load power | ~6–7 W |
| Storage | microSD or USB SSD |
| Cost in India | ~₹4,500–6,000 |

### YOLOv8n inference performance

| Backend | Resolution | FPS |
|---|---|---|
| PyTorch (CPU, fp32) | 640×640 | ~1 FPS |
| ONNX Runtime (CPU) | 640×640 | ~2 FPS |
| ncnn (int8, 4 threads) | 320×320 | **8–10 FPS** |
| ncnn (int8, 4 threads) | 640×640 | ~3–4 FPS |

**The recommended config: ncnn int8 at 320×320 → 8–10 FPS.**
This is sufficient for 2 m/s scan speed (39 frames per mine at 30fps camera, 10-12
YOLO frames per mine at 10 Hz YOLO).

### Can RPi 4 handle everything?

| Task | Can RPi 4 handle? | Notes |
|---|---|---|
| YOLOv8n at 8–10 FPS | ✓ Yes | ncnn int8 |
| OpenCV detection (HSV, Hough) | ✓ Yes | <5 ms per frame |
| MLX90640 thermal @ 8 Hz | ✓ Yes | Simple I2C read |
| pymavlink @ 10–50 Hz | ✓ Yes | Serial, <1 ms per send |
| ROS 2 node management | ✓ Yes | Humble on Ubuntu 22.04 arm64 |
| WiFi ad-hoc networking | ✓ Yes | Built-in WiFi chip |
| A* pathfinding (14K nodes) | ✓ Yes | ~50 ms, runs once |
| Three-node swarm aggregation | ✓ Yes | UDP + merge |

**Verdict: RPi 4B (4 GB) handles everything at our operating speeds.**

### Power and flight time impact

Assuming 4S 2200 mAh = 32.5 Wh at 3.7 V average:
- Total drone power draw in flight: ~120–150 W (motors at ~50% throttle hover)
- RPi 4B adds ~6 W at full load

Flight time reduction from RPi 4:
```
Base flight time (motors only): ~10–12 minutes
With RPi 4B (6 W addition): ~10–11 minutes (≈5% reduction)
```

**Negligible impact on flight time.**

---

## Proposal B: Raspberry Pi 5 (8 GB RAM)

### Specs

| Spec | Value |
|---|---|
| CPU | Broadcom BCM2712, quad-core Cortex-A76 @ 2.4 GHz |
| RAM | 8 GB LPDDR4X-4267 |
| Weight | ~51 g (board only) |
| With heatsink + case | ~80–90 g |
| Idle power | ~3 W |
| Full load power | ~9–10 W |
| Cost in India | ~₹7,500–10,000 |

### YOLOv8n inference performance

| Backend | Resolution | FPS |
|---|---|---|
| ncnn (int8, 4 threads) | 320×320 | **25–35 FPS** |
| ncnn (int8, 4 threads) | 640×640 | **10–15 FPS** |
| TFLite (int8) | 320×320 | ~20 FPS |

With RPi 5, you can run YOLOv8n at **640×640 resolution at 10–15 FPS** — this gives
mine pixel size of ~122 pixels (vs 61 pixels on 320×320). Detection accuracy improves
significantly for small or low-contrast mines.

### Should you use RPi 5?

**Yes, if you can find one in India and it fits the weight budget.**

The performance jump is significant: 3× faster inference enables higher-resolution
detection or running detection + thermal fusion simultaneously without frame drops.

**Weight difference:** only 5 g more than RPi 4. Cost difference: ~₹3,000–4,000 more.
Well worth it for the detection accuracy improvement.

---

## Proposal C: Raspberry Pi 4 + Google Coral USB TPU

### Coral USB Accelerator specs

| Spec | Value |
|---|---|
| Compute | 4 TOPS (Edge TPU ASIC) |
| Power | **2 W** at peak |
| Interface | USB 3.0 (requires USB 3 for full speed; RPi 4 has USB 3) |
| Weight | ~30 g (device) + ~8 g (cable) |
| Size | 65 × 30 × 8 mm |
| Cost in India | ~₹3,500–5,000 |

### Performance with Coral

The Edge TPU requires specially compiled models (edge-tpu compilation via Coral SDK).
Not every YOLO operation is supported — quantization must be done in a specific way.

For YOLOv8n on Coral (compiled for Edge TPU):
- Theoretical: up to 400 FPS for MobileNet v2 equivalent workloads
- Practical for YOLOv8n with partial delegation: **~60–120 FPS at 320×320**

**Why "partial delegation":** Some YOLOv8 layers (custom attention, certain NMS
operations) don't map to Edge TPU operations and must run on CPU. The percentage
of layers on TPU vs CPU depends on your model architecture.

YOLOv8n-based models with Coral get approximately:
- Full Edge TPU delegation: ~80–120 FPS (if all ops are supported)
- Partial delegation (typical): ~20–40 FPS (unsupported ops fall back to CPU)

### Setup complexity

```bash
# Install Coral Edge TPU runtime
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | \
  sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
sudo apt-get update
sudo apt-get install libedgetpu1-std

# Compile model for Edge TPU (requires TFLite int8 first)
yolo export model=mine_yolov8n.pt format=tflite int8=True
edgetpu_compiler mine_yolov8n_int8.tflite
```

### Weight budget

| Component | Weight |
|---|---|
| RPi 4B + heatsink | ~70 g |
| Coral USB (device only) | ~30 g |
| USB cable | ~8 g |
| RPi Camera Module 3 | ~6 g |
| MLX90640 module | ~6 g |
| Matek 3901-L0X | ~5 g |
| Miscellaneous wiring | ~15 g |
| **Total payload** | **~140 g** |

140 g payload + 522 g flight hardware = **662 g AUW** — within 750 g limit.

### Verdict

The Coral TPU is compelling for maximum detection performance on a weight-constrained
platform. However, the model compilation requirement adds complexity, and partial
delegation may reduce gains.

**Recommended:** If using RPi 4B, add Coral. If using RPi 5, skip Coral (Pi5 is
fast enough without it at 640×640 NCNN).

---

## Proposal D: Raspberry Pi 4 + Jetson Nano

### Jetson Nano specs

| Spec | Value |
|---|---|
| GPU | 128-core Maxwell GPU |
| CPU | Quad-core Cortex-A57 @ 1.43 GHz |
| RAM | 4 GB LPDDR4 |
| Weight | **136 g** (2GB module: 70 g; 4GB: 136 g) |
| Power | 5–10 W |
| Cost in India | ~₹15,000–25,000 |
| YOLOv8n on GPU | ~60–100 FPS at 640×640 |

### Weight analysis

Jetson Nano (4 GB) alone: **136 g**. This is already most of our compute budget.
Adding RPi 4B makes no sense — the Jetson Nano replaces the RPi as companion computer.

Using Jetson Nano as sole compute:

| Component | Weight |
|---|---|
| Jetson Nano 4GB | 136 g |
| Camera + thermal + sensors | ~17 g |
| Wiring | ~15 g |
| **Total payload** | **~168 g** |

168 g + 522 g flight = 690 g — within 750 g limit, but only 60 g margin.

### Problems

1. **Cost:** ~₹20,000 per drone × 3 drones = ₹60,000 in compute alone. Very expensive
   for a student competition.
2. **Power:** Jetson Nano draws 10 W under full GPU load. This significantly reduces
   flight time.
3. **Boot time:** Jetson Nano takes ~45–60 s to boot. You need to boot before
   placing on the field — then carry it armed/running to the start zone.
4. **Availability:** Jetson Nano 4GB is increasingly hard to source in India (supply
   constraints). Jetson Orin Nano is the replacement but costs even more.

### Verdict
**Ruled out for cost and practical reasons.** RPi 5 + ncnn gives 10–15 FPS at
640×640 at 1/3 the weight, 1/5 the cost, and better availability.

---

## Comparison summary

| Platform | YOLOv8n FPS (320×320) | Weight | Cost | Verdict |
|---|---|---|---|---|
| RPi 4B | 8–10 FPS | ~70 g | ~₹5,000 | **Use this (PoC)** |
| RPi 5 | 25–35 FPS | ~85 g | ~₹9,000 | **Upgrade to this (Finale)** |
| RPi 4B + Coral | 20–40 FPS | ~108 g | ~₹8,500 | Good alternative to RPi 5 |
| Jetson Nano | 60–100 FPS | ~168 g | ~₹20,000 | Too expensive, too heavy |

---

## Final recommendation

**Development:** Use RPi 4B (4 GB). Easy to source, cheap, replaceable if crashed.

**Grand Finale:** Upgrade to RPi 5 (8 GB) if available. Runs YOLOv8n at 640×640 for
better mine detection accuracy. Weight difference is only 15 g vs RPi 4B.

**If RPi 5 unavailable:** RPi 4B + Coral USB TPU. Comparable performance.

---

## References

1. YOLOv8–YOLOv12 benchmark on single-board computers (MDPI 2025):
   https://www.mdpi.com/2504-4990/8/7/204

2. Google Coral USB Accelerator datasheet:
   https://www.coral.ai/static/files/Coral-USB-Accelerator-datasheet.pdf

3. Ultralytics Coral Edge TPU guide:
   https://docs.ultralytics.com/guides/coral-edge-tpu-on-raspberry-pi

4. YOLOv8 ncnn RPi 4/5 (Qengineering):
   https://github.com/Qengineering/YoloV8-ncnn-Raspberry-Pi-4

5. Benchmarking deep learning on edge devices:
   https://arxiv.org/html/2409.16808v1
