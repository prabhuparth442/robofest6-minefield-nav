# 05 — Swarm Failure Modes

## What breaks, when, and what happens next

---

## Why think about failures before they happen

In a competition, unexpected events WILL occur. A drone that crashes silently is a
-20 point loss. A drone that gracefully lands is a zero-point loss. A swarm that
adapts to a failure and still delivers a partial path is a partial score. The difference
between these outcomes is whether you've written the failure-handling code before the
competition day.

Every failure mode below has a recommended response and an implementation sketch.

---

## Failure 1: Drone crash mid-mission

### What happens physically

During scanning, Drone 2 (covering Y=5–10 m strip) has a motor ESC failure. It drops
from 2.5 m altitude onto the field at t=80 s. It has scanned 40% of its strip
(x = 0–24 m, y = 5–10 m). The remaining 60% (x = 24–60 m, y = 5–10 m) is unscanned.

### Impact on coverage

Without adaptation: 60% of Drone 2's strip (60% × 5 m × 60 m = 3 m × 60 m = 180 m²)
is unscanned. Out of 900 m² total field: **20% unscanned**.

If any mines are in that unscanned zone and the person walks through it → person walks
through mines → safety hazard + likely judging penalty.

### Detection

Each drone broadcasts heartbeats at 10 Hz. If Drone 1 or Drone 3 receives no heartbeat
from Drone 2 for 3 consecutive seconds:

```python
class SwarmMonitor:
    def __init__(self, drone_id, expected_drones):
        self.last_seen = {d: time.time() for d in expected_drones}
        self.failed_drones = set()

    def update_heartbeat(self, drone_id, timestamp):
        self.last_seen[drone_id] = timestamp

    def get_failed_drones(self):
        now = time.time()
        return {d for d, t in self.last_seen.items() if now - t > 3.0}

    def check_and_adapt(self, my_id, my_strip_y_min, my_strip_y_max):
        failed = self.get_failed_drones()
        if failed:
            # Recalculate how to cover uncovered strips
            return self.compute_expanded_strip(my_id, failed,
                                               my_strip_y_min, my_strip_y_max)
        return None
```

### Response options

**Option A: Abort + conservative path**
Drones 1 and 3 immediately switch to path-planning mode with the partial map they
have. Unscanned cells are treated as mines (conservative). Path is routed around
the unscanned zone.

```python
def conservative_path(mine_map, scanned_mask):
    combined = mine_map.copy()
    # Treat unscanned as dangerous
    combined[scanned_mask == 0] = 2
    return astar_safe_path(combined, START, GOAL)
```

If there's no route through the conservative map (the unscanned zone blocks the entire
width), relax the assumption and route through the unscanned zone with a warning marker
(flashing LED = "proceed with caution").

**Option B: One surviving drone extends its coverage**
Drone 1 (finished with Y=0–5 m) extends its scan to cover part of Drone 2's uncovered
zone. This adds ~30–60 s to scan time but may be within budget.

```python
def expand_scan_zone(original_y_max, failed_drone_y_max, time_remaining):
    extra_width = failed_drone_y_max - original_y_max  # e.g. 5m extra
    passes_needed = int(extra_width / 1.5) + 1          # 4 more passes
    time_needed = passes_needed * 60 / 1.5              # at 1.5 m/s = 160 s
    if time_needed < time_remaining - 90:  # leave 90s for person crossing
        return failed_drone_y_max
    else:
        # Can only cover partial extension
        affordable_passes = int((time_remaining - 90) * 1.5 / 60)
        return original_y_max + affordable_passes * 1.5
```

### Recommended response

For Grand Finale: implement Option A (conservative path) as the default, with
Option B as a time-permitting enhancement. A conservative path that avoids unscanned
zones is better than sending the person into unknown territory.

### Score impact analysis

- Without recovery: miss 20% of field. Likely miss ~8 of 40 mines in unscanned zone.
  Mine detection score: ~(32/40) × 25 = 20 pts instead of 25 pts.
- With conservative routing: person may take longer path but avoids hazard. Safe
  crossing intact (20 pts). Net impact: ~-5 pts on mine detection, retain +20 crossing.

---

## Failure 2: Communications loss mid-mission

### Scenario

At t=60 s, the WiFi link between Drone 1 and the others drops (interference from another
team's equipment, signal obstruction). Drones 2 and 3 continue flying but stop receiving
mine updates from Drone 1 (and vice versa).

### Detection

```python
COMMS_TIMEOUT = 5.0  # seconds without any message = comms lost

def is_comms_alive(drone_id, last_msg_time):
    return time.time() - last_msg_time < COMMS_TIMEOUT
```

### Response: Graceful degradation to pre-planned mode

Each drone must have a fallback mode that activates on comms loss:

```python
def comms_lost_handler(self):
    """Called when no heartbeat received for COMMS_TIMEOUT seconds"""
    logger.warning(f"Comms lost at t={time.time():.1f}. Switching to autonomous mode.")
    self.state = DroneState.AUTONOMOUS_SOLO

    # Complete own strip scan independently
    # Do NOT attempt to merge maps
    # When strip complete, compute path ONLY through own strip
    # Mark own strip's path with LEDs, hope other drones do the same
    self.mission.run_solo_fallback(
        y_min=self.strip_y_min,
        y_max=self.strip_y_max
    )
```

In solo fallback mode, each drone computes a partial path through only its own strip.
The combined effect of three drones each marking their own strip section creates a
full field-width guidance corridor — less optimal than a unified path, but functional.

### Worst case

If comms drops AND Drone 2 crashes AND Drone 3's solo path avoidance is wrong → no
recovery. But this is a compound failure requiring three simultaneous independent events.
Accept this edge case.

---

## Failure 3: Battery depletion asymmetry

### Scenario

Drone 3 has a slightly lower-capacity battery due to a charging issue the night before.
At t=150 s, its battery reaches the low-voltage failsafe threshold (e.g., 3.5 V/cell).
Drones 1 and 2 have 8–10 minutes remaining.

### ArduPilot failsafe

ArduPilot has a built-in battery failsafe:
```
BATT_FS_LOW_ACT = 2   # Land on low battery
BATT_LOW_VOLT = 21.0  # 4S battery: 3.5V × 6 = 21V threshold
```

When triggered: ArduPilot ignores mission commands and begins landing. The RPi will
see the drone descend despite sending waypoint commands.

### Detection

```python
def monitor_battery(master_mavlink):
    msg = master_mavlink.recv_match(type='SYS_STATUS', blocking=False)
    if msg:
        voltage = msg.voltage_battery / 1000.0  # mV to V
        remaining = msg.battery_remaining        # %
        if remaining < 15:
            logger.warning(f"LOW BATTERY: {voltage:.1f}V, {remaining}%")
            return True
    return False
```

### Response

1. Broadcast "LOW BATTERY" status to other drones immediately
2. Other drones receive this and update their scan plans:
   - If Drone 3 has completed >70% of its strip: probably fine, conservative path
     will cover the remainder
   - If Drone 3 has completed <50% of its strip: Drone 1 or 2 extends to cover
     the critical part of Drone 3's strip
3. Drone 3 performs ArduPilot failsafe landing (automatic)

### Prevention

Before competition: charge all batteries to exactly the same level. Verify capacity
match by running a 5-minute hover test on each drone and comparing voltage drop rates.

---

## Failure 4: Map disagreement at strip boundaries

### Scenario

Drone 1 and Drone 2's strips share a boundary at Y=5 m. Due to navigation drift,
Drone 1 occasionally scans at Y=5.2 m (inside Drone 2's strip). Both detect an object
at approximately (x=32, y=5.1) but report slightly different positions:
- Drone 1: mine at (32.2, 5.15)
- Drone 2: mine at (31.8, 4.95)

Are these the same mine, or two mines 0.25 m apart?

### Deduplication logic

```python
def merge_mine_lists(all_detections, dedup_radius=0.6):
    """
    Merge detections within dedup_radius of each other into one detection.
    Keep the highest-confidence detection.
    Uses greedy clustering.
    """
    merged = []
    for det in sorted(all_detections, key=lambda d: -d['confidence']):
        # Check if too close to any already-merged detection
        close_to_existing = any(
            euclidean(det, m) < dedup_radius
            for m in merged
        )
        if not close_to_existing:
            merged.append(det)
    return merged

def euclidean(a, b):
    return ((a['x']-b['x'])**2 + (a['y']-b['y'])**2) ** 0.5
```

With `dedup_radius = 0.6 m`: two detections within 0.6 m of each other are treated
as the same mine. This is conservative — a 30 cm mine with 60 cm deduplication radius
means we'd only miss a mine if two real mines are <60 cm apart (which the PS says won't
happen — each mine has a 1 m exclusion zone).

### Disagreement on "clear" vs "mine"

A more serious case: Drone 1 classifies (32, 5.1) as **clear** (confidence 0.3 below
threshold), Drone 2 classifies it as **mine** (confidence 0.75).

Policy: **err on the side of caution**. If any drone reports mine above threshold at a
location, mark it as mine regardless of other drones' readings.

```python
def merge_with_max_confidence(detections_per_drone):
    """Any drone reporting a mine overrides others' 'clear' reading"""
    unified = {}
    for det in detections_per_drone:
        key = (round(det['x']/0.5), round(det['y']/0.5))  # 0.5m grid
        if key not in unified or det['confidence'] > unified[key]['confidence']:
            unified[key] = det
    return list(unified.values())
```

---

## Failure 5: Human enters field before path is marked

### Scenario

The person-at-risk becomes impatient and begins walking into the minefield before the
drones have completed scanning and marked the safe path.

### Detection

If drones have a downward camera looking at the field during scanning, they can detect
the person using their own detection pipeline:

```python
# Person is ~1.7m tall, ~0.5m wide
# At 2m altitude, a 70cm wide object covers ~0.7/2 × focal_length pixels
# Using same YOLOv8 model but with a "person" class

def is_person_in_field(frame, person_classifier):
    results = person_classifier(frame)
    return any(cls == 'person' and conf > 0.7
               for cls, conf in results)
```

Or simpler: use the HEARTBEAT from the person's transponder. But the PS probably
doesn't include a transponder for the person.

Simplest approach: **gesture/voice command triggers mission start AND locks out person
entry** — the mission doesn't start until all drones are in the air. Once in the air,
drones scan fast enough (3 minutes) that the person waits. A verbal announcement or
buzzer alerts the person to wait.

### Response if detected mid-scan

All drones immediately halt scanning, loiter in place. One drone descends to 1 m
altitude near the person and flashes RED LED (warning: unscanned zone). Mission resumes
when person backs out of field.

```python
def human_in_field_handler(drone_id):
    """Called when human detected in field during scanning"""
    # Pause at current position
    set_loiter_mode(mavlink_conn)
    # Flash red LED at 2 Hz
    activate_led_pattern('RED_FLASH_2HZ')
    # Broadcast to other drones
    broadcast({'type': 'HUMAN_IN_FIELD', 'drone': drone_id})
    # Wait for field clear signal (manual or auto-detected)
    wait_for_field_clear()
    # Resume scan
    resume_scan_from_current_position()
```

---

## Failure 6: Navigation failure (optical flow saturation)

### Scenario

During a turn or in bright sunlight, the optical flow sensor loses lock (e.g., direct
sunlight onto the downward camera). ArduPilot detects EKF position variance is too
high and disables GUIDED mode, switching to STABILISE (manual-like) or LAND.

ArduPilot EKF failsafe:
```
FS_EKF_ACTION = 1      # 1 = Land on EKF failsafe (2 = AltHold; verify in ArduPilot docs)
FS_EKF_THRESH = 0.8    # EKF variance threshold that triggers the failsafe (default)
```

### Detection and prevention

```python
# Monitor EKF health via MAVLink
msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=False)
if msg:
    ekf_flags = msg.flags
    # EKF_STATUS_FLAGS bits: 0x02 = horizontal velocity, 0x08 = horizontal position (relative),
    # 0x10 = horizontal position (absolute; needs GPS/external nav, so it is NOT expected to be set here)
    if not (ekf_flags & 0x02):
        logger.error("EKF horizontal velocity bad — flow may be lost")
    if not (ekf_flags & 0x08):
        logger.error("EKF relative horizontal position bad — may divert from strip")
```

### Prevention

- Mount optical flow sensor on rubber vibration isolators (reduces sensor noise)
- Use the laser rangefinder altitude output (not barometer) — more stable altitude
  estimate prevents altitude-induced flow scale errors
- Pre-flight: run flow calibration routine (hover in textured area, check estimated
  vs actual position from known start)
- Set EKF to use barometer as fallback if flow drops out (not ideal but recoverable)

---

## Failure summary table

| Failure | Detection | Response | Score impact |
|---|---|---|---|
| Drone crash | Heartbeat loss >3s | Conservative path, extend coverage | -5 to -10 pts |
| Comms loss | No msg for 5s | Solo fallback per drone | -5 pts (suboptimal path) |
| Battery fail | Low voltage MAVLink | Broadcast + graceful land | -5 pts (partial coverage) |
| Map disagreement | Deduplication | Max confidence wins | Minimal |
| Human in field | Visual + loiter | Pause, warn, resume | 0 pts (delay) |
| Nav failure (EKF) | EKF status flags | Hold briefly, then LAND in place (no RTL without GPS) | -10 to -20 pts if crash |

---

## Pre-competition checklist

- [ ] All batteries fully charged and capacity-matched (within 5% of each other)
- [ ] WiFi ad-hoc link tested at 60 m range outdoors
- [ ] Heartbeat loss handler tested (manually kill WiFi, verify drone goes solo)
- [ ] Battery failsafe voltage set in ArduPilot params (BATT_LOW_VOLT)
- [ ] EKF failsafe tested (cover optical flow sensor, verify drone lands safely)
- [ ] Conservative path algorithm tested with partial mine maps
- [ ] Deduplication tested with boundary-zone synthetic detections
- [ ] Human-detection loiter tested in outdoor conditions

> GPS-denied review (2026-10-24): replaced RTL with LAND for the EKF failure, and corrected the EKF failsafe parameters and status-flag bits above (please re-verify against the ArduPilot/MAVLink docs).
