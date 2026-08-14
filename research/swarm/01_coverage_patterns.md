# 01 — Coverage Patterns for 15 × 60 m Field

## How three drones divide and scan the minefield

---

## Setup

**Field:** 15 m wide × 60 m long. Drone starts at the 1 m × 15 m start zone.
The actual minefield is 58 m × 15 m.

**Drones:** 3 drones. Each weighs ≤ 750 g. Flying speed: 1–2 m/s. Altitude: 2 m.
Sensor: camera pointing down + MLX90640 thermal. Camera FOV at 2 m: roughly 2.4 m ×
1.8 m (assuming a 90° lens). Effective scan width per pass: ~1.5 m (with overlap).

**Time budget:** 10 minutes total mission time. If the person needs ~3–4 minutes to
cross once the path is marked, scanning must complete in ~6 minutes.

**Key constraint:** No area redundancy. Drones must not scan the same ground.

---

## Coverage math baseline

For a 15 × 60 m = 900 m² field, at 1.5 m effective scan width per pass:

```
Passes needed per drone (if each covers full width): 15 / 1.5 = 10 passes of 60 m
Total path length per drone: 10 × 60 = 600 m
At 2 m/s: 600 / 2 = 300 seconds = 5 minutes per drone (solo)

With 3 drones splitting the field:
  Each covers 5 m width = 5 / 1.5 ≈ 4 passes of 60 m
  Each drone path: 4 × 60 = 240 m
  At 2 m/s: 120 seconds = 2 minutes
  Turnaround time and deceleration: ~30 seconds extra
  Realistic scan time: ~2.5–3 minutes per drone
```

So 3 drones covering parallel strips should complete scanning in ~3 minutes, leaving
7 minutes for path marking and human crossing. This is feasible.

---

## Proposal A: Parallel Strip (Lawnmower)

### How it works

Field is divided into 3 equal vertical strips of 5 m width × 60 m length:

```
|<---5m--->|<---5m--->|<---5m--->|
|          |          |          |
| Drone 1  | Drone 2  | Drone 3  |
| Y=0–5m   | Y=5–10m  | Y=10–15m |
|          |          |          |
| →→→→→    | →→→→→    | →→→→→    |
|    ↓     |    ↓     |    ↓     |
| ←←←←←   | ←←←←←   | ←←←←←   |
|    ↓     |    ↓     |    ↓     |
| →→→→→    | →→→→→    | →→→→→    |
```

Each drone flies a lawnmower pattern within its 5 m strip. Passes go left-right
(across the 60 m length), spaced 1.5 m apart in the Y direction, with a 0.5 m overlap
at adjacent strip boundaries for safety.

### Waypoint sequence (Drone 1 example, local frame)

```python
# Drone 1 strip: Y = 0 to 5 m, X = 0 to 60 m
# Altitude: 2 m
waypoints = [
    (0,   0.75, 2),   # Start of pass 1 (centred in first 1.5m band)
    (60,  0.75, 2),   # End of pass 1
    (60,  2.25, 2),   # Start of pass 2
    (0,   2.25, 2),   # End of pass 2
    (0,   3.75, 2),   # Start of pass 3
    (60,  3.75, 2),   # End of pass 3
    (60,  4.5,  2),   # Boundary pass (edge + overlap)
]
# (X along 60m axis, Y across 15m axis, Z altitude)
```

### ArduPilot implementation

Send waypoints as `SET_POSITION_TARGET_LOCAL_NED` in GUIDED mode. Each drone has its
strip boundaries pre-loaded before launch. No inter-drone comms needed.

```python
# pymavlink: send local NED position target
master.mav.set_position_target_local_ned_send(
    0,                        # time_boot_ms
    master.target_system,     # target_system
    master.target_component,  # target_component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111111000,        # type_mask: position only
    x, y, -2.0,               # NED: x=North, y=East, z=Down (so -2 = 2m up)
    0, 0, 0,                  # vx, vy, vz
    0, 0, 0,                  # afx, afy, afz
    0, 0                      # yaw, yaw_rate
)
```

### Pros
- Extremely simple to implement — just a list of waypoints
- Zero inter-drone comms needed during flight
- Completely deterministic and verifiable before flight
- Easy to debug (you know exactly where each drone should be at any time)
- Survives comms failure with no degradation

### Cons
- No adaptation: if one drone fails, its strip is missed
- Boundaries are hard — if Drone 1 drifts 1 m right (navigation error), it overlaps
  Drone 2's strip and could cause a collision
- Must be launched simultaneously (or with careful timing offsets)

### Mine detection completeness estimate
With 1.5 m scan width, 0.5 m overlap at boundaries, and 3 drones covering all 15 m:
**Coverage ≈ 95–98%** assuming no navigation drift. At 0.5 m drift, ~85–90%.

### Estimated scan time
At 2 m/s, 4 passes × 60 m per drone = 240 m per drone. Plus 3 turns × ~5 s decel/accel.
**Estimated time: ~130–150 seconds (≈ 2.5 minutes)**

### Failure modes
1. Drone crashes → its strip uncovered, no rerouting
2. Navigation drift → strip overlap → collision risk
3. All three launch simultaneously → timing errors in takeoff

### Verdict
**Best first implementation.** Use this for PoC stage. Add comms and failure handling
for Grand Finale.

---

## Proposal B: Staggered Offset Strips

### How it works

Same as Proposal A (parallel strips), but adjacent strips have a 0.5 m deliberate
overlap, AND drones are staggered by 10 m along the X axis so they're never at the
same X position simultaneously.

```
Time T=0:
  Drone 1: at X=0, Y=0.75  (leading)
  Drone 2: at X=-10, Y=5.75  (10m behind)
  Drone 3: at X=-20, Y=10.75  (20m behind)
```

The 10 m longitudinal separation means if Drone 1 and 2 drift toward each other
laterally, they're 10 m apart longitudinally — no mid-air collision.

Overlap zones (0–0.5 m between strips) are intentionally scanned by both adjacent
drones for redundancy at boundaries (where mines are most likely to be "missed").

### ArduPilot implementation
Same as Proposal A, but each drone's first waypoint is offset:
- Drone 1: start at X=0
- Drone 2: start at X=-10 (outside field) → enter field at X=0 once Drone 1 is at X=10
- Drone 3: start at X=-20 → enter at X=0 once Drone 2 is at X=10

### Pros
- Collision safety much better than Proposal A (longitudinal separation)
- Boundary overlap reduces missed mines
- Still no runtime comms needed (stagger is pre-planned timing)

### Cons
- Slight strip overlap means marginally longer total scan time (~10% more)
- More complex pre-flight waypoint calculation
- Drone 2 and 3 must wait during the initial delay — fuel wasted hovering

### Mine detection completeness estimate
**Coverage ≈ 97–99%** — boundary overlap catches edge cases.

### Estimated scan time
~160–170 seconds (stagger adds ~20–30 s over Proposal A).

### Failure modes
1. Timing errors in stagger offset → drones catch up with each other
2. Wind pushes all drones in the same direction → stagger reduces but doesn't eliminate collision risk

### Verdict
**Preferred baseline.** The longitudinal stagger is low-cost insurance against the
primary collision risk in Proposal A.

---

## Proposal C: Spiral Inward per Zone

### How it works

Each drone is assigned a 5 m × 60 m zone (same as Proposal A). Within its zone,
instead of a lawnmower, it flies an inward spiral:

```
Drone 1 zone (5m × 60m):
  Pass 1: Outer perimeter clockwise  (covers edges)
  Pass 2: Inner pass 1.5m inside     (covers next ring)
  Pass 3: Continue inward until centre
```

This explores the boundary of the zone first (where mines near strip edges are), then
converges inward.

### Pros
- Starts coverage from all outer edges simultaneously across all drones
- If time runs out, the outer passes have already covered the most exposed area
- Natural shape for detecting mines near the human's likely entry/exit path

### Cons
- Path length is longer than lawnmower (corners and diagonal segments add distance)
- Harder to generate waypoints programmatically (need to calculate spiral offsets)
- Harder to verify coverage completeness
- Turnaround geometry is complex for ArduPilot waypoint navigation

### Mine detection completeness estimate
**Coverage ≈ 90–95%** (spiral centre can have gaps if spacing isn't calibrated right).

### Estimated scan time
~180–200 seconds — corners and longer path add ~30–50 s over lawnmower.

### Failure modes
1. Spiral centre not covered if zone has odd aspect ratio
2. Harder to resume if drone loses position mid-spiral

### Verdict
Not recommended for this PS. Lawnmower is simpler and faster. Use spiral only if you
want to guarantee boundary coverage first as a partial-credit strategy.

---

## Proposal D: Dynamic Frontier-Based Coverage

### How it works

No pre-assigned strips. The field is represented as a grid of 0.25 m cells
(15m × 60m → 60 × 240 = 14,400 cells). All drones share a coverage map (via wireless
or pre-agreed start positions).

Each drone:
1. Picks the nearest **frontier cell** (boundary between covered and uncovered)
2. Flies to it
3. Marks covered cells as it scans
4. Repeats

No static assignment. Drones naturally avoid each other's territory by picking
frontiers in different directions.

```python
# Simplified frontier selection
def pick_frontier(my_position, coverage_map, other_drone_positions):
    frontiers = get_frontier_cells(coverage_map)
    # Filter out frontiers claimed by other drones
    available = [f for f in frontiers if not claimed_by_other(f, other_drone_positions)]
    # Pick nearest available frontier
    return min(available, key=lambda f: distance(my_position, f))
```

Reference: Frontier-led swarming (Burgard et al. concept, implemented in ROS 2 multi-
robot packages). A 2022 ScienceDirect paper demonstrates robust coverage of unknown
environments with this method.

### Pros
- Fully adaptive — if one drone fails, the others' frontiers expand to cover the gap
- No pre-assigned strips — naturally distributes work
- Best theoretical coverage efficiency

### Cons
- Requires real-time inter-drone comms to share coverage map (or will duplicate work)
- Each drone must run a frontier-selection algorithm (more onboard compute)
- Harder to debug — unpredictable flight paths
- If comms drops, drones default to independently selecting frontiers → high redundancy

### Mine detection completeness estimate
**Coverage ≈ 98–100%** assuming comms is reliable. Without comms: ~70–80% (high
redundancy, not all area covered).

### Estimated scan time
~150–200 seconds depending on frontier selection efficiency and comms latency.

### Failure modes
1. Comms loss → each drone has stale map → high redundancy
2. Two drones pick the same frontier simultaneously → collision risk
3. Coverage map merge conflict → some cells double-marked or missed

### Verdict
Best long-term goal for a robust swarm. Significant software complexity. Feasible
for Grand Finale if comms (see `02_inter_drone_comms.md`) is implemented early.

---

## Proposal E: Leader-Follower Convoy with Lateral Offset

### How it works

Drone 1 (leader) flies straight up the field centre at Y=7.5 m. Drones 2 and 3 fly
in formation 5 m to either side (Y=2.5 m and Y=12.5 m), offset 5 m behind the leader.

```
Top view:

Drone 2  →→→→→→→→→→→→→→→→→→
         (Y=2.5m)

Drone 1        →→→→→→→→→→→→
               (Y=7.5m, leader)

Drone 3    →→→→→→→→→→→→→→→→
           (Y=12.5m)
```

All three advance at the same speed down the X axis. They cover 3 strips
simultaneously in a single pass. At the end, all three U-turn and come back offset
1.5 m to fill in gaps.

### Pros
- Scan one direction at full width in a single pass — very fast
- Natural formation — easy to check visually that it's working
- Leader can broadcast turn commands to followers — simple comms protocol

### Cons
- Requires comms for formation keeping (leader commands U-turn timing to followers)
- All three drones in flight simultaneously at close Y spacing (2.5 m gaps)
- If leader crashes, followers continue straight with no waypoint updates
- Formation keeping in wind is hard without a position-sharing comms link

### Mine detection completeness estimate
**Coverage ≈ 90–95%** per single sweep. ~97–99% after return sweep with offset.

### Estimated scan time
~180–200 seconds (2 sweeps of 60m, including U-turn).

### Failure modes
1. Wind pushes drones off lateral position → drones collide
2. Leader crash → followers fly off-field or collide with each other
3. Formation gap too small → required comms precision is high

### Verdict
Looks impressive for swarm formation scoring (+10 pts), but collision risk is too
high for a 15 m wide field with 3 drones spaced 2.5 m apart. Consider for PoC
demonstration only, with larger lateral spacing.

---

## Summary comparison

| Proposal | Scan time | Coverage % | Comms needed | Collision risk | Complexity |
|---|---|---|---|---|---|
| A: Parallel strip | ~150 s | 95–98% | None | Medium | Very low |
| B: Staggered offset | ~165 s | 97–99% | None | Low | Low |
| C: Spiral | ~190 s | 90–95% | None | Low | Medium |
| D: Frontier-based | ~175 s | 98–100% | Required | Low | High |
| E: Leader-follower | ~190 s | 90–99% | Required | High | Medium |

---

## Recommendation

**Phase 1 (PoC stage):** Implement Proposal B (Staggered Offset Strips). No comms
needed. Predictable paths. 10 m longitudinal stagger eliminates the primary collision
risk. Achieves ~97% coverage.

**Phase 2 (Grand Finale):** Add frontier-based adaptive coverage on top of B. If comms
is available, drones can dynamically reassign strips if one drone fails.

---

## References

1. Frontier-led swarming (coverage of unknown environments):
   https://www.sciencedirect.com/science/article/abs/pii/S2210650222001389

2. Distributed coverage path planning for UAV swarms:
   https://www.sciencedirect.com/science/article/pii/S0957417426012959

3. Modified lawnmower search pattern for weighted regions:
   https://www.researchgate.net/publication/224718383

4. ArduPilot GUIDED mode waypoint commands:
   https://ardupilot.org/dev/docs/mavlink-commands.html
