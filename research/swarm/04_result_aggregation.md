# 04 — Result Aggregation

## How three drones' mine maps get merged into one safe path

---

## The problem stated precisely

At the end of the scanning phase, the system has:
- Drone 1: a list of (x, y, confidence) mine detections in its strip (Y = 0–5 m)
- Drone 2: a list of mine detections in its strip (Y = 5–10 m)
- Drone 3: a list of mine detections in its strip (Y = 10–15 m)

The system needs:
- One unified mine map covering all 15 × 60 m
- A safe path from (x=0, y=7.5) to (x=60, y=7.5) with 1 m clearance from every mine
- That path encoded as waypoints or visual markers

The question is: **which drone computes the unified map, and when?**

Sub-questions:
- Can this happen before scanning is complete? (Incremental path planning)
- What if one drone's data is unavailable? (Partial map path planning)
- How is the path communicated to the person? (Physical LED/marker)

---

## Grid representation (common to all proposals)

Regardless of aggregation method, the mine map should be a discrete grid:

```
Field: 15 m × 60 m
Grid resolution: 0.25 m
Grid size: (15/0.25) × (60/0.25) = 60 × 240 = 14,400 cells

Each cell stores:
  - 0: unscanned
  - 1: scanned, clear
  - 2: mine detected (occupancy > threshold)

Memory: 14,400 bytes = 14 KB (fits easily on RPi)
```

```python
import numpy as np

# Grid: 60 rows (Y), 240 cols (X)
mine_map = np.zeros((60, 240), dtype=np.uint8)

def mark_mine(x_m, y_m):
    """Mark a detected mine at world coordinates (x_m, y_m) in metres"""
    xi = int(x_m / 0.25)   # column index
    yi = int(y_m / 0.25)   # row index
    # Mark 1m exclusion zone (radius = 4 cells at 0.25m/cell)
    for dy in range(-4, 5):
        for dx in range(-4, 5):
            if dx*dx + dy*dy <= 16:  # circle radius 4 cells
                mine_map[yi+dy, xi+dx] = 2
```

---

## Proposal A: Post-flight USB Dump

### How it works

After scanning completes, all three drones land. Each drone's mine detection list is
copied off its SD card (or output to USB) and merged on a laptop manually. The path
is computed and loaded onto one drone for the guidance phase.

```
Timeline:
  T=0–180s:  All three drones scan (simultaneously)
  T=180s:    All drones land
  T=180–200s: Human copies mine_detections.json from each RPi SD card
  T=200–210s: Laptop merges three lists, runs path algorithm, outputs waypoints
  T=210s:    Upload waypoints to one escort drone
  T=210–600s: Escort drone guides person
```

### What "merging" means

```python
import json

all_mines = []
for drone_id in [1, 2, 3]:
    with open(f'drone{drone_id}_mines.json') as f:
        mines = json.load(f)
    all_mines.extend(mines)

# Deduplicate (two drones detecting same mine at boundary)
def deduplicate(mines, threshold=0.5):
    unique = []
    for mine in mines:
        too_close = False
        for existing in unique:
            if abs(mine['x'] - existing['x']) < threshold and \
               abs(mine['y'] - existing['y']) < threshold:
                too_close = True
                # Keep higher confidence detection
                if mine['confidence'] > existing['confidence']:
                    unique.remove(existing)
                    unique.append(mine)
                break
        if not too_close:
            unique.append(mine)
    return unique
```

### Path planning (A*)

```python
import heapq

def astar_safe_path(mine_map, start, goal):
    """Find path through mine_map avoiding cells marked 2 (mine/exclusion)"""
    h, w = mine_map.shape
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct(came_from, current)

        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < w and 0 <= ny < h and mine_map[ny,nx] != 2:
                new_g = g_score[current] + (1.4 if dx and dy else 1.0)
                if new_g < g_score.get((nx,ny), float('inf')):
                    g_score[(nx,ny)] = new_g
                    f = new_g + abs(nx-goal[0]) + abs(ny-goal[1])
                    heapq.heappush(open_set, (f, (nx,ny)))
                    came_from[(nx,ny)] = current
    return None  # No path found
```

### Pros
- Zero inter-drone comms needed during flight
- Path computed on powerful laptop — any algorithm, no compute limit
- Can visually verify the merged map before dispatching the escort drone

### Cons
- Requires human intervention at T=180s (SD card copying)
- The 20–30 second manual step is operationally fragile under competition pressure
- Laptop is external to the mission — technically the "external computer" prohibited
  by the PS? (After the flight, not during — this is ambiguous)

### Timing for our 10-minute budget
- Scan: 180 s
- Merge + path: 30 s manual + 10 s compute
- Person crossing at 1 m/s (slow to be safe): 60 s
- **Total: 280 s ≈ 4.7 minutes — well within 10 minutes** ✓

### Verdict
**Best for PoC stage.** Simple, reliable, debuggable. For Grand Finale, replace the
manual step with Proposal C or D.

---

## Proposal B: WiFi Broadcast at End of Scanning Phase

### How it works

When each drone completes its scan, it broadcasts its full mine list over WiFi to all
other drones. One designated "master drone" (e.g., Drone 1) collects all three lists,
merges them, runs A*, and transmits the final path back to the others.

```
T=0–120s:   Drone 1 scans strip 1
T=0–120s:   Drone 2 scans strip 2 (simultaneously)
T=0–120s:   Drone 3 scans strip 3 (simultaneously)
T=120s:     All drones finished scanning
            Drone 2 broadcasts MINE_LIST to Drone 1
            Drone 3 broadcasts MINE_LIST to Drone 1
T=122s:     Drone 1 has all three lists, runs merge + A*
T=123s:     Drone 1 broadcasts final PATH waypoints
T=123s+:    All drones fly to path waypoints, activate LEDs
```

### WiFi bandwidth check

A mine list of 40 mines × 3 fields (x, y, confidence) × 4 bytes each = 480 bytes.
JSON overhead: ~1200 bytes total. At 100 kbps effective WiFi throughput: <0.1 seconds.
**No bandwidth concern.**

### Pros
- No human intervention needed
- Path computed within seconds of scanning completion
- All three drones can then escort the person along the path simultaneously

### Cons
- Requires WiFi comms hardware (see `02_inter_drone_comms.md` Proposal B)
- If master drone (Drone 1) crashes before broadcasting, no one has the full map
  (single point of failure at aggregation step)
- WiFi range at far end of 60 m field may be marginal

### Failure handling

If Drone 1 crashes before aggregation:
```python
# Each drone monitors heartbeats
# If master (drone_id=1) misses 3 consecutive heartbeats, promote next lowest drone_id
def elect_new_master(known_drones):
    alive = [d for d in known_drones if d.last_heartbeat < 3.0]  # <3s ago
    return min(alive, key=lambda d: d.id)
```

### Timing
- Scan: ~120–150 s
- WiFi broadcast + merge + A*: ~3 s
- LED activation: ~5 s
- Person crossing: ~60–90 s
- **Total: ~178–248 s ≈ 3–4 minutes** ✓

### Verdict
**Preferred for Grand Finale.** Fast, automatic, needs only WiFi which we have from
Proposal B in `02_inter_drone_comms.md`.

---

## Proposal C: Master Drone Collects Incrementally via UWB

### How it works

Rather than waiting for scanning to complete, drones broadcast mine detections in real-
time to a designated master drone via UWB. The master drone continuously updates the
merged grid and begins path planning before scanning is complete.

```
t=0:          Drone 2 detects mine at (x=15, y=7.3)
t=0.005s:     UWB packet: "MINE:15.0:7.3:0.92" → Drone 1
t=0.006s:     Drone 1 updates mine_map[29,60] = 2
t=0.006s:     Drone 1 runs incremental A* update
...
t=100s:       Drone 1's strip scan complete, 60% of total field scanned
              Drone 1 already has partial path computed
t=120s:       All three strips complete
              Drone 1 finalises path and broadcasts
```

### Pros
- Real-time map building — if time runs out, partial path is still usable
- Inherently incremental — can start guiding person before full scan if needed

### Cons
- UWB hardware required (see `02_inter_drone_comms.md` Proposal C — legally grey)
- Master drone is still a single point of failure for aggregation

### Incremental A* sketch

```python
def update_path_incrementally(mine_map, current_path, new_mine_cell):
    """
    When a new mine is detected, check if it affects the current path.
    If not, path is still valid. If yes, recompute only affected segment.
    """
    # Check if new mine invalidates any path cell
    affected = [i for i, cell in enumerate(current_path)
                if manhattan_dist(cell, new_mine_cell) < 4]  # within 1m

    if not affected:
        return current_path  # Path still valid

    # Recompute from last valid cell before affected zone
    replan_start = max(0, min(affected) - 1)
    new_segment = astar_safe_path(mine_map,
                                   current_path[replan_start],
                                   current_path[-1])
    return current_path[:replan_start] + new_segment
```

### Verdict
**Best theoretical solution**, but requires UWB hardware. Implement if UWB is cleared
for competition. Otherwise use Proposal B (WiFi at end of scan).

---

## Proposal D: Shared Pre-mapped Grid (All Drones Write Own Cells)

### How it works

Rather than aggregating at one master drone, the grid is "distributed" — each drone
owns its strip's rows in the grid and broadcasts occupancy updates. Any drone receiving
all updates can reconstruct the full grid. The path is computed by any drone that has
a complete grid.

```python
# Each drone broadcasts its own strip's cells at 1 Hz
def broadcast_my_strip(grid, strip_y_min, strip_y_max, drone_id):
    y_min_cell = int(strip_y_min / 0.25)
    y_max_cell = int(strip_y_max / 0.25)

    # Only send cells that changed since last broadcast
    changed = {
        (xi, yi): grid[yi, xi]
        for yi in range(y_min_cell, y_max_cell)
        for xi in range(240)
        if grid_changed[yi, xi]
    }
    sock.sendto(pickle.dumps({'drone': drone_id, 'cells': changed}), BROADCAST)
```

Each drone locally merges received updates from other drones. After all three drones
have broadcast their strips, every drone has the full map independently. No single
point of failure.

### Pros
- Fully distributed — no master drone for aggregation
- Any surviving drone can compute the path if others fail
- Naturally resilient to partial failures

### Cons
- Bandwidth: 14,400 cells × 1 byte = 14 KB per full broadcast. At 1 Hz delta updates:
  much less (maybe 100–500 bytes per broadcast on average). Fine for WiFi.
- All drones must receive all others' updates — if one misses packets, its grid is stale
- More complex software — each drone runs both a broadcaster and a receiver

### Verdict
**Ideal architecture for robustness.** More complex to implement. Best if WiFi is
reliable across the full field. Makes failure handling (Proposal 05 below) much simpler.

---

## What happens if one drone fails mid-mission

Regardless of aggregation method, partial maps must be handled gracefully:

```python
def compute_path_with_partial_map(mine_map, scanned_mask):
    """
    mine_map: full grid (some cells may be unscanned)
    scanned_mask: which cells have been scanned by at least one drone

    Treat unscanned cells as potentially dangerous (unknown = obstacle)
    unless we have evidence they're clear.
    """
    # Mark all unscanned cells as obstacles for path planning
    conservative_map = mine_map.copy()
    conservative_map[scanned_mask == 0] = 2  # treat unknown as mine

    path = astar_safe_path(conservative_map, start, goal)

    if path is None:
        # No path through conservative map — relax: treat unknowns as clear
        # (risky but better than no guidance)
        path = astar_safe_path(mine_map, start, goal)

    return path
```

This "conservative then optimistic" fallback means: first try to route around unknown
areas, and only send the person through unscanned zones if there is literally no other
path.

---

## Summary

| Proposal | Comms needed | Human action | Latency | Failure tolerance |
|---|---|---|---|---|
| A: USB dump | None in-flight | Yes (manual) | ~30 s post-flight | Medium |
| B: WiFi at scan end | WiFi | None | ~3 s post-scan | Low (master PoF) |
| C: UWB incremental | UWB | None | Real-time | Low (master PoF) |
| D: Distributed grid | WiFi | None | ~5 s post-scan | High (no master) |

**Recommendation:** Proposal A for PoC, Proposal D for Grand Finale.

---

## References

1. A* pathfinding reference: https://en.wikipedia.org/wiki/A*_search_algorithm

2. ArduROSPi ROS2-pymavlink bridge for mission planning:
   https://discuss.ardupilot.org/t/ardurospi-ros-2-pymavlink-bridge/142058

3. Distributed coverage path planning for UAV swarms:
   https://www.sciencedirect.com/science/article/pii/S0957417426012959
