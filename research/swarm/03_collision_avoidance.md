# 03 — Multi-Drone Collision Avoidance

## Keeping three drones from hitting each other in a 15 m wide field

---

## Why this matters more than anything else

The PS scoring:

- Drone crash or surface contact: **-20 points**
- Collision or unsafe proximity: **-10 points**

Mine detection is worth 25 points. One collision wipes out half of that. One crash
wipes out the entire mine detection score. No collision avoidance strategy is perfect,
but the cost of getting it wrong is severe.

The field is 15 m wide. Three drones need to cover it. If each drone covers a 5 m
strip, adjacent drones are flying 5 m apart laterally. With 0.5 m navigation drift
(optical flow), they could be 4 m apart. At 2 m/s each, closing speed could be 4 m/s,
leaving 1 second of reaction time from 4 m away.

Collision avoidance is not optional.

---

## Proposal A: Altitude Separation

### How it works

Each drone operates at a different altitude. They scan the same X-Y area but at
different heights, so even if their horizontal positions overlap, they don't collide.

```
Drone 1: flies at 2.0 m AGL
Drone 2: flies at 2.5 m AGL
Drone 3: flies at 3.0 m AGL
```

Vertical separation of 0.5 m. Typical drone height is ~20 cm. So the gap between
Drone 1 prop arc and Drone 2 body is ~30 cm. This is technically safe but feels tight.

### ArduPilot implementation

Set altitude in the `SET_POSITION_TARGET_LOCAL_NED` z component:

```python
# Drone 1: 2.0m AGL
z_target = -2.0   # NED frame: negative = up

# Drone 2: 2.5m AGL  
z_target = -2.5
```

No extra software or comms needed — altitude is just a parameter.

### Effect on detection

At 2.0 m, MLX90640 55° FOV covers a ~2 m × 1.5 m ground patch.
At 3.0 m, it covers ~3 m × 2.25 m — bigger footprint but lower resolution per pixel.

Thermal pixel "density" at 3.0 m altitude: 32 pixels / 3 m = 10.7 pixels/metre.
At 2.0 m: 32 pixels / 2 m = 16 pixels/metre.

Drone 3 (at 3 m) has ~33% lower spatial resolution. For 20–30 cm mines: at 3 m, each
mine covers ~3 pixels. At 2 m, it covers ~5 pixels. Both detectable, but 2 m is better.

### Complexity: Very low
One parameter per drone. No software changes. No comms.

### Failure modes
1. Strong updraft pushes a low drone upward → altitude collision
2. Barometer drift → actual altitude differs from commanded altitude
   (Barometer accuracy: ±0.5 m at low altitudes in outdoor wind)
3. If Drone 2 barometer reads 0.5 m high, it flies at effective 3.0 m = same as Drone 3

### Mitigation
Use the laser rangefinder in the Matek 3901-L0X as the altitude sensor (not barometer).
Laser rangefinder accuracy: ±2 cm. This eliminates barometer drift almost entirely.

### Verdict
**Essential safety layer — implement always, for all scenarios.** Even when using other
avoidance methods, altitude separation is free insurance. 0.5 m separation with laser
altitude is ~±2 cm error — a 47.6 cm effective clearance.

---

## Proposal B: Time Separation (Sequential Launch)

### How it works

Drone 1 launches, scans its strip, lands. Then Drone 2 launches. Then Drone 3.
Zero chance of mid-air collision because only one drone is airborne at a time.

```
t=0:    Drone 1 launches, scans Strip 1
t=180:  Drone 1 lands
t=185:  Drone 2 launches, scans Strip 2
t=365:  Drone 2 lands
t=370:  Drone 3 launches, scans Strip 3
t=550:  Drone 3 lands
Total scan time: ~550 seconds = 9.2 minutes
```

At 9.2 minutes, there's no time left for the person to cross within the 10-minute limit.

### Verdict
**Mathematically incompatible with the time budget.** 550 seconds of sequential
scanning leaves 50 seconds for a person to cross 60 metres. A human walks at ~1.3 m/s.
60 m takes ~46 seconds. Barely possible but no margin. **Not recommended.**

Use ONLY as a testing/debugging approach during development, never for competition.

---

## Proposal C: Geographic Separation (Hard Zone Boundaries)

### How it works

Strictly enforce that each drone never crosses its strip's Y boundaries. Combined with
pre-planned flight paths (Proposal B from coverage patterns), each drone has a hard
geofence that ArduPilot enforces.

ArduPilot supports polygon fences and cylinder fences. For our strips:

```python
# Set a fence for Drone 1: Y = 0 to 5m, X = 0 to 60m
# ArduPilot fence parameters (loaded at startup)
FENCE_TYPE = 2     # Polygon
FENCE_ACTION = 1   # RTL if breached (but see GPS-denied caution below)
FENCE_MARGIN = 0.5 # 0.5m breach margin
```

Or more simply: constrain the waypoints themselves to always stay within [Y_min, Y_max].

```python
def clamp_waypoint(x, y, y_min, y_max):
    return x, max(y_min, min(y_max, y))
```

> **GPS-denied review (2026-10-24):** do not rely on ArduPilot's hardware fence or RTL in a GPS-denied setup. Fence polygons are defined in
> lat/lon and RTL needs a valid home and position; with no GPS, the home/EKF origin handling is unclear and the position
> being used is the same drifting estimate that caused the breach. Verify behaviour in SITL with GPS disabled. Preferred
> design: a **software fence in our own code**, working in local NED (clamp every waypoint to the strip and monitor the
> pose; on breach, hold or step back inside), with LAND, not RTL, as the last-resort action.

### Complexity: Low
Configure ArduPilot fence parameters per drone before launch.

### Failure modes
1. Navigation drift causes fence trigger → drone RTLs mid-mission → incomplete scan
2. If RTL uses barometric altitude (drifty) → drone descends at wrong altitude during RTL
3. Doesn't prevent vertical collisions if all drones have same altitude

### Integration with Proposal A
**Always combine with altitude separation.** Geographic boundaries prevent most
horizontal collision scenarios. Altitude separation catches the remaining cases where
navigation drift breaches the boundary before the fence triggers.

### Verdict
**Implement as always-on safety layer** alongside Proposal A. Fence + altitude
separation + staggered launch timing (Proposal B from coverage patterns) provides
three independent layers of collision protection.

---

## Proposal D: ORCA Velocity Obstacles (Reactive, Real-Time)

### How it works

ORCA (Optimal Reciprocal Collision Avoidance) is a decentralized algorithm where each
agent independently computes a collision-free velocity by treating other agents as
moving obstacles defined in velocity space.

Each agent:
1. Observes other agents' current positions and velocities
2. Computes the set of velocities that would cause a collision within time horizon T
3. Takes a half-share of responsibility for avoiding: picks a velocity outside the
   combined "forbidden" region that is closest to its desired velocity
4. Applies that velocity

The "reciprocal" part means: if Agent A adjusts its velocity to avoid Agent B, Agent B
also adjusts by the same amount to avoid Agent A. Neither agent has to do all the work.

A 2025 Scientific Reports paper shows a DWA-ORCA fusion algorithm with 48.75 ms average
planning cycle — well within real-time requirements at 2 m/s flight speed.

### Implementation in ROS 2

```python
import numpy as np

class ORCAAgent:
    def __init__(self, drone_id, radius=0.5, time_horizon=5.0):
        self.id = drone_id
        self.radius = radius          # drone physical radius in metres
        self.time_horizon = time_horizon  # seconds to look ahead
        self.max_speed = 2.0          # m/s

    def compute_new_velocity(self, position, velocity, preferred_velocity,
                              other_agents):
        """
        position: np.array [x, y]
        velocity: np.array [vx, vy]
        preferred_velocity: np.array — where we want to go
        other_agents: list of (pos, vel) tuples from comms
        """
        orca_lines = []

        for other_pos, other_vel in other_agents:
            rel_pos = other_pos - position
            rel_vel = velocity - other_vel
            dist = np.linalg.norm(rel_pos)
            combined_radius = 2 * self.radius  # both drones

            # Combined radius check
            if dist < combined_radius:
                # Already overlapping — emergency diverge
                orca_lines.append(self._emergency_line(rel_pos))
                continue

            # Velocity obstacle
            u = self._compute_u(rel_pos, rel_vel, combined_radius)
            normal = u / np.linalg.norm(u)
            orca_lines.append((normal, other_vel + 0.5 * u))

        # Linear programming to find closest feasible velocity
        new_vel = self._linear_program(orca_lines, preferred_velocity, self.max_speed)
        return new_vel
```

This runs on the RPi companion computer, outputs a desired velocity, and sends it to
ArduPilot via `SET_POSITION_TARGET_LOCAL_NED` in velocity mode.

### Data requirement
ORCA requires each drone to know the **position and velocity** of every other drone.
This requires comms (WiFi, UWB, or equivalent). Without comms, ORCA reduces to a pure
geometry avoidance with assumed constant velocity of neighbours (less reactive).

### Complexity: High
Requires: comms for position sharing, ROS 2 node for ORCA computation, ArduPilot
velocity target mode integration, parameter tuning.

### Latency requirements
ORCA must run at ≥10 Hz. At 2 m/s, drone moves 0.2 m between updates — acceptable.
WiFi latency: ~20 ms. ORCA compute time: ~1–5 ms. Total lag: ~25 ms. At 2 m/s: 0.05 m
— negligible.

### Failure modes
1. Comms loss → ORCA works on stale positions → may not avoid correctly
2. Stale position data + prediction disagreement → oscillation (two drones avoiding
   each other's ghost positions)
3. ORCA assumes all agents are computing ORCA. If one drone fails and freezes, it appears
   as a stationary obstacle — ORCA still handles this, just as a static obstacle.

### Verdict
**Implement for Grand Finale if comms is available.** The 2025 research confirms ORCA
is practical on small processors. Combined with geographic zones (Proposal C) as the
outer boundary, ORCA handles fine-grained real-time avoidance within those zones.

---

## Proposal E: Potential Field Repulsion (via UWB positions)

### How it works

Each drone computes an artificial repulsive force from its neighbours. The force is
inversely proportional to distance: close drones feel strong repulsion, far ones feel
negligible repulsion. This force is added to the drone's planned velocity vector.

```python
def repulsion_force(my_pos, other_positions, min_dist=2.0, k_rep=1.0):
    """
    k_rep: repulsion gain
    min_dist: minimum safe distance in metres
    """
    total_force = np.zeros(2)
    for other_pos in other_positions:
        delta = my_pos - other_pos
        dist = np.linalg.norm(delta)
        if dist < min_dist and dist > 0:
            # Repulsion proportional to (1/dist - 1/min_dist)
            magnitude = k_rep * (1.0/dist - 1.0/min_dist) / (dist**2)
            direction = delta / dist
            total_force += magnitude * direction
    return total_force

def blend_velocities(planned_vel, repulsion, alpha=0.3):
    """Blend planned velocity with repulsion correction"""
    return planned_vel + alpha * repulsion
```

The result is fed as a velocity correction to ArduPilot.

### Difference from ORCA

ORCA is mathematically guaranteed to be collision-free (given sufficient time horizon).
Potential fields are NOT guaranteed — they can create deadlocks where two drones repel
each other equally and get stuck. However, potential fields are simpler to tune and
understand.

### Complexity: Medium
Simpler math than ORCA. Same comms requirements (need neighbour positions).

### Failure modes
1. Deadlock: two drones approach head-on → equal repulsion → neither moves → stuck
2. Local minima: drone gets "trapped" between two repulsion sources
3. Oscillation: gains too high → drone jiggles back and forth
4. Without comms: must estimate neighbour positions from acoustic/visual sensing (very hard)

### Verdict
Use as an ORCA fallback or debugging tool. ORCA is strictly better given similar
complexity. If you're implementing ORCA, you don't need potential fields.

---

## Summary

| Proposal | Comms needed | Complexity | Guaranteed safe? | Notes |
|---|---|---|---|---|
| A: Altitude separation | None | Very low | ~Yes (laser alt) | Always implement |
| B: Time separation | None | Very low | Yes | Incompatible with time budget |
| C: Geographic fences | None | Low | ~Yes | Always implement as outer layer |
| D: ORCA | Required | High | Yes (mathematically) | Add for Grand Finale |
| E: Potential fields | Required | Medium | No (deadlock risk) | Use only if ORCA too complex |

---

## Recommended implementation stack

**Layer 1 (always on, no comms):**
- Altitude separation: Drone 1 = 2.0 m, Drone 2 = 2.5 m, Drone 3 = 3.0 m (laser-based)
- Staggered launch: 10-second intervals so all drones aren't at the same X position

**Layer 2 (fence, always on):**
- ArduPilot polygon fence limits each drone to its Y strip ± 0.5 m
- Fence breach: software fence in local NED first; any hardware fence action should be LAND/hold, not RTL (see caution in Proposal C)

**Layer 3 (for Grand Finale, if WiFi comms works):**
- ORCA velocity obstacle on each RPi at 10 Hz
- WiFi position sharing between drones

These three layers are independent. Any one of them failing still leaves two others
active. The probability of a collision with all three layers is extremely low.

---

## References

1. ORCA algorithm original: Van den Berg et al. (2011), ICRA.

2. DWA-ORCA fusion for multi-UAV (2025 Scientific Reports):
   https://www.nature.com/articles/s41598-025-99111-8

3. Hierarchical ORCA for UAVs with velocity optimization:
   https://www.sciencedirect.com/science/article/abs/pii/S0957417425028210

4. V-RVO: Voronoi + reciprocal velocity obstacles for decentralized avoidance:
   https://arxiv.org/pdf/2102.13281

5. ArduPilot geofence documentation:
   https://ardupilot.org/copter/docs/common-geofencing.html
