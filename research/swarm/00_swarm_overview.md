# Swarm Coordination — Complete Overview

## Start here before reading the other swarm files

---

## Part 1 — What "swarm" means in this PS

The word "swarm" gets overloaded. In nature it means thousands of insects behaving
emergently. In Robofest 6.0, it means something much more specific:

**A minimum of 3 autonomous drones that together cover a 15 × 60 m field without
GPS, without any external computer, and without an operator sending commands during
the run — while not re-checking the same ground unnecessarily.**

That's it. The swarm doesn't need to be beautiful. It doesn't need flocking behaviour.
It needs to:

1. Divide a field into sections so all of it gets covered
2. Not fly into each other
3. Share detection results somehow so the path can be computed from all three maps
4. Adapt if one drone fails

---

## Part 2 — What the PS actually constrains

Read the PS carefully. The exact quotes that constrain swarm design:

| PS Clause | What it means for us |
|---|---|
| "minimum of 3 miniature drones" | Three real flying platforms, not simulated |
| "Communication between drones... permitted solely for swarm coordination" | Inter-drone comms IS allowed — but only for task allocation, formation, status |
| "No communication with external computers, cloud servers, base stations" | No ground laptop during the run, no wifi to a router outside the field |
| "license-exempt wireless bands... WPC India regulations" | 2.4 GHz, 5.8 GHz ISM bands are fine; 900 MHz LoRa is fine; licensed spectrum is not |
| "off-board computation... strictly prohibited" | All decisions must be made by the drone itself |
| "avoid redundancy in mine detection" | Drones must coordinate to avoid scanning the same area |
| "Generate and communicate a 1-meter clearance safe path" | The path output must exist and be shareable between drones |

The key liberating insight: **inter-drone comms is explicitly allowed for coordination**.
This is not a fully autonomous-independent-per-drone problem. The drones CAN talk to
each other. They just cannot talk to anything outside the swarm.

---

## Part 3 — What coordination actually needs to solve

Given 3 drones over a 15 × 60 m field, here are the concrete problems:

### 3.1 Coverage allocation
Who scans which part of the field? If this isn't decided upfront (or negotiated in
flight), two drones might scan the same strip while one third remains uncovered.

### 3.2 Collision avoidance
Three drones flying in a 15 m wide space at ~2 m altitude need to stay separated.
At 2 m/s, two drones approaching each other have ~2 seconds to react from 4 m away.
This is tight. Collision avoidance cannot be an afterthought.

### 3.3 Result aggregation
Drone A finds 5 mines in its strip. Drone B finds 7 mines. Drone C finds 3.
Who builds the unified map? When? How does the path get computed across all 15 mines?

### 3.4 Failure degradation
Drone 2 crashes at t=180 seconds. It has covered 30% of its strip. The remaining 70%
of that strip is now uncovered. Does the mission fail, or do drones 1 and 3 reroute?

### 3.5 Human tracking
Once the safe path is defined, one or more drones need to track the person as they
cross, guiding them visually. This means at least one drone must shift from scan mode
to escort mode at the right time.

---

## Part 4 — Architecture families

There are three broad families of swarm architecture, ranging from simplest to most complex:

### Family A: Pre-planned (no runtime coordination)
Each drone has a completely fixed flight plan baked in before launch. Drone 1 gets
strip 0–5 m, Drone 2 gets 5–10 m, Drone 3 gets 10–15 m. No inter-drone comms needed.
No coordination needed. If one drone fails, its strip simply doesn't get covered.

**Simplicity:** Maximum. **Robustness:** Low.

### Family B: Leader-follower (one master drone)
One "leader" drone coordinates the others. It gets status updates from the followers,
assigns tasks, and aggregates maps. If the leader crashes, the mission stalls.
Inter-drone comms is required.

**Simplicity:** Medium. **Robustness:** Medium (single point of failure at leader).

### Family C: Distributed peer-to-peer
All drones are equal. Each broadcasts its own status, reads others' statuses, and makes
local decisions about what to scan next (frontier-based). Maps are merged by consensus.
No single point of failure. Requires reliable mesh comms and more complex software.

**Simplicity:** Low. **Robustness:** High.

---

## Part 5 — Our current design (as of initial repo)

The existing `master/` and `slave/` code follows a modified **leader-follower** model:

```
master/app.py         → Flask HTTPS server on ground station PC
                        (NOT carried on a drone — this is a laptop)
slave/main_orchestrator.py → Each drone connects to master via TCP

Sequence:
  1. Master assigns strip boundaries to each slave on mission start
  2. Slaves fly their strips, report mine detections back to master TCP
  3. Master aggregates map, computes path, displays in dashboard
  4. Master sends path back to slaves for LED marking
```

**Problem:** The "master" is a laptop. This violates the PS rule: "no communication
with external computers". We need to move coordination entirely onto one of the drones
or go fully distributed.

**Options explored in subsequent files:**
- `01_coverage_patterns.md` — which scan pattern to use
- `02_inter_drone_comms.md` — what hardware handles drone-to-drone comms
- `03_collision_avoidance.md` — how drones stay separated
- `04_result_aggregation.md` — how to merge three drones' maps
- `05_swarm_failure_modes.md` — what breaks and how

---

## Part 6 — Scoring context

From the PS evaluation criteria (Grand Finale):

| Task | Points |
|---|---|
| Safe takeoff and swarm activation | 10 |
| Gesture/voice command recognition | 10 |
| **Swarm formation (in the sky)** | **10** |
| **Mine detection & correct mapping** | **25** |
| Safe path creation & marking | 15 |
| Person crosses safely | 20 |
| Time bonus | 10 |
| Collision penalty | -10 |
| Drone crash / surface contact | -20 |

Mine detection is the highest single task. A collision is -10. A crash is -20.

This means: **flying safely matters more than flying clever**. A simple pre-planned
swarm that doesn't crash beats a sophisticated adaptive swarm that has one mid-air
collision.

---

## Reading order

| File | What you'll learn |
|---|---|
| `01_coverage_patterns.md` | Five proposals for dividing the 15×60m field |
| `02_inter_drone_comms.md` | Five comms proposals: WiFi, UWB, LED, acoustic, pre-planned |
| `03_collision_avoidance.md` | Five collision avoidance approaches and their trade-offs |
| `04_result_aggregation.md` | How three drones' maps become one safe path |
| `05_swarm_failure_modes.md` | Failure scenarios and degradation strategies |

---

## Key vocabulary

| Term | Plain English |
|---|---|
| **Swarm** | Multiple drones working on the same mission with some form of coordination |
| **Coverage** | What fraction of the field has been scanned |
| **Redundancy** | Two drones scanning the same area — wastes time, PS penalises it |
| **Frontier** | The boundary between scanned and unscanned area — next place to go |
| **Leader-follower** | One drone coordinates, others receive instructions |
| **Peer-to-peer** | All drones equal, no single controller |
| **MAVLink** | The protocol ArduCopter uses for flight control commands |
| **Strip** | A rectangular section of field assigned to one drone |
| **Waypoint** | A GPS/local coordinate the drone flies to |
| **GUIDED mode** | ArduCopter flight mode where an onboard computer sends position targets |
