# 02 — Inter-Drone Communications

## What the PS allows, what's available, and what to actually use

---

## The core question

The PS says drones can communicate with each other. It says they cannot communicate
with anything external. But it does NOT define precisely where "the swarm" ends and
"external" begins.

The relevant PS clause verbatim:

> "Communication between drones of the same team is permitted solely for swarm
> coordination (e.g., task allocation, formation control, and status sharing)
> using license-exempt wireless bands in accordance with the prevailing WPC,
> Government of India regulations."

And:

> "No communication with external computers, cloud servers, base stations, or
> remote operators shall be permitted during the competition."

So the test is: **is the communication happening between two drones, or between a
drone and something outside the swarm?**

A laptop on the ground = external. ❌  
A radio strapped to one of the drones = part of the drone. ✓  
An ESP32 tied to a pole = base station = external. ❌  
A Raspberry Pi on Drone 2 routing data to Drone 3 = intra-swarm. ✓  

---

## Proposal A: Pre-planned (no runtime comms)

### How it works

No wireless hardware is used for coordination at all. Each drone has a fully baked
flight plan written before launch. Coverage strips are pre-assigned. Each drone flies
independently.

"Coordination" is done entirely before the mission: you decide who scans what, encode
it as waypoints, load it to each drone's RPi, and launch.

### Legality under PS rules
**100% legal.** No comms = no comms violation. The TAC has nothing to inspect.

### Latency
N/A — all decisions are pre-computed.

### Bandwidth
N/A.

### Implementation complexity
**Lowest possible.** Each drone's `main_orchestrator.py` gets a hardcoded strip
boundary and runs its waypoint list autonomously.

```python
# Drone 1 config
STRIP_Y_MIN = 0.0
STRIP_Y_MAX = 5.0
SCAN_ALTITUDE = 2.0
SCAN_SPEED = 1.5   # m/s
```

### Failure modes
1. Drone 2 crashes → its strip unscanned, no recovery
2. Navigation drift → drone wanders into adjacent strip
3. No result aggregation during flight → final map must be assembled post-mission
   (violates the "generate and communicate a safe path" objective during the run)

### Fatal flaw for our PS

The PS requires the safe path to be **generated and communicated** (presumably to the
person-at-risk visually during the run). If each drone only knows its own scan sector,
no single drone has the complete map. The person can only be guided across the full
60 m if one entity knows all mines in all three strips.

**Workaround:** Designate one "master drone" that receives all three drones' findings.
But then that master drone needs to receive data from the other two — which requires
comms. So pure pre-planned only works if each drone independently marks its own strip's
path, and the human follows all three drones simultaneously. This is possible but messy
operationally.

### Verdict
Use as fallback if comms hardware fails. Not sufficient for the full marking + guidance
task.

---

## Proposal B: Ad-hoc WiFi Mesh (Raspberry Pi / ESP32)

### How it works

Each drone carries a Raspberry Pi with its built-in WiFi adapter (or a dedicated ESP32
module). The three RPis form an ad-hoc WiFi network — a peer-to-peer mesh with no
access point.

On Linux, this is straightforward:

```bash
# On each RPi — set up ad-hoc network
sudo iw dev wlan0 set type ibss
sudo ip link set wlan0 up
sudo iw dev wlan0 ibss join SWARM_NET 2412
sudo ip addr add 192.168.99.x/24 dev wlan0
```

Or use hostapd on one drone to act as a soft AP, and other drones connect to it.

Data shared: mine detections (x, y, confidence), coverage map diffs, status heartbeats.

### Legality analysis

This is the critical question. The PS says "license-exempt wireless bands."

**WiFi (2.4 GHz, 802.11b/g/n) IS a license-exempt band under WPC India.** The 2.4 GHz
ISM band (2400–2483.5 MHz) does not require a license for low-power devices in India
as per WPC exemption notifications.

However, the PS also says "no external computers." An RPi on a drone is part of the
drone, not external. So drone-to-drone WiFi (ad-hoc, no router) is arguably legal.

**Risk:** TAC might interpret "WiFi" as needing a "base station" to route through. An
ad-hoc (IBSS mode) WiFi network has no base station — it's pure peer-to-peer. Make
this argument clearly in your technical documentation.

### Latency
WiFi 2.4 GHz ad-hoc: **5–20 ms typical** for a 100-byte UDP packet.
At 2 m/s flight speed: in 20 ms, a drone moves 0.04 m — negligible for our use case.

### Bandwidth
~20–54 Mbps theoretical. We only need ~10 kbps for status + mine detection packets.
No bandwidth concern whatsoever.

### Implementation complexity
**Medium.** Setting up ad-hoc WiFi on RPi is well documented. The tricky part is
reliability: WiFi can drop in outdoor environments with multipath and interference.

```python
# Simple UDP broadcast for mine detection
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def broadcast_mine(x, y, confidence):
    msg = f"MINE:{x:.2f}:{y:.2f}:{confidence:.2f}"
    sock.sendto(msg.encode(), ('192.168.99.255', 5005))

def listen_for_mines():
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(('', 5005))
    while True:
        data, addr = recv_sock.recvfrom(256)
        parse_mine_message(data.decode())
```

### Failure modes
1. WiFi interference from other teams' equipment → packet loss
2. Range limitation: RPi WiFi adapter at 2.4 GHz in open field: ~30–50 m
   (field is 60 m long — potential dead zone at far end)
3. If one drone is at far end and another at near end: 60+ m range may be marginal
4. TAC disqualification if they rule WiFi violates "no external comms" spirit

### Verdict
**Best practical option if range holds.** Use UDP broadcast, 10 Hz heartbeats. Test
range on actual outdoor field before competition. The legal argument is solid for
ad-hoc mode.

---

## Proposal C: UWB Peer-to-Peer Ranging (DW3000)

### How it works

Ultra-Wideband (UWB) uses very short radio pulses across a wide frequency band to
measure the time-of-flight between two transceivers with ≤10 cm accuracy. You've
already researched UWB in the `gps_denied/04_uwb_ranging.md` context.

For inter-drone comms, UWB can do two things:
1. **Ranging only** — measure distance between two drones (passive, no data payload)
2. **Data + ranging** — DW3000 supports sending small data payloads alongside range

The DW3000 (Qorvo/Decawave) chip supports up to 6.8 Mbps data rate with ±10 cm
ranging. Modules like Makerfabs UWB Module weigh ~5 g.

### Legality analysis

UWB operates in the 6–8.5 GHz band. In India, UWB for short-range indoor/outdoor use
is in a **grey area** under WPC regulations as of 2025. The WPC has not issued clear
exemption notifications for UWB in the same way as WiFi/Bluetooth. The PS says
"license-exempt wireless bands" — UWB may not qualify.

**Risk: MEDIUM-HIGH.** If TAC inspects comms hardware and sees UWB transceivers, they
may not be able to confirm it's WPC-compliant. Prepare documentation.

### Latency
~1–2 ms for a ranging exchange. Data packet: ~5–10 ms.

### Bandwidth
~1–6.8 Mbps — far more than needed for mine detection packets.

### Implementation complexity
**Medium-High.** DW3000 requires SPI interface and a custom MAC protocol to handle
multi-drone ranging without collision (need TDMA scheduling).

### Failure modes
1. WPC compliance risk → disqualification
2. DW3000 range in outdoor field: ~50–100 m line-of-sight — adequate for 60 m field
3. At 2 m altitude, drones are above each other's near-ground multipath zone — good

### Verdict
**Ideal technically, risky legally.** If you can get a WPC compliance ruling before
the competition, this is the cleanest solution. Otherwise, use WiFi (Proposal B).

---

## Proposal D: Visible Light Communication (LED blink codes)

### How it works

Each drone has a high-brightness LED on top (pointing upward) and a camera on another
drone looking at it. Data is encoded in blink patterns: e.g., a short-long-short
pattern encodes a mine detection. This is Optical Wireless Communication (OWC) or
Li-Fi at very low bandwidth.

Example: blink at 10 Hz → 10 bits/second. To transmit an (x, y) mine position:
- x = 0–60 m, resolution 0.5 m → 7 bits
- y = 0–15 m, resolution 0.5 m → 5 bits
- confidence = 3 levels → 2 bits
- Total: 14 bits = 1.4 seconds per mine detection

### Legality
**Legally unambiguous.** Light is not a radio band. No WPC regulations apply.
The PS says "wireless bands" for the comms restriction, but visible light is not
a radio band in any regulatory sense. This is the safest option legally.

### Latency
~1.4 seconds per mine report. If 15 mines are found: ~21 seconds total for all reports.
Fine for our use case (mines are found during flight, not all at once).

### Bandwidth
~10–100 bits/second depending on blink frequency. Sufficient for mine coordinates only.

### Implementation complexity
**High.** Requires: upward-facing LED on transmitting drone, downward or side-facing
camera on receiving drone, computer vision to detect and decode blink patterns, and
a protocol to handle 3 drones potentially blinking simultaneously.

```python
# LED blink encoding (on transmitting drone)
import RPi.GPIO as GPIO
import time

LED_PIN = 18

def blink_bits(bits):
    for bit in bits:
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.05 if bit else 0.02)  # long=1, short=0
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(0.02)

def encode_mine(x, y):
    x_bits = format(int(x / 0.5), '07b')
    y_bits = format(int(y / 0.5), '05b')
    return x_bits + y_bits
```

### Failure modes
1. Direct sunlight washes out LED signals (outdoor field in Gujarat — likely problem)
2. Drones must be in line-of-sight of each other at camera angle
3. Camera decoder must track LED position in noisy outdoor background
4. At 60 m separation, a 5 mm LED is effectively a single pixel — signal is tiny

### Verdict
**Legally elegant, practically difficult outdoors.** The solar interference problem
in a Gujarat outdoor field (bright sunlight) is likely fatal. Useful as a backup for
short-range status signals only.

---

## Proposal E: Acoustic Communication (Ultrasonic)

### How it works

Ultrasound (>20 kHz, inaudible) can carry data over short distances. Each drone has
a piezo transducer transmitting and receiving modulated ultrasonic pulses.

Typical modules: HC-SR04 operates at 40 kHz. With proper modulation (FSK), data rates
of ~100–500 bps are achievable at 1–5 m range.

### Legality
**Unambiguous.** Sound waves are not radio waves. No WPC restrictions.

### Latency
~5–50 ms depending on encoding and range.

### Bandwidth
100–500 bps — sufficient for mine coordinate packets.

### Implementation complexity
**Very High.** The HC-SR04 is not designed as a data transceiver — it's a ranging
sensor. Building a true acoustic data link requires custom hardware, carrier frequency
modulation, error correction, and the entire protocol stack. This is research-grade
hardware work.

### Fatal problem
**Propeller noise.** Drone motors at 2 m/s generate significant acoustic noise in the
15–40 kHz range, directly competing with ultrasonic communication. Signal-to-noise
ratio is essentially zero when motors are running.

### Verdict
**Not feasible.** Motor noise overwhelms acoustic signals at all practical drone
operation speeds. Eliminate this option.

---

## Summary comparison

| Proposal | Legality | Latency | Range | Complexity | Verdict |
|---|---|---|---|---|---|
| A: Pre-planned | ✅ No risk | N/A | N/A | Very low | Fallback only |
| B: WiFi ad-hoc | ✅ Likely OK | 5–20 ms | 30–50 m | Medium | **Use this** |
| C: UWB DW3000 | ⚠️ Grey area | 1–10 ms | 50–100 m | Medium-high | If WPC clears it |
| D: LED blink | ✅ No risk | ~1.4 s/msg | ~20 m | High | Outdoor impractical |
| E: Acoustic | ✅ No risk | 5–50 ms | 1–5 m | Very high | Motor noise kills it |

---

## Recommended protocol

**Use WiFi ad-hoc (Proposal B) as primary, pre-planned (Proposal A) as fallback.**

### Data packets (UDP broadcast, port 5005)

```
HEARTBEAT packet (10 Hz per drone):
  drone_id:1  type:HB  x:12.3  y:4.5  z:2.0  status:SCAN

MINE packet (on detection):
  drone_id:1  type:MINE  x:23.1  y:6.2  conf:0.87

COVERAGE packet (1 Hz):
  drone_id:1  type:COV  strip_pct:45  cells_scanned:2340

PATH packet (when computing safe path):
  drone_id:1  type:PATH  nodes:[[0,7.5],[10,7.2],[20,8.1],...]
```

If a drone misses 3 consecutive heartbeats from another drone, it assumes that drone
has failed and expands its own scan zone accordingly (fallback to solo).

---

## References

1. WPC India license-exempt bands notification:
   https://dot.gov.in/wireless-planning-coordination

2. ArduPilot companion computer communication guide:
   https://ardupilot.org/dev/docs/companion-computers.html

3. Cooperative UAV swarm communication (GPS-denied):
   https://www.mdpi.com/2504-446X/10/5/355

4. DW3000 UWB module datasheet (Qorvo):
   https://www.qorvo.com/products/p/DW3000
