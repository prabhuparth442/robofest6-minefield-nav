# Gujcost Robofest 6.0

New PS codebase. Built on SpeedyBee F405 + ArduCopter (ArduPilot stack).

## Mission
GPS-denied indoor navigation, thermal anomaly detection, 4-pass swarm coverage.
Master-slave MAVLink architecture. Optical flow primary positioning (Matek 3901-L0X).

## Stack
- FC: SpeedyBee F405 Wing V3 running ArduCopter
- Comms: pymavlink / MAVProxy (GUIDED mode)
- Positioning: EKF3 + optical flow (GPS-denied)
- Thermal: MLX90640 on Raspberry Pi (slave)
- Swarm: 4-drone master-slave via MAVLink

## Structure
- slave/       — RPi flight code (to be written)
- master/      — Ground control (to be written)
- research/    — GPS-denied nav, swarm, thermal research
- migration/   — ArduPilot setup and transition docs
- hardware/    — SpeedyBee F405 hardware docs
- docs/        — Architecture, field layout, competition brief
- scripts/     — Daily push automation

## Predecessor
5.0 codebase: ../Gujcost-Robofest-Resource-IITBBS/
# robofest6-minefield-nav
