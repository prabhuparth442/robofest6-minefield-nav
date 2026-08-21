#!/bin/bash
# Run this once from your Mac Terminal to commit the 16 research files.
# The sandbox couldn't delete the stale git lock files; this script does it locally.

REPO="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO"

echo "Removing stale lock files..."
rm -f .git/index.lock .git/HEAD.lock

echo "Staging all research files..."
git add research/swarm/ research/detection/ research/code_architecture/

echo "Committing..."
git commit -m "Research: swarm, detection, and code architecture deep-dives

- research/swarm/ (5 files): coverage patterns, comms, ORCA collision avoidance,
  result aggregation, failure modes
- research/detection/ (6 files): YOLOv8n benchmarks, MLX90640 FOV math,
  sensor fusion, altitude/speed analysis, mine mapping
- research/code_architecture/ (5 files): ROS2 vs MAVLink, C++ vs Python,
  onboard compute comparison, full software architecture, simulation proposals"

echo ""
git log --oneline
