#!/usr/bin/env bash
# Run all tests from the alpha/ directory.
cd "$(dirname "$0")"
python -m pytest \
    shm/tests/ \
    vicon/tests/ \
    mavlink/tests/ \
    operator/tests/ \
    trajectories/tests/ \
    tests/ \
    -v "$@"
