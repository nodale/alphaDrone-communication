#!/bin/bash
# Start all onboard processes in the background (development mode, no systemd)
# Run from repo root or alpha/

ALPHA_DIR="$(cd "$(dirname "$0")/../alpha" && pwd)"
PIDS_FILE="/tmp/alphadrone_pids"

if [ -f "$PIDS_FILE" ]; then
    echo "Processes may already be running (found $PIDS_FILE). Run dev_stop.sh first."
    exit 1
fi

cd "$ALPHA_DIR"

echo "Starting processes..."
pids=()

# Creators of shared memory must start first
python vicon_collector.py &;    pids+=($!); echo "vicon_collector      PID $!"
python estimator_collector.py & pids+=($!); echo "estimator_collector  PID $!"

sleep 1  # give SHM creators time to initialize

python triangulator.py &        pids+=($!); echo "triangulator         PID $!"
python hongi_process.py &       pids+=($!); echo "hongi_process        PID $!"
python active_com.py &          pids+=($!); echo "active_com           PID $!"
python baby_sitter.py &         pids+=($!); echo "baby_sitter          PID $!"
python -m nn.inferrer &         pids+=($!); echo "nn.inferrer          PID $!"
python -m nn.online_learner &   pids+=($!); echo "nn.online_learner    PID $!"

printf "%s\n" "${pids[@]}" > "$PIDS_FILE"
echo ""
echo "All started. PIDs saved to $PIDS_FILE"
echo "Run scripts/dev_stop.sh to stop."
