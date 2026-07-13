#!/bin/bash
# Stop all processes started by dev_start.sh

PIDS_FILE="/tmp/alphadrone_pids"

if [ ! -f "$PIDS_FILE" ]; then
    echo "No PID file found at $PIDS_FILE. Nothing to stop."
    exit 0
fi

echo "Stopping processes..."
while read -r pid; do
    if kill -0 "$pid" 2>/dev/null; then
        kill "$pid" && echo "Killed PID $pid"
    else
        echo "PID $pid already gone"
    fi
done < "$PIDS_FILE"

rm "$PIDS_FILE"
echo "Done."
