#!/bin/bash

# Path to the healthcheck script
HEALTHCHECK_SCRIPT="/roscore/health-check.sh"

# Read event from stdin
while read line; do
    # Check for the event that indicates App1 has changed state
    if echo "$line" | grep -q "PROCESS_STATE"; then
        # Execute the health check script
        bash "$HEALTHCHECK_SCRIPT"
    fi
done
