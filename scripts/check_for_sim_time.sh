#!/bin/bash
# Timeout in seconds for parameter queries
TIMEOUT=${1:-2}

echo "=== ROS2 use_sim_time Parameter Report ==="
echo "=========================================="
echo "Timeout: ${TIMEOUT}s per node"
echo ""

# Get list of all nodes
nodes=$(ros2 node list)

# Check if any nodes were found
if [ -z "$nodes" ]; then
    echo "No ROS2 nodes found running."
    exit 0
fi

# Iterate through each node
while IFS= read -r node; do
    echo "Node: $node"

    # Try to get the use_sim_time parameter with timeout
    # Use timeout command to prevent hanging on unresponsive nodes
    param_value=$(timeout "${TIMEOUT}" ros2 param get "$node" use_sim_time 2>&1)
    exit_code=$?

    # Check if the command timed out
    if [ $exit_code -eq 124 ]; then
        echo "  use_sim_time: TIMEOUT (node did not respond)"
    # Check if the parameter exists (successful get vs error)
    elif echo "$param_value" | grep -q "Parameter not found"; then
        echo "  use_sim_time: not present"
    elif echo "$param_value" | grep -q "Set parameter successful"; then
        # Sometimes get returns this message, need to handle
        echo "  use_sim_time: not present"
    else
        # Extract just the value from the output
        value=$(echo "$param_value" | grep -oP "(?<=value: ).*" | head -1)
        if [ -z "$value" ]; then
            # Fallback: sometimes the format is different
            value=$(echo "$param_value" | tail -1)
        fi
        echo "  use_sim_time: $value"
    fi

    echo ""
done <<< "$nodes"

echo "=========================================="
echo "Report complete."
