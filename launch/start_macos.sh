#!/usr/bin/env bash

set -e

# Check that Docker Desktop is running.
if ! docker info >/dev/null 2>&1; then
    echo "Docker Desktop is not running."
    exit 1
fi

# Allow local Docker connections to XQuartz.
if [ -x /opt/X11/bin/xhost ]; then
    /opt/X11/bin/xhost +localhost
else
    echo "XQuartz was not found in /opt/X11."
    exit 1
fi

# Start the CrazySim development container.
docker compose \
    -f compose.yaml \
    -f compose.macos.yaml \
    run --rm crazysim bash
