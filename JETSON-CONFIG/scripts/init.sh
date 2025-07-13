#!/bin/sh

# Wait for system to be ready
sleep 5

# Start D-Bus if not running
if ! pgrep -x "dbus-daemon" > /dev/null; then
    dbus-daemon --system --fork
fi

# Start NetworkManager if not running
if ! pgrep -x "NetworkManager" > /dev/null; then
    NetworkManager --no-daemon &
    sleep 3
fi

# Setup hotspot
/scripts/setup_hotspot.sh

echo "Hotspot setup complete"
