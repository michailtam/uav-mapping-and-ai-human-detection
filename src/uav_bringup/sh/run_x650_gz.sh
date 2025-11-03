#!/usr/bin/env bash
# ------------------------------------------------------------
# PX4 SITL launcher for the X650 model with Gazebo Harmonic
# Designed to be executed from a ROS 2 bringup launch file
# ------------------------------------------------------------

# Get all required directory paths
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Simulation configuration
export PX4_ROOT
export PX4_SYS_AUTOSTART=4229
export PX4_SIM_MODEL=x650
export PX4_GZ_WORLD=baylands
echo "export PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0"
export GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:${GZ_SIM_RESOURCE_PATH}"

# Debug info
echo "-------------------------------------------------"
echo " PX4 SITL launch (ROS 2 bringup)"
echo " PX4 root:   $PX4_ROOT"
echo " Airframe:   $PX4_SYS_AUTOSTART"
echo " Model:      $PX4_SIM_MODEL"
echo " World:      $PX4_GZ_WORLD"
echo " Spawn pose: $PX4_GZ_MODEL_POSE"
echo "-------------------------------------------------"

echo "[run_x650_gz.sh] Launching PX4 SITL..."
# exec ./bin/px4