#!/usr/bin/env bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/dual_fr3_maniskill_ros_logs
export MPLCONFIGDIR=/tmp/dual_fr3_mpl
exec /usr/bin/python3 src/dual_fr3_maniskill/test/run_integration.py "$@"
