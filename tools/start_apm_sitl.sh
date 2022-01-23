#!/usr/bin/env bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

set -e

apm_firmware_dir=/home/user/ArduPilot
echo "DEBUG: APM_FIRMWARE_DIR: " ${apm_firmware_dir}
# Make sure everything is stopped first.
${script_dir}/stop_px4_sitl.sh
echo "DEBUG: script_dir: " ${script_dir}
# To prevent any races.
sleep 1

current_dir=$(pwd)
log_dir=$current_dir/logs
echo "DEBUG: log_dir: " ${log_dir}

# Create log dir in case it doesn't exist yet
mkdir -p $log_dir

# No user input needed, run it in deamon mode.
# export NO_PXH=1

# The logs are saved with a timestamp.
timestamp=`date +%Y-%m-%dT%H-%M-%S%z`

# Before changing dir, save where we start from.
pushd .

APM_HOME_LAT=${APM_HOME_LAT:-47.397742}
APM_HOME_LONG=${APM_HOME_LONG:-8.545594}
APM_HOME_ALT=${APM_HOME_ALT:-488}
APM_HOME_DIR=${APM_HOME_DIR:-180}

# Go to Firmware build dir.
cd $apm_firmware_dir
echo "DEBUG: current dir: " $(pwd)
echo "DEBUG: Launching Screen"
screen -dmS ardupilot python3 ./Tools/autotest/sim_vehicle.py \
        --vehicle Copter \
        -I0 \
        --custom-location=${APM_HOME_LAT},${APM_HOME_LONG},${APM_HOME_ALT},${APM_HOME_DIR} \
        -w \
        --frame quad \
        --no-rebuild \
        --speedup 1 \
        -m "--out=udp:127.0.0.1:14540 --out=udp:localhost:14550"

# Go back to dir where we started
popd

echo "waiting for SITL to be running"
sleep 10
echo "done"
