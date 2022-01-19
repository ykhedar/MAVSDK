#!/usr/bin/env sh

set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path/to/Firmware>"
    exit 1
fi

if [ "${PX4_VERSION}" ]; then
    echo "PX4 Autopilot Version Specified: " ${PX4_VERSION}
    PX4_FIRMWARE_DIR=$1
elif [ "${APM_VERSION}" ]; then
    echo "Ardupilot Autopilot Version Specified: " ${APM_VERSION}
    APM_FIRMWARE_DIR=$1
else
    echo "No Autopilot Version specified. Exiting."
    exit 1
fi

NPROCS=$(nproc --all)


cmake -DCMAKE_BUILD_TYPE=Debug -DASAN=ON -DUBSAN=ON -DLSAN=ON -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -j $NPROCS -Bbuild/debug -H.;
cmake --build build/debug -- -j $NPROCS;

if [ "${PX4_VERSION}" ]; then
    echo "PX4 Autopilot Version Specified: " ${PX4_VERSION}
    PX4_SIM_SPEED_FACTOR=10 AUTOSTART_SITL=1 PX4_FIRMWARE_DIR=$PX4_FIRMWARE_DIR HEADLESS=1 build/debug/src/integration_tests/integration_tests_runner --gtest_filter="SitlTest.*"
else
    echo "Ardupilot Autopilot Version Specified: " ${APM_VERSION}
    SIM_SPEEDUP=10 AUTOSTART_SITL=1 APM_FIRMWARE_DIR=$APM_FIRMWARE_DIR HEADLESS=1 build/debug/src/integration_tests/integration_tests_runner --gtest_filter="SitlTest.Mission*"
fi



# 3. Info // [09:32:51|Warn ] command unsupported (528). (mavlink_commands.cpp:187)
# [09:32:51|Warn ] Dropping command 528 that is already being sent (mavlink_commands.cpp:111)
# [09:32:51|Warn ] Dropping command 520 that is already being sent (mavlink_commands.cpp:111)
# 5. Param*

# 4. Mission*

# 1. Action*
# 2. GeofenceInclusion
# 7. Telemetry*
# 6. SystemConnectionAsync


# SitlTest.
#   ActionHoverAsync
#   ActionHoverSyncDefault
#   ActionHoverSyncHigher
#   ActionHoverSyncLower
#   ActionTakeoffAndKill
#   ActionTransitionSync_standard_vtol
#   ActionGoto
#   ActionHold
#   FollowMeOneLocation
#   FollowMeMultiLocationWithConfig
#   GeofenceInclusion
#   Info
#   MissionUploadCancellation
#   MissionDownloadCancellation
#   MissionChangeSpeed
#   MissionAddWaypointsAndFly
#   MissionWithRTLHigh
#   MissionWithRTLLow
#   MissionWithRTLHigherAnyway
#   MissionRawMissionChanged
#   MissionRawImportAndFly
#   MissionSetCurrent
#   OffboardAccelerationNED
#   OffboardVelocityNED
#   OffboardVelocityBody
#   OffboardPositionNED
#   ParamSad
#   ParamHappy
#   SystemConnectionAsync
#   TelemetryAsync
#   TelemetryHealth
#   TelemetrySync
#   MavlinkPassthrough
#   MissionTransferLossy



