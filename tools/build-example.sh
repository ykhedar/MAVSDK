#!/usr/bin/env sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/


cd /home/user/MAVSDK/examples/takeoff_and_land_apm
cmake -Bbuild -H.
cmake --build build