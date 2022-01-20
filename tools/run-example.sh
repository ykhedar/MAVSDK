#!/usr/bin/env sh

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/


/home/user/MAVSDK/examples/takeoff_and_land_apm/build/takeoff_and_land_apm udp://:14540
