#!/usr/bin/env sh

set -e

NPROCS=$(nproc --all)


cmake -DCMAKE_BUILD_TYPE=Debug -DASAN=OFF -DUBSAN=OFF -DLSAN=OFF -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -j $NPROCS -Bbuild/debug -H.;
cmake --build build/debug -- -j $NPROCS;

cd build/debug
make install

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/


cd /home/user/MAVSDK/examples/takeoff_and_land_apm
cmake -Bbuild -H.
cmake --build build