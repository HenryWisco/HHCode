#!/bin/sh
# Builds and runs the simulator tests (Linux, macOS, Git Bash, GitHub Actions).
# Needs a C++17 compiler. Uses g++ unless CXX says otherwise:
#
#   sh tests/run.sh
#   CXX=clang++ sh tests/run.sh
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
mkdir -p build/sim

status=0
for config in 1 2 3 4; do
  echo "=== Robot $config: building"
  $CXX -std=gnu++17 -O1 -pthread -Dmain=robot_main -DSIM_CONFIG=$config \
    -Itests/sim/vex -Itests/sim -Iinclude \
    src/JAR-Template/*.cpp src/hh/*.cpp src/autons/*.cpp src/main.cpp tests/sim/*.cpp \
    -o build/sim/sim$config
  echo "=== Robot $config: running"
  ./build/sim/sim$config || status=1
done

if [ $status -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "SOME TESTS FAILED"; fi
exit $status
