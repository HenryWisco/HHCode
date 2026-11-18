#!/bin/sh
# Builds and runs the simulator tests (Linux, macOS, Git Bash, GitHub Actions).
# Needs a C++17 compiler. Uses g++ unless CXX says otherwise:
#
#   sh tests/run.sh
#   CXX=clang++ sh tests/run.sh
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
mkdir -p build/sim

# Windows (Git Bash) can only run programs whose names end in .exe.
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) exe=.exe ;; *) exe= ;; esac

# On GitHub, failures are also written as annotations so they show on the
# run's summary page.
annotate() {
  if [ -n "$GITHUB_ACTIONS" ]; then
    grep -E "$1" "$2" | head -10 | while read -r line; do echo "::error::$line"; done
  fi
}

status=0
for config in 1 2 3 4; do
  echo "=== Robot $config: building"
  # -fpermissive: JAR-Template names a member the same as its type
  # (drive_setup), which g++ otherwise refuses.
  if ! $CXX -std=gnu++17 -O1 -pthread -fpermissive -w -Dmain=robot_main -DSIM_CONFIG=$config \
      -Itests/sim/vex -Itests/sim -Iinclude \
      src/JAR-Template/*.cpp src/hh/*.cpp src/autons/*.cpp src/main.cpp tests/sim/*.cpp \
      -o build/sim/sim$config$exe > build/sim/build$config.log 2>&1; then
    cat build/sim/build$config.log
    annotate 'error' build/sim/build$config.log
    status=1
    continue
  fi
  echo "=== Robot $config: running"
  ./build/sim/sim$config$exe > build/sim/run$config.log 2>&1 || status=1
  cat build/sim/run$config.log
  annotate 'FAIL line|SIM|No test' build/sim/run$config.log
done

if [ $status -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "SOME TESTS FAILED"; fi
exit $status
