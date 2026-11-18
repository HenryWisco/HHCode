#!/bin/sh
# Updates the HHCode library in a team's folder from a newer copy of the
# template, without touching the team's own files. Mac / Linux version.
#
# 1. Unzip the new template somewhere (for example Downloads/HHCode-new).
# 2. Open YOUR TEAM'S folder in VS Code and open a terminal (Terminal > New Terminal).
# 3. Run:   sh tools/update-library.sh ~/Downloads/HHCode-new
#
# Replaced:  the library (include/hh, src/hh, include/JAR-Template,
#            src/JAR-Template), the VEX build files, the docs, the tests and
#            these tools.
# Kept:      src/robot-config.cpp, include/robot-config.h, src/main.cpp,
#            include/autons.h, your autons and PID constants in src/autons,
#            and everything in the subsystems folders.
#
# The old library is saved in a folder named _library-backup-<date> first.
set -e

if [ -z "$1" ]; then
  echo "Usage: sh tools/update-library.sh <folder of the new template>"
  exit 1
fi
team="$(cd "$(dirname "$0")/.." && pwd)"
new="$(cd "$1" && pwd)"

folders="include/hh src/hh include/JAR-Template src/JAR-Template vex docs tests/sim tools"
files="makefile include/vex.h src/autons/tests.cpp tests/README.md tests/run.ps1 tests/run.sh README.md CLAUDE.md"
watch="src/main.cpp src/autons/auton_list.cpp src/autons/constants.cpp include/autons.h include/robot-config.h src/robot-config.cpp"

if [ ! -f "$new/makefile" ] || [ ! -d "$new/include/hh" ]; then
  echo "That doesn't look like an HHCode template folder: $new"
  echo "Pick the folder that has 'makefile' and 'include' directly inside it."
  exit 1
fi
if [ "$new" = "$team" ]; then
  echo "That is this same folder. Point it at the NEW template instead."
  exit 1
fi

backup="$team/_library-backup-$(date +%Y-%m-%d-%H%M)"
mkdir -p "$backup"

for item in $folders $files; do
  [ -e "$new/$item" ] || continue
  if [ -e "$team/$item" ]; then
    mkdir -p "$(dirname "$backup/$item")"
    cp -R "$team/$item" "$backup/$item"
    rm -rf "$team/$item"
  fi
  mkdir -p "$(dirname "$team/$item")"
  cp -R "$new/$item" "$team/$item"
  echo "updated  $item"
done

echo ""
echo "Kept your own files. These ones differ from the new template, which is"
echo "normal, but look at the new template's copy if the build fails:"
for item in $watch; do
  if [ -f "$new/$item" ] && [ -f "$team/$item" ]; then
    cmp -s "$new/$item" "$team/$item" || echo "  $item"
  fi
done
echo ""
echo "Done. Your old library is in $(basename "$backup")."
echo "Now click Build. Delete the backup folder once everything works."
