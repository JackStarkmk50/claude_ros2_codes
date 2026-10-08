#!/usr/bin/env bash
# Workspace check: clean build from scratch, then verify packages, interfaces, executables and tests.
# Touches nothing outside this workspace. Does not start MAVROS or talk to the Pixhawk.
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

echo "== 1. clean build (removes build/ install/ log/)"
rm -rf build install log
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source "$WS_DIR/env.sh"   # ROS + this workspace + the venv (pymavlink for the decode tests)

fail=0
check() {  # check <description> <expected count> <actual count>
  if [ "$2" = "$3" ]; then echo "PASS  $1 ($3)"; else echo "FAIL  $1: expected $2, got $3"; fail=1; fi
}

# Exact names this workspace builds. Extra drone_* names found elsewhere on the path
# (another sourced workspace, an apt package) are reported as WARN, not counted.
PKGS="drone_bringup drone_fcu drone_interfaces drone_llm drone_mission drone_perception drone_safety"
EXES="drone_fcu:fcu_command_node drone_fcu:fcu_state_node drone_llm:intent_bridge drone_mission:mission_manager drone_perception:target_tracker drone_perception:yolo_detector drone_safety:safety_supervisor"
expect_set() {  # expect_set <description> <expected names> <found names>
  local missing="" extra="" n=0
  for x in $2; do if echo "$3" | grep -qx "$x"; then n=$((n+1)); else missing="$missing $x"; fi; done
  for x in $3; do echo " $2 " | grep -q " $x " || extra="$extra $x"; done
  if [ -z "$missing" ]; then echo "PASS  $1 ($n)"; else echo "FAIL  $1 missing:$missing"; fail=1; fi
  if [ -n "$extra" ]; then echo "WARN  $1 from outside this workspace:$extra"; fi
}

echo "== 2. packages"
expect_set "drone packages" "$PKGS" "$(ros2 pkg list | grep '^drone_')"
for p in $(ros2 pkg list | grep '^drone_'); do
  echo " $PKGS " | grep -q " $p " || echo "      $p is at $(ros2 pkg prefix "$p")"
done

echo "== 3. interfaces"
check "drone_interfaces types" 9 "$(ros2 interface list | grep -c 'drone_interfaces/')"
ros2 interface show drone_interfaces/action/FlightOperation > /dev/null && echo "PASS  interface show FlightOperation"
python3 -c "from drone_interfaces.action import FlightOperation, SearchAndTrack" && echo "PASS  python import"

echo "== 4. executables"
expect_set "drone executables" "$EXES" "$(ros2 pkg executables | grep '^drone_' | tr ' ' ':')"

echo "== 5. python deps"
if python3 -c "import pymavlink" 2>/dev/null; then echo "PASS  pymavlink importable"
else echo "FAIL  pymavlink not importable: run tools/setup_venv.sh"; fail=1; fi

echo "== 6. tests"
colcon test --event-handlers console_direct- > /dev/null
colcon test-result --all | tail -1
colcon test-result > /dev/null || fail=1
skipped=$(colcon test-result --all | tail -1 | sed -n 's/.* \([0-9]*\) skipped.*/\1/p')
if [ "${skipped:-0}" != "0" ]; then echo "FAIL  $skipped tests skipped"; fail=1; fi

if [ $fail = 0 ]; then echo "WORKSPACE CHECK: ALL PASSED"; else echo "WORKSPACE CHECK: FAILURES ABOVE"; exit 1; fi
