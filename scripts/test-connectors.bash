#!/usr/bin/env bash

# test script for anchor connectors (mock, serial, CAN)

set -o pipefail

repo_root="$(git rev-parse --show-toplevel)"

if [[ -z $repo_root ]]; then
  echo "script must be run from within the rover-ros2 repo" >&2
  exit 1
fi

cd "$repo_root"

# colors
BOLD='\033[1m'
RED='\033[1;31m'
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

TESTS_PASSED=0
TESTS_FAILED=0

log() {
  echo -e "${BOLD}${YELLOW}info:${NC} ${1}"
}

pass() {
  echo -e "${BOLD}${GREEN}pass:${NC} ${1}"
  TESTS_PASSED=$((TESTS_PASSED + 1))
}

fail() {
  echo -e "${BOLD}${RED}fail:${NC} ${1}"
  TESTS_FAILED=$((TESTS_FAILED + 1))
}

cleanup() {
  log "cleaning up"
  if [[ -n $ANCHOR_PID ]]; then
    kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
    wait "$ANCHOR_PID" 2>/dev/null || true
  fi
  if [[ -n $SOCAT_PID ]]; then
    kill -INT "$SOCAT_PID" 2>/dev/null || true
    wait "$SOCAT_PID" 2>/dev/null || true
  fi
  rm -f /tmp/ttyACM9 /tmp/ttyOUT 2>/dev/null || true
}

trap cleanup EXIT

source_ros2() {
  source install/setup.bash
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-5}"
  local count=0
  while ! ros2 topic list 2>/dev/null | grep -q "^${topic}$"; do
    sleep 0.5
    count=$((count + 1))
    if [[ $count -ge $((timeout * 2)) ]]; then
      return 1
    fi
  done
  return 0
}

# run a ROS pub/echo test
# usage: ros_pubsub_test <echo_topic> <pub_topic> <msg_type> <msg_data>
# returns the echo output via stdout
ros_pubsub_test() {
  local echo_topic="$1"
  local pub_topic="$2"
  local msg_type="$3"
  local msg_data="$4"

  timeout 5 bash -c "
    ros2 topic echo --once $echo_topic &
    ECHO_PID=\$!
    sleep 0.5
    ros2 topic pub --once $pub_topic $msg_type \"$msg_data\" >/dev/null 2>&1
    wait \$ECHO_PID
  " 2>/dev/null || true
}

test_mock_connector() {
  log "testing mock connector"

  log "starting anchor with mock connector"
  setsid ros2 run anchor_pkg anchor --ros-args -p connector:=mock &
  ANCHOR_PID=$!
  sleep 2

  if ! kill -0 "$ANCHOR_PID" 2>/dev/null; then
    fail "mock connector: anchor failed to start"
    return 1
  fi

  if ! wait_for_topic "/anchor/to_vic/relay" 10; then
    fail "mock connector: topics not available"
    kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
    return 1
  fi

  log "anchor started successfully"

  # test: relay -> debug
  log "testing relay -> debug"
  local output
  output=$(ros_pubsub_test "/anchor/to_vic/debug" "/anchor/to_vic/relay" \
    "astra_msgs/msg/VicCAN" '{mcu_name: \"core\", command_id: 50, data: [1.0, 2.0, 3.0, 4.0]}')

  if [[ -n $output ]] && echo "$output" | grep -q "can_relay_tovic,core,50"; then
    pass "mock connector: relay -> debug"
  else
    fail "mock connector: relay -> debug"
  fi

  # test: mock_mcu -> from_vic/core
  log "testing mock_mcu (core) -> from_vic/core"
  output=$(ros_pubsub_test "/anchor/from_vic/core" "/anchor/from_vic/mock_mcu" \
    "astra_msgs/msg/VicCAN" '{mcu_name: \"core\", command_id: 10, data: [100.0, 200.0]}')

  if [[ -n $output ]] && echo "$output" | grep -q "mcu_name: core" && echo "$output" | grep -q "command_id: 10"; then
    pass "mock connector: mock_mcu -> from_vic/core"
  else
    fail "mock connector: mock_mcu -> from_vic/core"
  fi

  # test: mock_mcu -> from_vic/arm
  log "testing mock_mcu (arm) -> from_vic/arm"
  output=$(ros_pubsub_test "/anchor/from_vic/arm" "/anchor/from_vic/mock_mcu" \
    "astra_msgs/msg/VicCAN" '{mcu_name: \"arm\", command_id: 55, data: [0.0, 450.0, 900.0, 0.0]}')

  if [[ -n $output ]] && echo "$output" | grep -q "mcu_name: arm" && echo "$output" | grep -q "command_id: 55"; then
    pass "mock connector: mock_mcu -> from_vic/arm"
  else
    fail "mock connector: mock_mcu -> from_vic/arm"
  fi

  # test: mock_mcu -> from_vic/bio
  log "testing mock_mcu (citadel) -> from_vic/bio"
  output=$(ros_pubsub_test "/anchor/from_vic/bio" "/anchor/from_vic/mock_mcu" \
    "astra_msgs/msg/VicCAN" '{mcu_name: \"citadel\", command_id: 20, data: [5.0]}')

  if echo "$output" | grep -q "mcu_name: citadel" && echo "$output" | grep -q "command_id: 20"; then
    pass "mock connector: mock_mcu -> from_vic/bio"
  else
    fail "mock connector: mock_mcu -> from_vic/bio"
  fi

  # test: relay_string -> debug
  log "testing relay_string -> debug"
  output=$(ros_pubsub_test "/anchor/to_vic/debug" "/anchor/to_vic/relay_string" \
    "std_msgs/msg/String" '{data: \"test_raw_string_data\"}')

  if [[ -n $output ]] && echo "$output" | grep -q "test_raw_string_data"; then
    pass "mock connector: relay_string -> debug"
  else
    fail "mock connector: relay_string -> debug"
  fi

  kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
  wait "$ANCHOR_PID" 2>/dev/null || true
  ANCHOR_PID=""
}

test_serial_connector() {
  log "testing serial connector"

  log "creating virtual serial ports with socat"
  socat pty,raw,echo=0,link=/tmp/ttyACM9 pty,raw,echo=0,link=/tmp/ttyOUT 2>/dev/null &
  SOCAT_PID=$!
  sleep 2

  if ! kill -0 "$SOCAT_PID" 2>/dev/null; then
    fail "serial connector: failed to create virtual serial ports"
    return 1
  fi

  log "starting anchor with serial connector (override: /tmp/ttyACM9)"
  setsid ros2 run anchor_pkg anchor --ros-args -p connector:=serial -p serial_override:=/tmp/ttyACM9 &
  ANCHOR_PID=$!
  sleep 2

  if ! kill -0 "$ANCHOR_PID" 2>/dev/null; then
    fail "serial connector: anchor failed to start"
    kill -INT "$SOCAT_PID" 2>/dev/null || true
    return 1
  fi

  if ! wait_for_topic "/anchor/to_vic/relay" 10; then
    fail "serial connector: topics not available"
    kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
    kill -INT "$SOCAT_PID" 2>/dev/null || true
    return 1
  fi

  pass "serial connector: anchor starts with virtual serial"

  # test: relay -> serial output (VicCAN encoding)
  log "testing relay -> serial output"

  local serial_out_file
  serial_out_file=$(mktemp)

  # Start head first (blocks waiting for input), then publish
  timeout 5 head -n1 /tmp/ttyOUT >"$serial_out_file" &
  local head_pid=$!
  sleep 0.3
  ros2 topic pub --once /anchor/to_vic/relay astra_msgs/msg/VicCAN \
    '{mcu_name: "core", command_id: 30, data: [1.0, 2.0, 3.0, 4.0]}' >/dev/null 2>&1
  wait $head_pid 2>/dev/null || true

  local serial_out
  serial_out=$(cat "$serial_out_file")
  rm -f "$serial_out_file"

  if [[ -n $serial_out ]] && echo "$serial_out" | grep -q "can_relay_tovic,core,30"; then
    pass "serial connector: relay -> serial output"
  else
    fail "serial connector: relay -> serial output (got: $serial_out)"
  fi

  # test: serial input -> from_vic/core
  log "testing serial input -> from_vic/core"

  local output
  output=$(timeout 5 bash -c '
    ros2 topic echo --once /anchor/from_vic/core &
    ECHO_PID=$!
    sleep 0.5
    echo "can_relay_fromvic,core,15,10.0,20.0,30.0,40.0" > /tmp/ttyOUT
    sleep 0.5
    echo "can_relay_fromvic,core,15,10.0,20.0,30.0,40.0" > /tmp/ttyOUT
    wait $ECHO_PID
  ' 2>/dev/null) || true

  if echo "$output" | grep -q "mcu_name: core" && echo "$output" | grep -q "command_id: 15"; then
    pass "serial connector: serial input -> from_vic/core"
  else
    fail "serial connector: serial input -> from_vic/core (got: $output)"
  fi

  # test: relay_string -> debug
  log "testing relay_string -> debug"
  output=$(ros_pubsub_test "/anchor/to_vic/debug" "/anchor/to_vic/relay_string" \
    "std_msgs/msg/String" '{data: \"serial_test_string\"}')

  if [[ -n $output ]] && echo "$output" | grep -q "serial_test_string"; then
    pass "serial connector: relay_string -> debug"
  else
    fail "serial connector: relay_string -> debug"
  fi

  kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
  wait "$ANCHOR_PID" 2>/dev/null || true
  ANCHOR_PID=""
  kill -INT "$SOCAT_PID" 2>/dev/null || true
  wait "$SOCAT_PID" 2>/dev/null || true
  SOCAT_PID=""
}

test_can_connector() {
  log "testing CAN connector"

  log "starting anchor with CAN connector (override: vcan0)"
  setsid ros2 run anchor_pkg anchor --ros-args -p connector:=can -p can_override:=vcan0 &
  ANCHOR_PID=$!
  sleep 2

  if ! kill -0 "$ANCHOR_PID" 2>/dev/null; then
    fail "CAN connector: anchor failed to start"
    return 1
  fi

  if ! wait_for_topic "/anchor/to_vic/relay" 10; then
    fail "CAN connector: topics not available"
    kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
    return 1
  fi

  log "anchor started successfully"
  sleep 1

  # test: relay -> CAN bus
  # core=1, int16x4=2, cmd=30 -> id = (1<<8)|(2<<6)|30 = 0x19E
  log "testing relay -> CAN bus"

  local output
  output=$(timeout 8 bash -c '
    candump -n 1 vcan0 &
    DUMP_PID=$!
    sleep 1
    ros2 topic pub --once /anchor/to_vic/relay astra_msgs/msg/VicCAN "{mcu_name: \"core\", command_id: 30, data: [1, 2, 3, 4]}" >/dev/null 2>&1
    sleep 0.5
    ros2 topic pub --once /anchor/to_vic/relay astra_msgs/msg/VicCAN "{mcu_name: \"core\", command_id: 30, data: [1, 2, 3, 4]}" >/dev/null 2>&1
    wait $DUMP_PID
  ' 2>/dev/null) || true

  if echo "$output" | grep -qi "19E"; then
    pass "CAN connector: relay -> CAN bus"
  else
    fail "CAN connector: relay -> CAN bus (got: $output)"
  fi

  # test: CAN -> from_vic/core
  log "testing CAN bus -> from_vic/core"

  output=$(timeout 5 bash -c '
    ros2 topic echo --once /anchor/from_vic/core &
    ECHO_PID=$!
    sleep 1
    cansend vcan0 18F#000A0014001E0028
    sleep 0.5
    cansend vcan0 18F#000A0014001E0028
    wait $ECHO_PID
  ' 2>/dev/null) || true

  if echo "$output" | grep -q "mcu_name: core" && echo "$output" | grep -q "command_id: 15"; then
    pass "CAN connector: CAN -> from_vic/core"
  else
    fail "CAN connector: CAN -> from_vic/core"
  fi

  # test: CAN -> from_vic/arm
  log "testing CAN bus -> from_vic/arm"

  output=$(timeout 5 bash -c '
    ros2 topic echo --once /anchor/from_vic/arm &
    ECHO_PID=$!
    sleep 1
    cansend vcan0 294#00640096012C01F4
    sleep 0.5
    cansend vcan0 294#00640096012C01F4
    wait $ECHO_PID
  ' 2>/dev/null) || true

  if echo "$output" | grep -q "mcu_name: arm" && echo "$output" | grep -q "command_id: 20"; then
    pass "CAN connector: CAN -> from_vic/arm"
  else
    fail "CAN connector: CAN -> from_vic/arm"
  fi

  # test: CAN double data type (data_type_key=0)
  log "testing CAN double data type"

  output=$(timeout 8 bash -c '
    ros2 topic echo --once /anchor/from_vic/core &
    ECHO_PID=$!
    sleep 1
    cansend vcan0 105#3FF0000000000000
    sleep 0.5
    cansend vcan0 105#3FF0000000000000
    sleep 0.5
    cansend vcan0 105#3FF0000000000000
    wait $ECHO_PID
  ' 2>/dev/null) || true

  if echo "$output" | grep -q "mcu_name: core" && echo "$output" | grep -q "command_id: 5"; then
    pass "CAN connector: double data type"
  else
    fail "CAN connector: double data type"
  fi

  # test: CAN float32x2 data type (data_type_key=1)
  log "testing CAN float32x2 data type"

  output=$(timeout 8 bash -c '
    ros2 topic echo --once /anchor/from_vic/core &
    ECHO_PID=$!
    sleep 1
    cansend vcan0 14A#3F80000040000000
    sleep 0.5
    cansend vcan0 14A#3F80000040000000
    sleep 0.5
    cansend vcan0 14A#3F80000040000000
    wait $ECHO_PID
  ' 2>/dev/null) || true

  if echo "$output" | grep -q "mcu_name: core" && echo "$output" | grep -q "command_id: 10"; then
    pass "CAN connector: float32x2 data type"
  else
    fail "CAN connector: float32x2 data type"
  fi

  kill -INT -- -"$ANCHOR_PID" 2>/dev/null || true
  wait "$ANCHOR_PID" 2>/dev/null || true
  ANCHOR_PID=""
}

check_prerequisites() {
  log "checking prerequisites"
  local missing=0

  if [[ ! -f install/setup.bash ]]; then
    fail "install/setup.bash not found; run 'colcon build --symlink-install' first"
    missing=1
  fi

  if ! command -v socat &>/dev/null; then
    fail "socat not found; install it or use 'nix develop'"
    missing=1
  fi

  if ! command -v cansend &>/dev/null || ! command -v candump &>/dev/null; then
    fail "can-utils (cansend/candump) not found; install it or use 'nix develop'"
    missing=1
  fi

  if ! ip link show vcan0 &>/dev/null; then
    fail "vcan0 interface not found"
    log "  create it with:"
    log "    sudo ip link add dev vcan0 type vcan"
    log "    sudo ip link set vcan0 up"
    missing=1
  elif ! ip link show vcan0 | grep -q ",UP"; then
    fail "vcan0 exists but is not UP"
    log "  enable it with: sudo ip link set vcan0 up"
    missing=1
  fi

  if [[ $missing -eq 1 ]]; then
    echo ""
    log "prerequisites not met"
    exit 1
  fi

  log "all prerequisites met"
}

main() {
  echo ""
  log "anchor connector test suite"
  echo ""

  check_prerequisites

  log "sourcing ROS2 workspace"
  source_ros2

  test_mock_connector
  test_serial_connector
  test_can_connector

  echo ""
  log "test summary"
  echo -e "${BOLD}${GREEN}passed:${NC} $TESTS_PASSED"
  echo -e "${BOLD}${RED}failed:${NC} $TESTS_FAILED"
  echo ""

  if [[ $TESTS_FAILED -gt 0 ]]; then
    exit 1
  fi
  exit 0
}

main "$@"
