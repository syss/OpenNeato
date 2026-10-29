#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
flags=(-std=gnu++17 -Wall -Wextra -Werror -g -Ifirmware/src)
if [[ ${SANITIZE:-0} == 1 ]]; then
    flags+=(-fsanitize=address,undefined -fno-omit-frame-pointer)
fi
"${CXX:-g++}" "${flags[@]}" tests/navigation_math_test.cpp firmware/src/navigation_math.cpp -o "$build_dir/math"
"$build_dir/math"
host=(-Itests/host -include tests/host/navigation_fakes.h -DCHIP_MODEL=\"HOST\" -DCONFIG_IDF_TARGET_ESP32C3=1 -DENABLE_LOGGING=0 -Wno-unused-parameter)
"${CXX:-g++}" "${flags[@]}" "${host[@]}" tests/manual_navigation_test.cpp firmware/src/manual_clean_manager.cpp -o "$build_dir/manual"
"$build_dir/manual"
"${CXX:-g++}" "${flags[@]}" "${host[@]}" -DOPENNEATO_EXPERIMENTAL_NAVIGATION=1 tests/navigation_manager_test.cpp firmware/src/navigation_manager.cpp firmware/src/navigation_math.cpp firmware/src/manual_clean_manager.cpp -o "$build_dir/manager"
"$build_dir/manager"
"${CXX:-g++}" "${flags[@]}" "${host[@]}" tests/navigation_disabled_test.cpp firmware/src/navigation_manager.cpp firmware/src/navigation_math.cpp firmware/src/manual_clean_manager.cpp -o "$build_dir/disabled"
"$build_dir/disabled"
"${CXX:-g++}" "${flags[@]}" -Itests/host -DCHIP_MODEL=\"HOST\" -DCONFIG_IDF_TARGET_ESP32C3=1 -DENABLE_LOGGING=0 -Wno-unused-parameter -Wno-unused-variable tests/neato_navigation_test.cpp firmware/src/neato_commands.cpp firmware/src/neato_serial.cpp firmware/src/json_fields.cpp -o "$build_dir/serial"
"$build_dir/serial"
