#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/peripherals/key/${SROBOTIS_TEST_NAME:-key-gpio-hardware-smoke}}"
log_dir="$artifact_dir/logs"
build_dir="$artifact_dir/build"
log_file="$log_dir/key_gpio_hardware_smoke.log"

timeout_s="${KEY_HW_SMOKE_TIMEOUT_S:-30}"
expect_pattern="${KEY_HW_EXPECT_PATTERN:-物理按下|单击事件|双击事件|长按事件}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] timeout_s=$timeout_s"
    echo "[info] expect_pattern=$expect_pattern"

    cmake -S "$module_root" -B "$build_dir"
    cmake --build "$build_dir" --target test_key -j"$(nproc)"
    echo "[info] waiting for a key event"
    set +e
    LD_LIBRARY_PATH="$build_dir:${LD_LIBRARY_PATH:-}" \
        stdbuf -oL -eL "$build_dir/test_key" &
    test_pid=$!
    seen=0
    for _ in $(seq 1 "$timeout_s"); do
        if grep -Eq "$expect_pattern" "$log_file"; then
            seen=1
            break
        fi
        if ! kill -0 "$test_pid" 2>/dev/null; then
            break
        fi
        sleep 1
    done
    if kill -0 "$test_pid" 2>/dev/null; then
        kill "$test_pid" 2>/dev/null
        wait "$test_pid" 2>/dev/null
    else
        wait "$test_pid" 2>/dev/null
    fi
    set -e
    if [ "$seen" -ne 1 ]; then
        echo "[error] no key event matching $expect_pattern observed within ${timeout_s}s"
        exit 1
    fi
} 2>&1 | tee "$log_file"

grep -Eq "$expect_pattern" "$log_file"
