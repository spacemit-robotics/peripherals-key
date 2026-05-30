#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/peripherals/key/${SROBOTIS_TEST_NAME:-key-api-contract}}"
log_dir="$artifact_dir/logs"
build_dir="$artifact_dir/build"
mode="${1:-all}"
case "$mode" in
    all|functional|error-paths) ;;
    *) echo "usage: $0 [all|functional|error-paths]" >&2; exit 2 ;;
esac

log_file="$log_dir/key_api_${mode//-/_}.log"
cc="${CC:-cc}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] cc=$cc"
    echo "[info] mode=$mode"

    "$cc" -D_DEFAULT_SOURCE -std=c99 -Wall -Wextra -Werror \
        -I"$module_root/tests/fakes" \
        -I"$module_root/include" \
        "$module_root/src/key.c" \
        "$module_root/tests/fakes/fake_gpiod.c" \
        "$module_root/tests/test_key_api_contract.c" \
        -pthread \
        -o "$build_dir/test_key_api_contract"

    "$build_dir/test_key_api_contract" "$mode"
} | tee "$log_file"

case "$mode" in
    all) grep -q "key api contract test PASSED" "$log_file" ;;
    functional) grep -q "key api functional test PASSED" "$log_file" ;;
    error-paths) grep -q "key api error paths test PASSED" "$log_file" ;;
esac
