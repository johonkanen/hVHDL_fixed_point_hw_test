#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  test_all.sh - load each board's bitstream over JTAG and run test_uart.py
#
#    ./test_all.sh                 all four boards: au axc3000 ti60evm trion
#    ./test_all.sh ti60evm trion   only these
#
#  Builds nothing, run <board>/build.sh first. A board that fails to program
#  is not tested, the other boards still run. The output of every step goes
#  to test_logs/<board>_program.log and test_logs/<board>_test.log.
#  Exits 1 when any board failed.
# ---------------------------------------------------------------------------
set -uo pipefail
cd "$(dirname "$0")"

# test_uart.py board name -> board folder
declare -A FOLDER=([au]=alchitry [axc3000]=axc3000 [ti60evm]=ti60evm [trion]=trion)

boards=("$@")
[[ ${#boards[@]} -eq 0 ]] && boards=(au axc3000 ti60evm trion)
for board in "${boards[@]}"; do
    [[ -n "${FOLDER[$board]:-}" ]] || { echo "unknown board '$board', choose from: ${!FOLDER[*]}" >&2; exit 1; }
done

mkdir -p test_logs
declare -A RESULT
failed=0

for board in "${boards[@]}"; do
    program_log="test_logs/${board}_program.log"
    test_log="test_logs/${board}_test.log"
    echo "======== $board"

    if ! "${FOLDER[$board]}/build.sh" program > "$program_log" 2>&1; then
        RESULT[$board]="PROGRAM FAILED  ($program_log)"
        failed=1
        tail -5 "$program_log"
        continue
    fi
    echo "programmed"

    if python3 test_uart.py --board "$board" > "$test_log" 2>&1; then
        RESULT[$board]="passed, $(grep -c '^  PASS' "$test_log") checks"
    else
        RESULT[$board]="TEST FAILED  ($test_log)"
        failed=1
    fi
    grep -E '^  FAIL|Traceback|Error' "$test_log"
    tail -1 "$test_log"
done

echo
echo "======== summary"
for board in "${boards[@]}"; do
    printf '  %-8s %s\n' "$board" "${RESULT[$board]}"
done
exit $failed
