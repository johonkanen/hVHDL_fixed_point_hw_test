#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  build_all.sh - check the VHDL with Quartus, then build every board
#
#    ./build_all.sh                 all four boards : alchitry axc3000 ti60evm trion
#    ./build_all.sh ti60evm trion   only these
#
#  First Quartus' analysis and elaboration (axc3000/build.sh elaborate, about
#  a minute) : Quartus and Efinity reject VHDL that nvc and Vivado accept,
#  e.g. a descending slice of an unconstrained parameter, and a full build
#  takes ten. If it fails nothing is built. Then the builds run in parallel,
#  each logged to build_logs/<board>.log, and the summary gives each
#  build's result, whether it wrote a new bitstream and its worst setup
#  slack. Exits 1 when the check or any build failed.
#  Then ./test_all.sh programs and tests the boards.
# ---------------------------------------------------------------------------
set -uo pipefail
cd "$(dirname "$0")"

declare -A BITSTREAM=([alchitry]=alchitry/output/alchitry_au_top.bit
                      [axc3000]=axc3000/output_files/uart_test.sof
                      [ti60evm]=ti60evm/outflow/uart_test.bit
                      [trion]=trion/outflow/uart_test.bit)

boards=("$@")
[[ ${#boards[@]} -eq 0 ]] && boards=(alchitry axc3000 ti60evm trion)
for board in "${boards[@]}"; do
    [[ -n "${BITSTREAM[$board]:-}" ]] || { echo "unknown board '$board', choose from: ${!BITSTREAM[*]}" >&2; exit 1; }
done

mkdir -p build_logs

echo "======== quartus analysis and elaboration"
if ! axc3000/build.sh elaborate > build_logs/elaborate.log 2>&1; then
    grep -E "^Error|Error \(" build_logs/elaborate.log | head -10
    echo "elaboration failed, see build_logs/elaborate.log ; nothing built" >&2
    exit 1
fi
echo "passed"

./write_git_hash.sh > /dev/null
started=$(date +%s)
echo "======== building ${boards[*]}"
for board in "${boards[@]}"; do
    ( "$board/build.sh" > "build_logs/$board.log" 2>&1; echo "exit $?" >> "build_logs/$board.log" ) &
done
wait

slack() {
    case $1 in
        alchitry) grep -m1 -oE "Slack \((MET|VIOLATED)\) : +-?[0-9.]+ns" alchitry/output/timing_summary.rpt | grep -oE -- "-?[0-9.]+ns" ;;
        axc3000)  sed -n '/^Type  : Setup/{n;p;q}' axc3000/output_files/uart_test.sta.summary | grep -oE -- "-?[0-9.]+" | sed 's/$/ns/' ;;
        ti60evm|trion) grep -m1 "Slack  " "$1/outflow/uart_test.timing.rpt" | grep -oE -- "-?[0-9.]+" | head -1 | sed 's/$/ns/' ;;
    esac
}

failed=0
echo
echo "======== summary"
for board in "${boards[@]}"; do
    result=$(grep '^exit' "build_logs/$board.log" | tail -1)
    fresh="old bitstream"
    if [[ -f "${BITSTREAM[$board]}" ]] && (( $(stat -c %Y "${BITSTREAM[$board]}") >= started )); then
        fresh="new bitstream"
    fi
    if [[ "$result" == "exit 0" && "$fresh" == "new bitstream" ]]; then
        printf '  %-8s built, %s, setup slack %s\n' "$board" "$fresh" "$(slack "$board")"
    else
        printf '  %-8s FAILED (%s, %s), see build_logs/%s.log\n' "$board" "$result" "$fresh" "$board"
        failed=1
    fi
done
exit $failed
