#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  build.sh - Quartus Pro build of the uart test for the Arrow AXC3000
#
#    ./build.sh            full compile -> output_files/uart_test.sof
#    ./build.sh program    load the .sof over JTAG (see program.sh)
#    ./build.sh elaborate  analysis and elaboration only, a minute's check
#                          of the VHDL ; not while a build runs, they share
#                          the project
#
#  Set QUARTUS_BIN if quartus is not in ~/altera_pro/26.1.1/quartus/bin.
# ---------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

QUARTUS_BIN="${QUARTUS_BIN:-$HOME/altera_pro/26.1.1/quartus/bin}"

case "${1:-build}" in
    build)
        ../write_git_hash.sh
        # build.tcl only adds assignments to an existing project, start from
        # a fresh one so nothing stale from an earlier run carries over
        rm -f uart_test.qsf uart_test.qpf
        "$QUARTUS_BIN/quartus_sh" -t build.tcl compile
        # quartus writes the .sof whatever the timing, like the alchitry
        # build fail on a negative slack and leave no .sof to program
        if grep -qE '^Slack : -' output_files/uart_test.sta.summary; then
            grep -B1 -A2 -E '^Slack : -' output_files/uart_test.sta.summary >&2
            rm -f output_files/uart_test.sof
            echo "timing not met, see output_files/uart_test.sta.rpt" >&2
            exit 1
        fi
        ;;
    program)
        exec ./program.sh output_files/uart_test.sof
        ;;
    elaborate)
        ../write_git_hash.sh
        rm -f uart_test.qsf uart_test.qpf
        exec "$QUARTUS_BIN/quartus_sh" -t build.tcl elaborate
        ;;
    *)
        echo "Usage: $0 [build|program|elaborate]" >&2
        exit 1
        ;;
esac
