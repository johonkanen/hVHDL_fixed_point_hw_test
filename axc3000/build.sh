#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  build.sh - Quartus Pro build of the uart test for the Arrow AXC3000
#
#    ./build.sh            full compile -> output_files/uart_test.sof
#    ./build.sh program    load the .sof over JTAG (see program.sh)
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
        exec "$QUARTUS_BIN/quartus_sh" -t build.tcl compile
        ;;
    program)
        exec ./program.sh output_files/uart_test.sof
        ;;
    *)
        echo "Usage: $0 [build|program]" >&2
        exit 1
        ;;
esac
