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
