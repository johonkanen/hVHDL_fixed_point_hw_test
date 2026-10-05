#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  build.sh - Vivado build of the uart test for the Alchitry Au+
#
#    ./build.sh            build bitstream into output/
#    ./build.sh program    load output/alchitry_au_top.bit over JTAG
#
#  Set VIVADO_BIN if vivado is not in ~/Xilinx/2024.2 or on PATH.
# ---------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

if [[ -z "${VIVADO_BIN:-}" ]]; then
    if [[ -x "$HOME/Xilinx/2024.2/Vivado/2024.2/bin/vivado" ]]; then
        VIVADO_BIN="$HOME/Xilinx/2024.2/Vivado/2024.2/bin/vivado"
    else
        VIVADO_BIN=vivado
    fi
fi

STAGE="${1:-build}"

mkdir -p output

case "$STAGE" in
    build)
        ../write_git_hash.sh
        cd output
        exec "$VIVADO_BIN" -mode batch -nojournal -log build.log -source ../build.tcl
        ;;
    program)
        cd output
        exec "$VIVADO_BIN" -mode batch -nojournal -log program.log -source ../program.tcl
        ;;
    *)
        echo "Usage: $0 [build|program]" >&2
        exit 1
        ;;
esac
