#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  build.sh - Efinity build of the uart test for the Ti60F225 EVM
#
#    ./build.sh            synthesis, place and route, bitstream -> outflow/
#    ./build.sh program    load outflow/uart_test.bit over JTAG
#    ./build.sh peri       regenerate uart_test.peri.xml with make_peri.py
#
#  Set EFINITY_HOME if Efinity is not in ~/efinity/2026.1.
#  The EVM's FT4232H has to be attached to wsl (usbipd attach) to program.
# ---------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

EFINITY_HOME="${EFINITY_HOME:-$HOME/efinity/2026.1}"
# setup.sh is not written for set -euo pipefail (it dies of SIGPIPE)
set +euo pipefail
source "$EFINITY_HOME/bin/setup.sh" > /dev/null
set -euo pipefail

case "${1:-build}" in
    build)
        ../write_git_hash.sh
        exec efx_run --prj uart_test.xml
        ;;
    program)
        exec efx_run uart_test.xml --flow program --pgm_opts mode=jtag
        ;;
    peri)
        exec python3 make_peri.py
        ;;
    *)
        echo "Usage: $0 [build|program|peri]" >&2
        exit 1
        ;;
esac
