#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  build.sh - Efinity build for the Efinix Trion T120F324 development board
#
#    ./build.sh            synthesis, place and route, bitstream
#    ./build.sh program    load outflow/uart_test.bit over JTAG
#    ./build.sh peri       regenerate uart_test.peri.xml
#  Set EFINITY_HOME if Efinity is not in ~/efinity/2026.1, TRION_SERIAL if
#  the board's FT2232H serial is not FT56NF97.
# ---------------------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"

EFINITY_HOME="${EFINITY_HOME:-$HOME/efinity/2026.1}"
TRION_SERIAL="${TRION_SERIAL:-FT56NF97}"
# setup.sh is not written for set -euo pipefail (it dies of SIGPIPE)
set +euo pipefail
source "$EFINITY_HOME/bin/setup.sh" > /dev/null
set -euo pipefail

stage="${1:-build}"
case "$stage" in
    build)
        ../write_git_hash.sh
        exec efx_run --prj uart_test.xml
        ;;
    program)
        project=uart_test
        # efx_run's program flow takes the first FTDI device it finds, which
        # with several boards attached can be another board : program the
        # Trion's FT2232H by its serial number, JTAG is on channel B
        url="ftdi://0x0403:0x6010:$TRION_SERIAL/2"
        if ! (cd "$EFINITY_HOME/pgm/bin" && python3 -m efx_pgm.ftdi_program -l 2>/dev/null) | grep -q "$TRION_SERIAL/2"; then
            echo "Trion board (FT2232H $TRION_SERIAL) is not attached, usbipd attach --wsl --busid <b-p>" >&2
            exit 1
        fi
        bit="$PWD/outflow/$project.bit"
        cd "$EFINITY_HOME/pgm/bin"
        exec python3 -m efx_pgm.ftdi_program "$bit" -m jtag -u "$url" -b "Generic Board Profile Using FT2232H"
        ;;
    peri)
        exec python3 make_peri.py
        ;;
    *)
        echo "Usage: $0 [build|program|peri]" >&2
        exit 1
        ;;
esac
