#!/usr/bin/env bash
# write_git_hash.sh - write source/git_hash_pkg.vhd with the short hash of
# HEAD (0 before the first commit), every build script runs this first
set -euo pipefail
cd "$(dirname "$0")"
hash=$(git rev-parse --short=8 HEAD 2>/dev/null || echo 00000000)
cat > source/git_hash_pkg.vhd <<VHDL
library ieee;
    use ieee.std_logic_1164.all;

package git_hash_pkg is

    constant git_hash : std_logic_vector(31 downto 0) := x"$hash";
end package;
VHDL
echo "git hash $hash"
