#!/bin/sh
# Offline MAB smoke test: compiles the real kernel MAB with -DSIM.
# Replaces the Makefile. Usage:
#   ./build.sh          build the sim binary
#   ./build.sh clean    remove the sim binary
#
# Honors CC and CFLAGS from the environment.

set -eu

CC=${CC:-gcc}
CFLAGS=${CFLAGS:-"-O2 -g -Wall"}

cd "$(dirname "$0")"

case "${1:-sim}" in
sim|all)
	echo "$CC $CFLAGS -DSIM -I. -I.. -o sim sim.c ../kernel_mab.c"
	# shellcheck disable=SC2086
	$CC $CFLAGS -DSIM -I. -I.. -o sim sim.c ../kernel_mab.c
	;;
clean)
	rm -f sim
	;;
*)
	echo "usage: $0 [sim|clean]" >&2
	exit 1
	;;
esac
