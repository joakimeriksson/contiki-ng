#!/bin/bash
# Runs a 17-tun-rpl-br test under Cooja on this Mac. Run as:  sudo ./run-07.sh [test-basename]
# (Cooja spawns the host script without a tty, so a cached sudo password does not reach it.)
[ "$(id -u)" -eq 0 ] || { echo "run with: sudo $0"; exit 1; }
export HOME=/Users/joakimeriksson
export PATH=/opt/homebrew/opt/openjdk/bin:/opt/homebrew/opt/coreutils/libexec/gnubin:/opt/homebrew/bin:/opt/homebrew/sbin:$PATH
cd "$(dirname "$0")/tests/17-tun-rpl-br" || exit 1
T=${1:-07-native-border-router-cooja}
if [ "$T" = all ]; then
  # Whole directory, in CI order, via the stock test Makefile.
  make summary 2>&1 | tee ../../all.run.log | grep -E 'Loading .*csc|TEST OK|FAILED|ping6 statistics|packets received|Summary|All tests OK|Failures|TEST FAIL'
  exit 0
fi
rm -f $T.testlog summary COOJA.testlog
../../tools/cooja/gradlew --no-watch-fs -p ../../tools/cooja run --args="--no-gui --contiki=$(realpath ../..) --logdir=$(pwd) --random-seed=1 $(realpath $T.csc)" 2>&1 | tee $T.run.log | grep -v '^ccache\|^gcc\|^mkdir\|^> make\|^  CC\|^rm '
echo "=== exit: ${PIPESTATUS[0]}; logs: tests/17-tun-rpl-br/COOJA.testlog, $T.run.log"
