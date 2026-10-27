#!/bin/sh
set -eu
cd "$(dirname "$0")"
./build.sh
python3 test_probe.py > green.log 2>&1
gcc -std=c11 -O1 -g -Wall -Wextra -Werror -fPIC -shared \
  -fsanitize=undefined -fsanitize-undefined-trap-on-error \
  swap_probe.c -o libswap_probe_ubsan.so -ldl -pthread
LD_PRELOAD="$PWD/libswap_probe_ubsan.so" ./fake_host bounds > sanitizer.log 2>&1
python3 test_mutants.py > mutant-run.log 2>&1
