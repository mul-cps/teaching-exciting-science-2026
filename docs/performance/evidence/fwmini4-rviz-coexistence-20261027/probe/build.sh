#!/bin/sh
set -eu
cd "$(dirname "$0")"
gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared swap_probe.c -o libswap_probe.so -ldl -pthread
gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared fake_gl.c -o libfake_gl.so
gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared fake_gl.c -o libfake_minimal.so -Wl,--version-script=hide_optional.map
gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -shared audit_heap.c -o libaudit_heap.so
gcc -std=c11 -O2 -Wall -Wextra -Werror fake_host.c -o fake_host -L. -Wl,-rpath,'$ORIGIN' -lfake_gl -laudit_heap
gcc -std=c11 -O2 -Wall -Wextra -Werror missing_host.c -o missing_host -ldl
gcc -std=c11 -O2 -Wall -Wextra -Werror fake_host.c -o minimal_host -L. -Wl,-rpath,'$ORIGIN' -lfake_minimal -laudit_heap
