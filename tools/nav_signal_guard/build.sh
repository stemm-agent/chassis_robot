#!/bin/sh
# 重新编译信号屏蔽库并安装到系统路径
set -e
gcc -shared -fPIC -O2 -o /usr/local/lib/nav_signal_guard/nav_signal_guard.so \
    "$(dirname "$0")/nav_signal_guard.c"
echo "installed: /usr/local/lib/nav_signal_guard/nav_signal_guard.so"
