#!/usr/bin/env bash
# Builds and runs the host-side engine tests (needs only g++ >= 9).
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -pthread engine_test.cpp -o engine_test
./engine_test
