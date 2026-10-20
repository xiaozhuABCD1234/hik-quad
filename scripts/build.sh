#!/usr/bin/env bash
# Configure, build and test the project with the release preset.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure
