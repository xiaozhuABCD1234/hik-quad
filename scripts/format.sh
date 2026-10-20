#!/usr/bin/env bash
# Format all C++ sources.  Use --check in CI to fail on drift.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
mapfile -t files < <(find include src apps tests -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.hpp.in' \))
if [ "${1:-}" = "--check" ]; then
  clang-format --dry-run --Werror "${files[@]}"
else
  clang-format -i "${files[@]}"
fi
