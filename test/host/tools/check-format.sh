#!/usr/bin/env bash
# Verify every tracked C/C++ source is clang-format LLVM-style clean.
# Exit non-zero (failing the CI test) if any file would be reformatted.
#
# Usage:
#   check-format.sh          # check only; lists offending files
#   check-format.sh --fix    # reformat offending files in place
set -euo pipefail

fix=0
if [ "${1:-}" = "--fix" ]; then
  fix=1
fi

# Enumerate sources. Prefer git (only tracked files, never vendored deps); fall
# back to find (excluding the CMake build tree) when git is unavailable.
if git rev-parse --show-toplevel >/dev/null 2>&1; then
  cd "$(git rev-parse --show-toplevel)"
  mapfile -t files < <(git ls-files '*.c' '*.cpp' '*.h' '*.hpp' '*.ino')
else
  cd "$(dirname "$0")/../../.."
  mapfile -t files < <(find . -path ./build -prune -o \
    \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \
    -o -name '*.ino' \) -print)
fi

if [ "${#files[@]}" -eq 0 ]; then
  echo "check-format: no C/C++ files found"
  exit 0
fi

clang-format --version

fail=0
for f in "${files[@]}"; do
  if [ "$fix" -eq 1 ]; then
    clang-format --style=LLVM -i "$f"
    continue
  fi
  if ! clang-format --style=LLVM --dry-run --Werror "$f" >/dev/null 2>&1; then
    echo "NOT clang-format clean: $f"
    fail=1
  fi
done

if [ "$fix" -eq 1 ]; then
  echo "check-format: reformatted ${#files[@]} file(s)"
  exit 0
fi

if [ "$fail" -ne 0 ]; then
  echo
  echo "Run: test/host/tools/check-format.sh --fix"
  exit 1
fi

echo "check-format: all ${#files[@]} file(s) clean"
