#!/usr/bin/env bash
set -euo pipefail

# Usage: ./build_and_run_fuzz.sh [target] [seconds] [-- extra libFuzzer args]
#   target   fuzzer directory name in fuzz/ (e.g. bts_channel), "all" by default
#   seconds  fuzzing time per target, 60 by default

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

BUILD_DIR="fuzz/build"

TARGET="all"
SECONDS_PER_TARGET=60
if [[ $# -gt 0 && "$1" != "--" ]]; then TARGET="$1"; shift; fi
if [[ $# -gt 0 && "$1" != "--" ]]; then SECONDS_PER_TARGET="$1"; shift; fi
if [[ $# -gt 0 && "$1" == "--" ]]; then shift; fi
EXTRA_ARGS=("$@")

CC="${CC:-clang}"
CXX="${CXX:-clang++}"
if ! command -v "${CXX}" >/dev/null; then
  echo "error: ${CXX} not found, fuzzing requires clang++ (set CXX to override)" >&2
  exit 1
fi

CC="${CC}" CXX="${CXX}" cmake -S . -B "${BUILD_DIR}" -DPDM_BUILD_FUZZERS=ON
cmake --build "${BUILD_DIR}" --parallel

if [[ "${TARGET}" == "all" ]]; then
  TARGETS=()
  for dir in fuzz/*/; do
    name="$(basename "${dir}")"
    [[ -f "fuzz/${name}/fuzz_${name}.cpp" ]] && TARGETS+=("${name}")
  done
else
  TARGETS=("${TARGET}")
fi

FAILED=()
for name in "${TARGETS[@]}"; do
  binary="${BUILD_DIR}/fuzz_${name}"
  if [[ ! -x "${binary}" ]]; then
    echo "error: fuzzer ${binary} not built" >&2
    exit 1
  fi

  # Work on a copy, so the committed seed corpus stays small and readable
  corpus="${BUILD_DIR}/corpus/${name}"
  artifacts="${BUILD_DIR}/artifacts/${name}"
  mkdir -p "${corpus}" "${artifacts}"
  cp -n "fuzz/${name}/corpus/"* "${corpus}/" 2>/dev/null || true

  echo "=== Fuzzing ${name} for ${SECONDS_PER_TARGET}s ==="
  if ! "${binary}" -max_total_time="${SECONDS_PER_TARGET}" \
      -artifact_prefix="${artifacts}/" "${EXTRA_ARGS[@]}" "${corpus}"; then
    FAILED+=("${name}")
  fi
done

if [[ ${#FAILED[@]} -gt 0 ]]; then
  echo
  echo "Crashes found in: ${FAILED[*]}"
  for name in "${FAILED[@]}"; do
    for crash in "${BUILD_DIR}/artifacts/${name}"/*; do
      [[ -f "${crash}" ]] && echo "  reproduce: ${BUILD_DIR}/fuzz_${name} ${crash}"
    done
  done
  exit 1
fi

echo "No crashes found."
