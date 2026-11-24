# PDM Unit Tests

## Framework
Tests are built using [GoogleTest](https://github.com/google/googletest) (documentation: [GoogleTest User's Guide](https://google.github.io/googletest/)). The library is fetched automatically via CMake `FetchContent` during the initial configuration step.

Shared test infrastructure (CMake functions, run scripts, fuzz helpers) lives in the
`PUTM_EV_TESTING_LIBRARY` git submodule. Initialize it after cloning:
```bash
git submodule update --init firmware/tests/PUTM_EV_TESTING_LIBRARY
```
See `PUTM_EV_TESTING_LIBRARY/README.md` for the available CMake functions and helpers.

## Structure
```
firmware/tests/
  CMakeLists.txt          # test targets (putm_add_unit_test, putm_add_fuzzer, ...)
  unit/                   # GoogleTest unit tests (test_*.cpp)
  mocks/                  # gmock replacements of hardware wrappers (Core/Inc/wrappers)
  helpers/                # shared test helpers, e.g. GoogleTest printers (bts_printers.hpp)
  fuzz/<name>/            # fuzz_<name>.cpp harness + corpus/ seed inputs
  PUTM_EV_TESTING_LIBRARY/   # shared test infrastructure (git submodule)
```
New unit test file: add it to `unit/` and to `SOURCES` of `putm_add_unit_test()` in
`CMakeLists.txt`. Headers in `mocks/` and `helpers/` are on the include path.

## Prerequisites
- CMake (>= 3.22)
- C++20 compatible compiler (e.g., GCC, Clang)

## Running Tests

Run the helper script from any directory:
```bash
./firmware/tests/PUTM_EV_TESTING_LIBRARY/scripts/build_and_run_tests.sh
```

Additional `ctest` arguments can be passed directly to the script (e.g., filtering tests by name):
```bash
./firmware/tests/PUTM_EV_TESTING_LIBRARY/scripts/build_and_run_tests.sh -R TurnOff
```

Manual build and execution from the `firmware/tests` directory:
```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Fuzz Tests

Fuzz targets use [libFuzzer](https://llvm.org/docs/LibFuzzer.html) with ASan/UBSan and `implicit-conversion` sanitizers. They are opt-in (`-DPUTM_BUILD_FUZZERS=ON`, set by the fuzz script) and require **clang++**, so a normal build and `build_and_run_tests.sh` never compile them and a broken harness cannot block unit tests.

| Target | Code under test | Input |
|---|---|---|
| `fuzz_bts_channel` | `BTS::Channel` overcurrent state machine | threshold + sequence of operations (see header comment in `fuzz/bts_channel/fuzz_bts_channel.cpp`) |

The harness checks state machine invariants after every operation (e.g. `PERM_LOCK` is permanent, `ERR` is always resolved by `handle_overcurrent()`, `turn_on()` never unlocks a locked channel).

Adding a new fuzzer: create `fuzz/<name>/fuzz_<name>.cpp` (+ optional `fuzz/<name>/corpus/`)
and register it in `CMakeLists.txt` with `putm_add_fuzzer(<name> LIBS <fuzz library>)`.
Production code under test is compiled with fuzz instrumentation by
`putm_add_fuzz_library()`. Harnesses can use `putm_test::ByteReader` and
`PUTM_FUZZ_CHECK` from `<putm_test/fuzz_helpers.hpp>`.

Run all fuzzers with the helper script (builds only fuzz targets with clang into `fuzz/build`, fuzzes each target for 60 s, exits with 1 and prints reproduce commands on crash):
```bash
./firmware/tests/PUTM_EV_TESTING_LIBRARY/scripts/build_and_run_fuzz.sh                    # all targets, 60 s each
./firmware/tests/PUTM_EV_TESTING_LIBRARY/scripts/build_and_run_fuzz.sh bts_channel 300    # one target, 5 min
./firmware/tests/PUTM_EV_TESTING_LIBRARY/scripts/build_and_run_fuzz.sh all 60 -- -max_len=256  # extra libFuzzer args
```
The working corpus in `fuzz/build/corpus/` grows between runs; crashes are saved in `fuzz/build/artifacts/`.

Manual build with clang (seed corpus is replayed by `ctest` as `fuzz_bts_channel_corpus`):
```bash
CC=clang CXX=clang++ cmake -S . -B build-clang -DPUTM_BUILD_FUZZERS=ON
cmake --build build-clang --parallel                         # unit tests + fuzzers
cmake --build build-clang --parallel --target putm_fuzzers   # fuzzers only
ctest --test-dir build-clang --output-on-failure
```

Run the fuzzer (work on a copy so the seed corpus stays small and readable):
```bash
mkdir -p /tmp/fuzz_corpus && cp fuzz/bts_channel/corpus/* /tmp/fuzz_corpus/
./build-clang/fuzz_bts_channel -max_total_time=60 -artifact_prefix=/tmp/ /tmp/fuzz_corpus
```

Reproduce a crash:
```bash
./build-clang/fuzz_bts_channel /tmp/crash-<hash>
```
