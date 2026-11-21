# PDM Unit Tests

## Framework
Tests are built using [GoogleTest](https://github.com/google/googletest) (documentation: [GoogleTest User's Guide](https://google.github.io/googletest/)). The library is fetched automatically via CMake `FetchContent` during the initial configuration step.

## Prerequisites
- CMake (>= 3.14)
- C++20 compatible compiler (e.g., GCC, Clang)

## Running Tests

Run the helper script from any directory:
```bash
./firmware/tests/build_and_run_tests.sh
```

Additional `ctest` arguments can be passed directly to the script (e.g., filtering tests by name):
```bash
./firmware/tests/build_and_run_tests.sh -R TurnOff
```

Manual build and execution from the `firmware/tests` directory:
```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Fuzz Tests

Fuzz targets use [libFuzzer](https://llvm.org/docs/LibFuzzer.html) with ASan/UBSan and `implicit-conversion` sanitizers. They are built only with **clang++**; with GCC they are skipped and unit tests build as before.

| Target | Code under test | Input |
|---|---|---|
| `fuzz_bts_channel` | `BTS::Channel` overcurrent state machine | threshold + sequence of operations (see header comment in `fuzz/bts_channel/fuzz_bts_channel.cpp`) |

The harness checks state machine invariants after every operation (e.g. `PERM_LOCK` is permanent, `ERR` is always resolved by `handle_overcurrent()`, `turn_on()` never unlocks a locked channel).

Run all fuzzers with the helper script (builds with clang into `fuzz/build`, fuzzes each target for 60 s, exits with 1 and prints reproduce commands on crash):
```bash
./firmware/tests/build_and_run_fuzz.sh                    # all targets, 60 s each
./firmware/tests/build_and_run_fuzz.sh bts_channel 300    # one target, 5 min
./firmware/tests/build_and_run_fuzz.sh all 60 -- -max_len=256  # extra libFuzzer args
```
The working corpus in `fuzz/build/corpus/` grows between runs; crashes are saved in `fuzz/build/artifacts/`.

Manual build with clang (seed corpus is replayed by `ctest` as `fuzz_bts_channel_corpus`):
```bash
CC=clang CXX=clang++ cmake -S . -B build-clang
cmake --build build-clang --parallel
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
