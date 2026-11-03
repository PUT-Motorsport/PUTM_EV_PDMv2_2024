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
