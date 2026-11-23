cmake_minimum_required(VERSION 3.22)
project(pdu_tests)

# Shared test infrastructure (GoogleTest, libFuzzer), git submodule
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/PUTM_EV_TEST_LIBRARY/CMakeLists.txt)
  message(FATAL_ERROR
    "PUTM_EV_TEST_LIBRARY submodule is missing, run:\n"
    "  git submodule update --init firmware/tests/PUTM_EV_TEST_LIBRARY")
endif()
add_subdirectory(PUTM_EV_TEST_LIBRARY)
putm_test_setup()

# --- Unit tests (GoogleTest) ---
add_subdirectory(../Components/BTS72220 bts72220_build)

# pdu.cpp is compiled with mocks/ in include path, so it uses mock Led and
# Spi_daisy_chain instead of Core/Inc/wrappers
putm_add_unit_test(test_pdu
  SOURCES
    unit/test_bts72220.cpp
    unit/test_pdu.cpp
    ../Core/Src/pdu.cpp
  LIBS BTS72220
  INCLUDES mocks helpers ../Core/Inc/
)
putm_test_link_can(test_pdu)

# --- Fuzz tests (clang only, skipped with other compilers) ---
# Same production source as the unit tests, instrumented for fuzzing
putm_add_fuzz_library(bts72220_fuzz
  SOURCES ../Components/BTS72220/BTS72220.cpp
  INCLUDES ../Components/BTS72220/include
)

# fuzz/<name>/fuzz_<name>.cpp + fuzz/<name>/corpus
putm_add_fuzzer(bts_channel LIBS bts72220_fuzz)
