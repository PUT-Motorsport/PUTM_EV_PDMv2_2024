cmake_minimum_required(VERSION 3.22)
project(pdu_tests)

add_subdirectory(PUTM_EV_TESTING_LIBRARY)

putm_test_setup()

# pdu.cpp is compiled with mocks/ in include path, so it uses mock Led and
# Spi_daisy_chain instead of Core/Inc/wrappers
putm_add_unit_test(test_pdu
  SOURCES
    unit/test_bts72220.cpp
    unit/test_pdu.cpp
    ../Core/Src/pdu.cpp
    ../Components/BTS72220/BTS72220.cpp
  INCLUDES 
    mocks 
    helpers 
    ../Core/Inc/ 
    ../Components/BTS72220/include
    ../Components/PUTM_EV_CAN_LIBRARY/database/generated
)

# --- Fuzz tests (clang only, skipped with other compilers) ---
# Same production source as the unit tests, instrumented for fuzzing
putm_add_fuzz_library(bts72220_fuzz
  SOURCES 
    ../Components/BTS72220/BTS72220.cpp
  INCLUDES 
    ../Components/BTS72220/include
)

# fuzz/<name>/fuzz_<name>.cpp + fuzz/<name>/corpus
putm_add_fuzzer(bts_channel LIBS bts72220_fuzz)
