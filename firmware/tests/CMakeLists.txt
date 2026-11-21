cmake_minimum_required(VERSION 3.14)
project(pdu_tests)

# GoogleTest requires at least C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_subdirectory(../Components/BTS72220 bts72220_build)

add_executable(test_pdu
  test_bts72220.cpp
  test_pdu.cpp

  ../Core/Src/pdu.cpp
)
target_link_libraries(test_pdu
  GTest::gtest_main
  GTest::gmock
  BTS72220
)

target_include_directories(test_pdu PRIVATE
  mocks
  ../Core/Inc/
  ../Components/PUTM_EV_CAN_LIBRARY/database/generated/
)

include(GoogleTest)
gtest_discover_tests(test_pdu)

# --- Fuzz tests (clang only: libFuzzer + sanitizers) ---
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  option(PDM_BUILD_FUZZERS "Build libFuzzer targets" ON)
else()
  set(PDM_BUILD_FUZZERS OFF)
  message(STATUS "Fuzz targets skipped: they require clang++")
endif()

if(PDM_BUILD_FUZZERS)
  set(FUZZ_COMPILE_FLAGS
    -O1 -g -fno-omit-frame-pointer
    -fsanitize=fuzzer-no-link,address,undefined,implicit-conversion
    -fno-sanitize-recover=undefined,implicit-conversion
  )
  set(FUZZ_LINK_FLAGS
    -fsanitize=fuzzer,address,undefined,implicit-conversion
  )

  # Same production source as the unit tests, instrumented for fuzzing
  add_library(bts72220_fuzz STATIC ../Components/BTS72220/BTS72220.cpp)
  target_include_directories(bts72220_fuzz PUBLIC
    ../Components/BTS72220/include
  )
  target_compile_options(bts72220_fuzz PRIVATE ${FUZZ_COMPILE_FLAGS})

  add_executable(fuzz_bts_channel fuzz/bts_channel/fuzz_bts_channel.cpp)
  target_link_libraries(fuzz_bts_channel PRIVATE bts72220_fuzz)
  target_compile_options(fuzz_bts_channel PRIVATE ${FUZZ_COMPILE_FLAGS})
  target_link_options(fuzz_bts_channel PRIVATE ${FUZZ_LINK_FLAGS})

  # Replay the seed corpus as a regular test (no mutation)
  add_test(NAME fuzz_bts_channel_corpus
    COMMAND fuzz_bts_channel -runs=0
            ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/bts_channel/corpus
  )
endif()
