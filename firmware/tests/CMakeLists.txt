cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_COMPILER clang++)
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

# Instrument the production source for libFuzzer coverage as well as ASan/UBSan.
# Both libraries compile the SAME source file, with different instrumentation.
add_library(adc_processing_fuzz STATIC ../Core/Src/pdu.cpp)
target_include_directories(adc_processing_fuzz PUBLIC
  mocks
  ../Core/Inc/
  ../Components/PUTM_EV_CAN_LIBRARY/database/generated/
)
target_link_libraries(adc_processing_fuzz PUBLIC
  BTS72220
  GTest::gmock
)
target_compile_options(adc_processing_fuzz PRIVATE
    -O1 -g -fno-omit-frame-pointer
    -fsanitize=fuzzer-no-link,address,undefined,float-cast-overflow
    -fno-sanitize-recover=undefined,float-cast-overflow
)

add_executable(fuzz_adc fuzz/adc/fuzz_adc.cpp)
target_link_libraries(fuzz_adc PRIVATE adc_processing_fuzz)
target_compile_options(fuzz_adc PRIVATE
    -O1 -g -fno-omit-frame-pointer
    -fsanitize=fuzzer-no-link,address,undefined,float-cast-overflow
    -fno-sanitize-recover=undefined,float-cast-overflow
)
target_link_options(fuzz_adc PRIVATE
    -fsanitize=fuzzer,address,undefined,float-cast-overflow
)