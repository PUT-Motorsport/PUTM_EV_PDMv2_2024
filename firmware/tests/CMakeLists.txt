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