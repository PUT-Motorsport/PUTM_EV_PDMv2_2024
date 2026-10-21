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

add_executable(test_bts72220
  test_bts72220.cpp
)
target_link_libraries(test_bts72220
  GTest::gtest_main
  BTS72220
)

include(GoogleTest)
gtest_discover_tests(test_bts72220)