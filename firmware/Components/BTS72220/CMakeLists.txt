cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_library(BTS72220)

target_sources(BTS72220
    PUBLIC
        BTS72220.cpp
)

target_include_directories(BTS72220
    PUBLIC
        include

)

