cmake_minimum_required(VERSION 2.6)
project(scanMatching)

include_directories(include)
file(GLOB SOURCES "src/*.cpp")

add_executable(matcher ${SOURCES})

