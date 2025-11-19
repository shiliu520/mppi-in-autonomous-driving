cmake_minimum_required(VERSION 3.20)
project(ContingencyPlanning LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(ENABLE_WARNINGS "Enable strict compiler warnings" OFF)

# Dependencies
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/environment-model)
set(MPPI_BUILD_TESTS OFF CACHE BOOL "Disable MPPI tests/internal libs" FORCE)
set(MPPI_BUILD_EXAMPLES OFF CACHE BOOL "Disable MPPI examples" FORCE)
set(MPPI_USE_CUDA_BARRIERS OFF CACHE BOOL "Disable CUDA barriers for older GPUs" FORCE)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/MPPI-Generic)

# Foxglove SDK (headers + prebuilt static lib)
set(FOXGLOVE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/foxglove")
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${FOXGLOVE_SOURCE_DIR}/src/*.cpp"
    "${FOXGLOVE_SOURCE_DIR}/src/server/*.cpp"
)

add_library(foxglove_sdk STATIC IMPORTED)
set_target_properties(foxglove_sdk PROPERTIES
    IMPORTED_LOCATION "${FOXGLOVE_SOURCE_DIR}/lib/libfoxglove.so"
    INTERFACE_INCLUDE_DIRECTORIES "${FOXGLOVE_SOURCE_DIR}/include"
)

# Sources
file(GLOB SIMULATOR_SOURCES CONFIGURE_DEPENDS "simulator/*.cpp")
file(GLOB PLANNING_SOURCES CONFIGURE_DEPENDS
    "planning/*.cpp"
    "planning/*.cu"
)

include_directories(${FOXGLOVE_SOURCE_DIR}/include)

# Simulator library
add_library(simulator SHARED ${SIMULATOR_SOURCES})
target_compile_definitions(simulator PUBLIC PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_include_directories(simulator
    PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${FOXGLOVE_SOURCE_DIR}/include
)

add_library(planning SHARED ${PLANNING_SOURCES})
set_target_properties(planning PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)
target_include_directories(planning
    PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${PROJECT_SOURCE_DIR}/3rdparty/MPPI-Generic/include
        ${PROJECT_SOURCE_DIR}
)
target_link_libraries(planning
    PUBLIC
        Eigen3::Eigen
        MPPI::MPPI
)

# Executable
add_executable(planning_node planning_node.cpp)
target_sources(planning_node PRIVATE ${FOXGLOVE_SOURCES})
target_link_libraries(planning_node
    PRIVATE
        simulator
        planning
        spdlog::spdlog
        yaml-cpp
        foxglove_sdk
        Threads::Threads
        dl
)

if(ENABLE_WARNINGS)
    foreach(tgt simulator planning_node)
        target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()
