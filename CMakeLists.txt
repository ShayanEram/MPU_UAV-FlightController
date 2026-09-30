cmake_minimum_required(VERSION 3.20)
project(UAV_FlightController LANGUAGES CXX)

set(PROJECT_NAME UAV_FlightController)

include(CTest)
include(${CMAKE_BINARY_DIR}/generators/conan_toolchain.cmake)

# Global find_package calls
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Source files
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/Src/*.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/Inc/Utils
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Boost::boost
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
    spdlog::spdlog_header_only
)

target_compile_definitions(${PROJECT_NAME} PUBLIC BOOST_ASIO_NO_DEPRECATED)

# Unit tests ##########################################
option(BUILD_TESTS "Enable building unit tests" OFF)

if(BUILD_TESTS)
    # Add GoogleTest
    add_subdirectory(${CMAKE_SOURCE_DIR}/../googletest googletest-build)

    file(GLOB_RECURSE TEST_SOURCES
        ${CMAKE_SOURCE_DIR}/Tests/*.cpp
    )

    add_executable(${PROJECT_NAME}_tests ${TEST_SOURCES})

    target_include_directories(${PROJECT_NAME}_tests PUBLIC
        ${CMAKE_SOURCE_DIR}/Inc
        ${CMAKE_SOURCE_DIR}/Inc/Utils
    )

    target_link_libraries(${PROJECT_NAME}_tests
        gtest
        gtest_main
        pthread
    )

    enable_testing()
    add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)

endif()