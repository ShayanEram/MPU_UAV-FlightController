cmake_minimum_required(VERSION 3.10)
project(UAV)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Source files
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/Src/*.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILES}
    Inc/HardwareAbstractions.hpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/Inc/Utils
)

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