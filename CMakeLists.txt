cmake_minimum_required(VERSION 4.0)
project(assignmentNarducci)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
include(FetchContent)
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 3.0.2
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(SFML)

if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xpreprocessor -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lomp")
    # Get Homebrew home directory
    execute_process(
            COMMAND brew --prefix
            OUTPUT_VARIABLE HOMEBREW_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    # The following 2 lines are needed on macOS.
    # Check the position of headers in libraries (e.g. Macports vs. Homebrew)
    include_directories(${HOMEBREW_PREFIX}/opt/libomp/include)
    link_directories(${HOMEBREW_PREFIX}/opt/libomp/lib)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

add_executable(assignmentNarducci main.cpp)
target_link_libraries(assignmentNarducci PRIVATE SFML::Graphics)
