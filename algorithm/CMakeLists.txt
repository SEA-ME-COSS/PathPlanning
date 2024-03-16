cmake_minimum_required(VERSION 3.5)

project(path_planning VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 20)

# Find Python
find_package(PythonLibs REQUIRED)

include_directories(
    ${PYTHON_INCLUDE_DIRS}
    planner
    utils
    visual
    .)

add_executable(${PROJECT_NAME} 
    main.cpp)

target_link_libraries(${PROJECT_NAME}
    ${PYTHON_LIBRARIES})
