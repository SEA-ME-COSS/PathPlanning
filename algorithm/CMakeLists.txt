cmake_minimum_required(VERSION 3.5)
project(pathplanning VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BASIC_PLANNER "Build with the basic planner" OFF)

# Find required packages
find_package(PythonLibs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)

if(BASIC_PLANNER)
    message(STATUS "Building with Basic Planner")

    include_directories(
        ${PYTHON_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/lib
        ${CMAKE_SOURCE_DIR}/lib/basic_pathplanning
    )

    add_executable(${PROJECT_NAME} "src/basic_planner/main.cpp")

    target_link_libraries(${PROJECT_NAME}
        ${PYTHON_LIBRARIES}
    )  
else()
    message(STATUS "Building with Hybrid A* Planner")

    include_directories(
        ${PYTHON_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        lib
        lib/hybrid_a_star
        lib/hybrid_a_star/hybrid-a-star
    )

    FILE(GLOB NavObjects lib/hybrid_a_star/hybrid-a-star/*.cpp)
    FILE(GLOB HybridAStar src/hybrid_a_star/*/*.cpp)

    add_executable(${PROJECT_NAME} ${NavObjects} ${HybridAStar} "src/hybrid_a_star/main.cpp")

    target_link_libraries(${PROJECT_NAME}
        ${CERES_LIBRARIES}
        ompl
        Eigen3::Eigen
        ${PYTHON_LIBRARIES}
    )
endif()
