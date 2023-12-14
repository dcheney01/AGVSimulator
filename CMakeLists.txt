cmake_minimum_required(VERSION 3.5)

project(AGVSimulator VERSION 0.1 LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Gui Widgets)
find_package(GTest REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# AutonomyLibrary =======================================================================================================
add_library(AutonomyLibrary SHARED)

target_sources(AutonomyLibrary
    PRIVATE
    robot.cpp
    astar.cpp
    environment.cpp
    node.cpp
    path_follower.cpp
    math_utils.cpp
    obstacle_utils.cpp

    PUBLIC
        FILE_SET HEADERS
            BASE_DIRS ${PROJECT_SOURCE_DIR}
            FILES robot.hpp environment.hpp astar.hpp node.hpp path_follower.hpp math_utils.hpp obstacle_utils.hpp
)

target_include_directories(AutonomyLibrary
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)


# UnitTests ===========================================================================================================

add_executable(UnitTests
    robot_unittests.cpp
    environment_unittests.cpp
    astar_unittests.cpp
    node_unittests.cpp
    math_utils_unittests.cpp
    path_follower_unittests.cpp
    obstacles_unittests.cpp
)

target_include_directories(UnitTests PRIVATE "${GTEST_INCLUDE_DIRS}")

target_link_libraries(UnitTests
    PRIVATE
        ${GTEST_LIBRARIES}
        ${GTEST_MAIN_LIBRARIES}
        AutonomyLibrary
    )




# SimulatorInterface ===================================================================================================
add_executable(SimulatorInterface

)

target_sources(SimulatorInterface
    PRIVATE
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        drawing_utils.hpp
        drawing_utils.cpp
        gui_utils.h gui_utils.cpp
)

target_compile_features(SimulatorInterface PUBLIC cxx_std_17)
target_include_directories(SimulatorInterface PRIVATE)

target_link_libraries(SimulatorInterface
    PRIVATE
        Qt6::Widgets
        Qt6::Gui
        AutonomyLibrary
)


