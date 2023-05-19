cmake_minimum_required(VERSION 3.0.0)
project(kinematics_simulator VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    )

add_executable(kinematics_simulator main.cpp)

target_link_libraries(
    kinematics_simulator 
    ${OpenCV_LIBS}
    ${SDL2_LIBRARIES}
    ${Boost_LIBRARIES}
)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
