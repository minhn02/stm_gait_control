cmake_minimum_required(VERSION 3.5)
project(stm_control)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include /usr/local/include)

find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Bezier REQUIRED)

add_library(nlopt SHARED IMPORTED)
set_target_properties(nlopt PROPERTIES IMPORTED_LOCATION /usr/local/lib/libnlopt.so)

include_directories(${EIGEN3_INCLUDE_DIRS})

set(FILES
    include/stm_control/control/transition_generator.hpp
    include/stm_control/control/wheel_controller.hpp
    include/stm_control/gaits/gait.hpp
    include/stm_control/gaits/idle_gait.hpp
    include/stm_control/gaits/squirming_gait.hpp
    include/stm_control/gaits/transition_gait.hpp
    include/stm_control/gaits/wheelwalking_gait.hpp
    include/stm_control/joints.h
    src/control/transition_generator.cpp
    src/control/wheel_controller.cpp
    src/gaits/idle_gait.cpp
    src/gaits/squirming_gait.cpp
    src/gaits/transition_gait.cpp
    src/gaits/wheelwalking_gait.cpp)

add_library(stm_state_machine MODULE include/stm_control/control/state_machine.hpp src/control/state_machine.cpp ${FILES})
target_link_libraries(stm_state_machine PRIVATE pybind11::module pybind11::lto pybind11::windows_extras PUBLIC Bezier nlopt Eigen3::Eigen)

pybind11_extension(stm_state_machine)
if(NOT MSVC AND NOT ${CMAKE_BUILD_TYPE} MATCHES Debug|RelWithDebInfo)
    # Strip unnecessary sections of the binary on Linux/macOS
    pybind11_strip(stm_state_machine)
endif()

set_target_properties(stm_state_machine PROPERTIES CXX_VISIBILITY_PRESET "hidden"
                                         CUDA_VISIBILITY_PRESET "hidden")