cmake_minimum_required(VERSION 2.8)
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(pwm_python)

    find_package(PythonInterp RQUIRED)
    find_package(PythonLibs RQUIRED)
    include(BuildBoost)

    include_directories(${Boost_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIR})

    # Build and link the pylib module
    add_library(pylibpwm SHARED pylibpwm.cpp)
    target_link_libraries(pylibpwm ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
    add_dependencies(mylib Boost)

    # Tweaks the name of the library to match what Python expects
    set_target_properties(pylibpwm PROPERTIES SUFFIX .so)
    set_target_properties(pylibpwm PROPERTIES PREFIX "")

