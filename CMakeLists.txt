# ----------------------------------------------------------------------------
# @file    CMakeLists.txt
# @brief   builds the HISPEC tracking camera daemon and its Python module
#
# This project owns the instrument; camerad comes in as a dependency. Clone and
# build, nothing else to set up:
#
#   cmake -S . -B build && cmake --build build
#
# Add -DBUILD_PYTHON_MODULE=ON for the Python module.
# ----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.24)

project(hispec_tracking_camera_camerad CXX)

# Declared before the fetch: it is what makes the core's static libraries
# position-independent and creates the camerad::python target
option(BUILD_PYTHON_MODULE "Build the Python module" OFF)

set(CAMERAD_PACKAGE_NAME "camera_interface_tracking" CACHE STRING
    "Python package the module installs into")

include(FetchContent)

FetchContent_Declare(camera_interface
  GIT_REPOSITORY https://github.com/CaltechOpticalObservatories/camera-interface
  GIT_TAG        main
)

set(CONTROLLER archon CACHE STRING "Controller the tracking camera uses")

FetchContent_MakeAvailable(camera_interface)

set(TRACKING_SOURCES
  hispec_tracking_camera_instrument.cpp
  hispec_tracking_camera_interface_factory.cpp
  hispec_tracking_camera_exposure_modes.cpp
  fits_header_dictionary.cpp
)

add_executable(camerad ${TRACKING_SOURCES})

# The daemon library carries no controller, and CONTROLLER_ARCHON rides on the
# controller target's interface, so both are needed
target_link_libraries(camerad PRIVATE camerad::daemon camerad::archon)

install(TARGETS camerad RUNTIME DESTINATION bin COMPONENT daemon)
install(DIRECTORY config/ DESTINATION share/hispec_tracking_camera COMPONENT daemon)

# ----------------------------------------------------------------------------
# Python module, built from the same instrument sources so that it drives the
# camera in-process. It needs no daemon, hence no camerad::daemon here
# ----------------------------------------------------------------------------
if (BUILD_PYTHON_MODULE)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

  # pybind11 ships its CMake config inside the wheel, so ask the interpreter
  # where it is rather than making every caller pass -Dpybind11_DIR
  if (NOT pybind11_DIR)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
      OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE PYBIND11_PROBE_RESULT
      ERROR_QUIET
    )
    if (NOT PYBIND11_PROBE_RESULT EQUAL 0)
      message(FATAL_ERROR
              "BUILD_PYTHON_MODULE=ON needs pybind11 in ${Python3_EXECUTABLE}. "
              "Install it with: ${Python3_EXECUTABLE} -m pip install pybind11")
    endif()
    list(APPEND CMAKE_PREFIX_PATH ${PYBIND11_CMAKE_DIR})
  endif()

  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(camera_interface ${TRACKING_SOURCES})
  target_compile_definitions(camera_interface PRIVATE
                             CAMERAD_INSTRUMENT_NAME="hispec_tracking_camera")
  target_link_libraries(camera_interface PRIVATE camerad::python camerad::archon)

  # Installing into a package of its own, rather than at the wheel root, lets
  # this build share an environment with other instruments
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/package/__init__.py [[
from . import camera_interface as _module
from .camera_interface import *  # noqa: F401,F403

__all__ = [name for name in dir(_module) if not name.startswith("_")]
]])
  install(TARGETS camera_interface
          LIBRARY DESTINATION ${CAMERAD_PACKAGE_NAME} COMPONENT python)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package/__init__.py
          DESTINATION ${CAMERAD_PACKAGE_NAME} COMPONENT python)

  # A shared library may leave symbols undefined, so a module missing its
  # interface factory links and fails only when imported
  enable_testing()
  add_test(NAME python_import
    COMMAND ${Python3_EXECUTABLE} -c
      "import camera_interface\nassert camera_interface.instrument_name() == 'hispec_tracking_camera', camera_interface.instrument_name()\nassert camera_interface.controller_name() == 'archon', camera_interface.controller_name()"
  )
  set_tests_properties(python_import PROPERTIES
    ENVIRONMENT PYTHONPATH=$<TARGET_FILE_DIR:camera_interface>
  )
endif()
