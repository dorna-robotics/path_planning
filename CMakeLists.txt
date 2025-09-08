cmake_minimum_required(VERSION 3.21) # TARGET_RUNTIME_DLLS needs >= 3.21
project(path_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Policies
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# Auto-use vcpkg toolchain if available
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE FILEPATH "")
endif()

# ---------- Python selection (manual) ----------
# Expect: -D Python3_EXECUTABLE="C:/Path/To/python.exe"
if(NOT DEFINED Python3_EXECUTABLE)
  message(FATAL_ERROR "Pass your interpreter with -D Python3_EXECUTABLE=... (python.exe)")
endif()

# Pull major+minor (e.g., 39), sys.exec_prefix, and site-packages (platlib)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import sys,sysconfig; mm=f'{sys.version_info[0]}{sys.version_info[1]}'; print(mm); print(sys.exec_prefix); print(sysconfig.get_path('platlib'))"
  OUTPUT_VARIABLE _PY_INFO
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REPLACE "\r\n" "\n" _PY_INFO "${_PY_INFO}")
string(REPLACE "\r"   "\n" _PY_INFO "${_PY_INFO}")
string(REPLACE "\n" ";" _PY_INFO "${_PY_INFO}")
list(GET _PY_INFO 0 PY_MM)
list(GET _PY_INFO 1 PY_ROOT)
list(GET _PY_INFO 2 PY_SITEARCH)

# If user supplied overrides, use them; else derive from Python3_EXECUTABLE
if(DEFINED Python3_INCLUDE_DIR)
  set(PY_INCLUDE "${Python3_INCLUDE_DIR}")
else()
  set(PY_INCLUDE "${PY_ROOT}/include") # Windows python.org layout
endif()

if(DEFINED Python3_LIBRARY)
  set(PY_LIB "${Python3_LIBRARY}")
else()
  set(PY_LIB "${PY_ROOT}/libs/python${PY_MM}.lib") # Windows import lib
endif()

# Sanity checks
if(NOT EXISTS "${PY_INCLUDE}/Python.h")
  message(FATAL_ERROR "Python headers not found at: ${PY_INCLUDE}/Python.h")
endif()
if(NOT EXISTS "${PY_LIB}")
  message(FATAL_ERROR "Python library not found: ${PY_LIB}")
endif()

# Normalize slashes so install() paths are clean on Windows
if(COMMAND cmake_path)
  cmake_path(CONVERT "${PY_SITEARCH}" TO_CMAKE_PATH_LIST PY_SITEARCH_CMAKE)
else()
  file(TO_CMAKE_PATH "${PY_SITEARCH}" PY_SITEARCH_CMAKE)
endif()

# ---------- pybind11 (don’t let it search Python; we provide it) ----------
set(PYBIND11_FINDPYTHON OFF)
set(PYTHON_EXECUTABLE  "${Python3_EXECUTABLE}")
set(PYTHON_INCLUDE_DIR "${PY_INCLUDE}")
set(PYTHON_LIBRARY     "${PY_LIB}")
find_package(pybind11 CONFIG REQUIRED)

# ---------- Dependencies (via vcpkg or your toolchain) ----------
find_package(ompl CONFIG REQUIRED)
find_package(fcl CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(urdfdom_headers CONFIG REQUIRED)
find_package(urdfdom CONFIG REQUIRED)

# ---------- Resources (URDF etc.) ----------
include(GNUInstallDirs)
set(RESOURCE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")
set(BUILD_RESOURCE_DIR   "${CMAKE_BINARY_DIR}/resources")
set(INSTALL_RESOURCE_DIR "${CMAKE_INSTALL_FULL_DATADIR}/path_planning")

if(EXISTS "${RESOURCE_SOURCE_DIR}")
  add_custom_target(copy_resources ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${RESOURCE_SOURCE_DIR}"
            "${BUILD_RESOURCE_DIR}"
    COMMENT "Copying resources to build directory")
endif()

# Example generated header, if you use it
if(EXISTS "${CMAKE_SOURCE_DIR}/cmake/config.hpp.in")
  configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/config.hpp.in"
    "${CMAKE_BINARY_DIR}/generated/config.hpp"
    @ONLY
  )
endif()

# =================== Python package layout (pure-Python) ===================
# Put your Python package under: python/path_planning/
#   python/path_planning/__init__.py
#   python/path_planning/helper.py
#   (optional) python/path_planning/data/...

# Install pure-Python files into site-packages/path_planning (no byte-compile)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/python/path_planning/"
        DESTINATION "${PY_SITEARCH_CMAKE}/path_planning"
        FILES_MATCHING
          PATTERN "*.py"
          PATTERN "__pycache__" EXCLUDE
)

# Optional: install extra data alongside Python package
if(EXISTS "${RESOURCE_SOURCE_DIR}")
  install(DIRECTORY "${RESOURCE_SOURCE_DIR}/"
          DESTINATION "${PY_SITEARCH_CMAKE}/path_planning/resources")
endif()

# =================== C++ extension: path_planning.core ===================
# IMPORTANT: PYBIND11_MODULE(core, m) in your bindings.cpp
pybind11_add_module(path_planning_core
  src/bindings.cpp
  # src/your_other_sources.cpp
)

# Make the built filename be "core.{so,pyd}" to match PYBIND11_MODULE(core, ...)
set_target_properties(path_planning_core PROPERTIES
  OUTPUT_NAME core
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

target_link_libraries(path_planning_core PRIVATE
  ompl::ompl
  fcl
  Eigen3::Eigen
  urdfdom_headers::urdfdom_headers
  urdfdom::urdfdom_model
  "${PY_LIB}"                       # link the selected Python explicitly (Windows)
)

target_include_directories(path_planning_core PRIVATE
  "${CMAKE_BINARY_DIR}/generated"
  "${PY_INCLUDE}"                   # selected Python headers
)

if(TARGET copy_resources)
  add_dependencies(path_planning_core copy_resources)
endif()

# Copy dependent DLLs next to the built .pyd (Windows dev quality-of-life)
if(MSVC)
  add_custom_command(TARGET path_planning_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Copying runtime DLLs for path_planning.core..."
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:path_planning_core>
            $<TARGET_FILE_DIR:path_planning_core>
    COMMAND_EXPAND_LISTS
  )
endif()

# ---------- Install: compiled module under the Python package ----------
install(TARGETS path_planning_core
  LIBRARY DESTINATION "${PY_SITEARCH_CMAKE}/path_planning"
  RUNTIME DESTINATION "${PY_SITEARCH_CMAKE}/path_planning"   # .pyd on Windows
  ARCHIVE DESTINATION "${PY_SITEARCH_CMAKE}/path_planning"
)

# Also install dependent DLLs next to the installed .pyd (Windows Release configs)
install(FILES $<TARGET_RUNTIME_DLLS:path_planning_core>
        DESTINATION "${PY_SITEARCH_CMAKE}/path_planning"
        CONFIGURATIONS Release RelWithDebInfo MinSizeRel)

# ---------- Linux rpaths and optional extra .so installs ----------
if(UNIX AND NOT APPLE)
  # where vcpkg put the built libs
  set(_vcpkg_libdir "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/lib")

  # let the built module find libs in build dir and installed dir
  set_target_properties(path_planning_core PROPERTIES
    BUILD_RPATH   "${_vcpkg_libdir}"
    INSTALL_RPATH "$ORIGIN"
  )

  # install selected shared libs next to the module, if you need them
  install(DIRECTORY "${_vcpkg_libdir}/"
          DESTINATION "${PY_SITEARCH_CMAKE}/path_planning"
          FILES_MATCHING
            REGEX "lib(ompl|fcl|ccd|console_bridge|urdfdom.*|boost_.*)\\.so(\\..*)?$")
endif()
