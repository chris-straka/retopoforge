cmake_minimum_required(VERSION 3.16)
project(retopoforge VERSION 0.1.0 LANGUAGES C CXX)

# retopoforge: fork of huxingyi/autoremesher (MIT), restructured around a
# Qt-free headless engine + CLI + Qt6 desktop app. One CMake build covers
# everything: retopo_core, the retopo CLI, and the autoremesher app.

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Core static library (Qt-free) ---
# Source list mirrors autoremesher.pro minus the Qt GUI shell.
set(CORE_SOURCES
    core/autoremesher.cpp
    core/isotropicremesher.cpp
    core/parameterizer.cpp
    core/surfacemesh.cpp
    core/singularitysimplifier.cpp
    core/constrainedleastsquares.cpp
    core/mixedintegerleastsquares.cpp
    core/framefield.cpp
    core/quadparameterizer.cpp
    core/quadextractor.cpp
    core/positionkey.cpp
    core/meshseparator.cpp
    thirdparty/isotropicremesher/isotropicremesher.cpp
    thirdparty/isotropicremesher/isotropichalfedgemesh.cpp
    thirdparty/isotropicremesher/axisalignedboundingboxtree.cpp
    thirdparty/meshoptimizer/src/simplifier.cpp
    thirdparty/meshoptimizer/src/indexgenerator.cpp
)

add_library(retopo_core STATIC ${CORE_SOURCES})

# C++ named modules, one per converted component (importable by dependents).
target_sources(retopo_core PUBLIC FILE_SET CXX_MODULES FILES
    core/constrainedleastsquares.cppm
    core/framefield.cppm
    core/isotropicremesher.cppm
    core/meshseparator.cppm
    core/mixedintegerleastsquares.cppm
    core/parameterizer.cppm
    core/positionkey.cppm
    core/quadparameterizer.cppm
    core/singularitysimplifier.cppm
    core/surfacemesh.cppm
)
# Scan every source for module use: without this, plain .cpp importers are
# compiled unscanned (no BMI flags, no ordering after the module build).
set_target_properties(retopo_core PROPERTIES CXX_SCAN_FOR_MODULES ON)

target_include_directories(retopo_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/isotropicremesher
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/meshoptimizer/src
)

target_compile_definitions(retopo_core PUBLIC _USE_MATH_DEFINES NOMINMAX)

if(APPLE)
    target_compile_definitions(retopo_core PUBLIC AUTO_REMESHER_USE_ACCELERATE=1)
    target_link_libraries(retopo_core PUBLIC "-framework Accelerate")
else()
    target_compile_definitions(retopo_core PUBLIC AUTO_REMESHER_USE_ACCELERATE=0)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(retopo_core PRIVATE $<$<CONFIG:Release>:-O3 -funroll-loops>)
elseif(MSVC)
    target_compile_options(retopo_core PRIVATE $<$<CONFIG:Release>:/O2 /fp:fast> /bigobj)
endif()

# TBB (system install; Homebrew hints for macOS)
find_package(TBB REQUIRED PATHS /opt/homebrew/opt/tbb /usr/local/opt/tbb)
target_link_libraries(retopo_core PUBLIC TBB::tbb)
if(TARGET TBB::tbbmalloc)
    target_link_libraries(retopo_core PUBLIC TBB::tbbmalloc)
endif()

# zlib (matches the unix link in autoremesher.pro)
if(UNIX)
    find_package(ZLIB)
    if(ZLIB_FOUND)
        target_link_libraries(retopo_core PUBLIC ZLIB::ZLIB)
    endif()
endif()

add_subdirectory(cli)

option(RETOPOFORGE_BUILD_QT_APP "Build the Qt6 desktop app" ON)
if(RETOPOFORGE_BUILD_QT_APP)
    add_subdirectory(app)
endif()
