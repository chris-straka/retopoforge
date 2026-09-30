cmake_minimum_required(VERSION 3.16)
project(retopoforge VERSION 0.1.0 LANGUAGES C CXX)

# retopoforge: fork of huxingyi/autoremesher (MIT), restructured around a
# Qt-free headless engine + CLI + Qt6 desktop app. One CMake build covers
# everything: retopo_core, the retopo CLI, and the retopoforge app.

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Every target gets module dependency scanning: plain .cpp importers (in the
# core library, the CLI, and the Qt app) need BMI flags and build ordering.
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

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
    core/objreader.cpp
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
    core/double.cppm
    core/framefield.cppm
    core/isotropicremesher.cppm
    core/meshseparator.cppm
    core/mixedintegerleastsquares.cppm
    core/parameterizer.cppm
    core/positionkey.cppm
    core/progress.cppm
    core/quadextractor.cppm
    core/quadparameterizer.cppm
    core/singularitysimplifier.cppm
    core/surfacemesh.cppm
    core/vector2.cppm
    core/vector3.cppm
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
if(TARGET retopoforge)
    # AUTOMOC's unity file includes moc outputs for app headers that import
    # retopo.core and retopo.app modules, but CMake compiles autogen sources
    # with the unscanned rule (no dyndep BMI flags), so hand the unity file
    # the module files it needs. Ordering uses only statically-known rules:
    # the finished static lib (completes after every core module compile)
    # and the app modules' own object files (each completes after its .pcm
    # exists). Never edge on a .pcm path directly: dyndep produces .pcms
    # with no static ninja rule, which fails fresh-build planning. Revisit
    # if CMake starts scanning autogen sources; extend both lists when new
    # app modules land.
    set(_RETOPO_MMI ${CMAKE_BINARY_DIR}/app/retopoforge_autogen/mocs_compilation.cpp)
    set(_RETOPO_PCM_DIR ${CMAKE_BINARY_DIR}/CMakeFiles/retopo_core.dir)
    set(_RETOPO_PCM_NAMES
        retopo.core.double_utils
        retopo.core.progress
        retopo.core.vector2
        retopo.core.vector3
        retopo.core.quad_extractor
    )
    set(_RETOPO_APP_PCM_DIR ${CMAKE_BINARY_DIR}/app/CMakeFiles/retopoforge.dir)
    set(_RETOPO_APP_PCM_NAMES
        retopo.app.model_shader_vertex
        retopo.app.monochrome_opengl_vertex
        retopo.app.opengl_buffer_util
        retopo.app.model_shader_program
        retopo.app.monochrome_opengl_program
        retopo.app.util
        retopo.app.model_shader_mesh
        retopo.app.monochrome_opengl_object
        retopo.app.model_shader_mesh_binder
        retopo.app.theme
        retopo.app.spinnable_awesome_button
    )
    set(_RETOPO_MMI_FLAGS "")
    foreach(_mod IN LISTS _RETOPO_PCM_NAMES)
        list(APPEND _RETOPO_MMI_FLAGS "-fmodule-file=${_mod}=${_RETOPO_PCM_DIR}/${_mod}.pcm")
    endforeach()
    set(_RETOPO_MMI_OBJS ${CMAKE_BINARY_DIR}/libretopo_core.a)
    foreach(_mod IN LISTS _RETOPO_APP_PCM_NAMES)
        list(APPEND _RETOPO_MMI_FLAGS "-fmodule-file=${_mod}=${_RETOPO_APP_PCM_DIR}/${_mod}.pcm")
    endforeach()
    foreach(_stem IN ITEMS modelshadervertex monochromeopenglvertex openglbufferutil
            modelshaderprogram monochromeopenglprogram util modelshadermesh
            monochromeopenglobject modelshadermeshbinder theme spinnableawesomebutton)
        list(APPEND _RETOPO_MMI_OBJS "${_RETOPO_APP_PCM_DIR}/${_stem}.cppm.o")
    endforeach()
    set_property(SOURCE ${_RETOPO_MMI} TARGET_DIRECTORY retopoforge
        APPEND PROPERTY COMPILE_OPTIONS ${_RETOPO_MMI_FLAGS})
    set_property(SOURCE ${_RETOPO_MMI} TARGET_DIRECTORY retopoforge
        APPEND PROPERTY OBJECT_DEPENDS ${_RETOPO_MMI_OBJS})
    unset(_RETOPO_MMI)
    unset(_RETOPO_PCM_DIR)
    unset(_RETOPO_PCM_NAMES)
    unset(_RETOPO_APP_PCM_DIR)
    unset(_RETOPO_APP_PCM_NAMES)
    unset(_RETOPO_MMI_FLAGS)
    unset(_RETOPO_MMI_OBJS)
    unset(_mod)
    unset(_stem)
endif()

enable_testing()
add_subdirectory(tests)
