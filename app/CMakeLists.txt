# Qt desktop app (Qt6). Replaces autoremesher.pro (qmake).
# Binary and macOS bundle are named `retopoforge`.

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets
    PATHS /opt/homebrew/opt/qtbase /usr/local/opt/qtbase)

set(APP_SOURCES
    main.cpp
    logbrowser.cpp
    logbrowserdialog.cpp
    util.cpp
    mainwindow.cpp
    aboutwidget.cpp
    theme.cpp
    graphicscontainerwidget.cpp
    graphicswidget.cpp
    modelshadermesh.cpp
    modelshadermeshbinder.cpp
    modelshaderprogram.cpp
    monochromeopenglprogram.cpp
    monochromeopenglobject.cpp
    modelshaderwidget.cpp
    rendermeshgenerator.cpp
    previewmeshgenerator.cpp
    quadmeshgenerator.cpp
    preferences.cpp
    floatnumberwidget.cpp
    intnumberwidget.cpp
    logbrowser.h
    logbrowserdialog.h
    version.h
    mainwindow.h
    aboutwidget.h
    graphicscontainerwidget.h
    graphicswidget.h
    modelshaderwidget.h
    rendermeshgenerator.h
    previewmeshgenerator.h
    quadmeshgenerator.h
    preferences.h
    floatnumberwidget.h
    intnumberwidget.h
    resources.qrc
    spinnerwidget.cpp
    spinnerwidget.h
)

add_executable(retopoforge ${APP_SOURCES})

# App-side named modules, one per converted plain component. Q_OBJECT
# widgets stay in headers: moc generates member definitions plus Qt
# includes that cannot coexist inside module purview (proven by pilot).
target_sources(retopoforge PRIVATE FILE_SET CXX_MODULES FILES
    modelshadervertex.cppm
    monochromeopenglvertex.cppm
    openglbufferutil.cppm
    modelshaderprogram.cppm
    monochromeopenglprogram.cppm
    util.cppm
    modelshadermesh.cppm
    monochromeopenglobject.cppm
    modelshadermeshbinder.cppm
    theme.cppm
)
set_target_properties(retopoforge PROPERTIES CXX_SCAN_FOR_MODULES ON)

set_target_properties(retopoforge PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_NAME "retopoforge"
    MACOSX_BUNDLE_GUI_IDENTIFIER "io.github.chris-straka.retopoforge"
    MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
    MACOSX_BUNDLE_ICON_FILE "retopoforge.icns"
)

if(APPLE)
    target_sources(retopoforge PRIVATE retopoforge.icns)
    set_source_files_properties(retopoforge.icns PROPERTIES
        MACOSX_PACKAGE_LOCATION "Resources")
endif()

# Version/identity defines (were qmake DEFINES in autoremesher.pro).
# Fork restarts versioning at 0.1.0; support URLs point at the fork.
# APP_NAME is the fork product name; APP_COMPANY keeps the upstream
# Dust3D credit.
if(APPLE)
    set(RETOPO_PLATFORM "MacOS")
elseif(WIN32)
    set(RETOPO_PLATFORM "Win32")
else()
    set(RETOPO_PLATFORM "Linux")
endif()
target_compile_definitions(retopoforge PRIVATE
    PROJECT_DEFINED_APP_COMPANY="Dust3D"
    PROJECT_DEFINED_APP_NAME="retopoforge"
    PROJECT_DEFINED_APP_VER="0.1.0.0"
    PROJECT_DEFINED_APP_HUMAN_VER="0.1.0"
    PROJECT_DEFINED_APP_HOMEPAGE_URL="https://github.com/chris-straka/retopoforge"
    PROJECT_DEFINED_APP_REPOSITORY_URL="https://github.com/chris-straka/retopoforge"
    PROJECT_DEFINED_APP_ISSUES_URL="https://github.com/chris-straka/retopoforge/issues"
    PROJECT_DEFINED_APP_PLATFORM="${RETOPO_PLATFORM}"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(retopoforge PRIVATE $<$<CONFIG:Release>:-O3 -funroll-loops>)
elseif(MSVC)
    target_compile_options(retopoforge PRIVATE $<$<CONFIG:Release>:/O2 /fp:fast> /bigobj)
    target_compile_definitions(retopoforge PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()

target_include_directories(retopoforge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(retopoforge PRIVATE
    retopo_core
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)
