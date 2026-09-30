# Qt desktop app (Qt6). Replaces autoremesher.pro (qmake).
# Binary keeps the upstream `autoremesher` name; product renaming is a
# separate branding step.

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets
    PATHS /opt/homebrew/opt/qtbase /usr/local/opt/qtbase)

set(APP_SOURCES
    main.cpp
    logbrowser.cpp
    logbrowserdialog.cpp
    spinnableawesomebutton.cpp
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
    ../thirdparty/QtAwesome/QtAwesome/QtAwesome.cpp
    ../thirdparty/QtAwesome/QtAwesome/QtAwesomeAnim.cpp
    ../thirdparty/QtAwesome/QtAwesome/QtAwesome.h
    ../thirdparty/QtAwesome/QtAwesome/QtAwesomeAnim.h
    ../thirdparty/QtAwesome/QtAwesome/QtAwesome.qrc
    spinnerwidget.cpp
    spinnerwidget.h
)

add_executable(autoremesher ${APP_SOURCES})

# App-side named modules, one per converted plain component. Q_OBJECT
# widgets stay in headers: moc generates member definitions plus Qt
# includes that cannot coexist inside module purview (proven by pilot).
target_sources(autoremesher PRIVATE FILE_SET CXX_MODULES FILES
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
    spinnableawesomebutton.cppm
)
set_target_properties(autoremesher PROPERTIES CXX_SCAN_FOR_MODULES ON)

set_target_properties(autoremesher PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_NAME "AutoRemesher"
    MACOSX_BUNDLE_ICON_FILE "autoremesher.icns"
)

if(APPLE)
    target_sources(autoremesher PRIVATE autoremesher.icns)
    set_source_files_properties(autoremesher.icns PROPERTIES
        MACOSX_PACKAGE_LOCATION "Resources")
endif()

# Version/identity defines (were qmake DEFINES in autoremesher.pro).
# Fork restarts versioning at 0.1.0; support URLs point at the fork,
# while product/company display names keep upstream values for now.
if(APPLE)
    set(RETOPO_PLATFORM "MacOS")
elseif(WIN32)
    set(RETOPO_PLATFORM "Win32")
else()
    set(RETOPO_PLATFORM "Linux")
endif()
target_compile_definitions(autoremesher PRIVATE
    PROJECT_DEFINED_APP_COMPANY="Dust3D"
    PROJECT_DEFINED_APP_NAME="AutoRemesher"
    PROJECT_DEFINED_APP_VER="0.1.0.0"
    PROJECT_DEFINED_APP_HUMAN_VER="0.1.0"
    PROJECT_DEFINED_APP_HOMEPAGE_URL="https://github.com/chris-straka/retopoforge"
    PROJECT_DEFINED_APP_REPOSITORY_URL="https://github.com/chris-straka/retopoforge"
    PROJECT_DEFINED_APP_ISSUES_URL="https://github.com/chris-straka/retopoforge/issues"
    PROJECT_DEFINED_APP_PLATFORM="${RETOPO_PLATFORM}"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(autoremesher PRIVATE $<$<CONFIG:Release>:-O3 -funroll-loops>)
elseif(MSVC)
    target_compile_options(autoremesher PRIVATE $<$<CONFIG:Release>:/O2 /fp:fast> /bigobj)
    target_compile_definitions(autoremesher PRIVATE _USE_MATH_DEFINES NOMINMAX)
endif()

target_include_directories(autoremesher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/QtAwesome/QtAwesome
)

target_link_libraries(autoremesher PRIVATE
    retopo_core
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)
