QT += core widgets opengl
# QOpenGLWidget moved into its own module in Qt 6.
greaterThan(QT_MAJOR_VERSION, 5): QT += openglwidgets
win32 {
    qtHaveModule(winextras) {
        QT += winextras
    }
}
CONFIG += release
CONFIG(release, debug|release) DEFINES += NDEBUG
CONFIG(debug, debug|release) DEFINES += AUTO_REMESHER_DEBUG
CONFIG(debug, debug|release) DEFINES += QT_MESSAGELOGCONTEXT
RESOURCES += app/resources.qrc

CONFIG += object_parallel_to_source

CONFIG(debug, debug|release) OBJECTS_DIR=obj-debug
CONFIG(release, debug|release) OBJECTS_DIR=obj
CONFIG(debug, debug|release) MOC_DIR=moc-debug
CONFIG(release, debug|release) MOC_DIR=moc

win32 {
    CONFIG(debug, debug|release) CONFIG += force_debug_info
	RC_FILE = app/autoremesher.rc
}

macx {
	ICON = app/autoremesher.icns

	RESOURCE_FILES.files = $$ICON
	RESOURCE_FILES.path = Contents/Resources
	QMAKE_BUNDLE_DATA += RESOURCE_FILES
}

isEmpty(HUMAN_VERSION) {
	HUMAN_VERSION = "1.2.0"
}
isEmpty(VERSION) {
	VERSION = 1.2.0.0
}

HOMEPAGE_URL = "https://autoremesher.dust3d.org/"
REPOSITORY_URL = "https://github.com/huxingyi/autoremesher"
ISSUES_URL = "https://github.com/huxingyi/autoremesher/issues"
PLATFORM = "Unknown"

PLATFORM = "Unknown"
macx {
	PLATFORM = "MacOS"
}
win32 {
	PLATFORM = "Win32"
}
unix:!macx {
	PLATFORM = "Linux"
}

QMAKE_TARGET_COMPANY = Dust3D
QMAKE_TARGET_PRODUCT = AutoRemesher
QMAKE_TARGET_DESCRIPTION = "AutoRemesher is a cross-platform open-source automatic quad remeshing software"
QMAKE_TARGET_COPYRIGHT = "Copyright (C) 2020 AutoRemesher Project. All Rights Reserved."

DEFINES += "PROJECT_DEFINED_APP_COMPANY=\"\\\"$$QMAKE_TARGET_COMPANY\\\"\""
DEFINES += "PROJECT_DEFINED_APP_NAME=\"\\\"$$QMAKE_TARGET_PRODUCT\\\"\""
DEFINES += "PROJECT_DEFINED_APP_VER=\"\\\"$$VERSION\\\"\""
DEFINES += "PROJECT_DEFINED_APP_HUMAN_VER=\"\\\"$$HUMAN_VERSION\\\"\""
DEFINES += "PROJECT_DEFINED_APP_HOMEPAGE_URL=\"\\\"$$HOMEPAGE_URL\\\"\""
DEFINES += "PROJECT_DEFINED_APP_REPOSITORY_URL=\"\\\"$$REPOSITORY_URL\\\"\""
DEFINES += "PROJECT_DEFINED_APP_ISSUES_URL=\"\\\"$$ISSUES_URL\\\"\""
DEFINES += "PROJECT_DEFINED_APP_PLATFORM=\"\\\"$$PLATFORM\\\"\""

# C++23: Qt5 qmake predates the c++23 knob; c++2b is its name for it
# (maps to -std=c++2b on clang/gcc, /std:c++latest on MSVC).
CONFIG += c++2b

macx {
	QMAKE_CXXFLAGS_RELEASE -= -O
	QMAKE_CXXFLAGS_RELEASE -= -O1
	QMAKE_CXXFLAGS_RELEASE -= -O2

	QMAKE_CXXFLAGS_RELEASE += -O3
	# LTO and aggressive loop optimizations — trade code size for speed
	QMAKE_CXXFLAGS_RELEASE += -flto -funroll-loops
	QMAKE_LFLAGS_RELEASE += -flto

	QMAKE_CFLAGS += -DNL_USE_BLAS
	QMAKE_CXXFLAGS += -DNL_USE_BLAS
	DEFINES += AUTO_REMESHER_USE_ACCELERATE=1
	LIBS += -framework Accelerate
}

unix:!macx {
	QMAKE_CXXFLAGS_RELEASE -= -O
	QMAKE_CXXFLAGS_RELEASE -= -O1
	QMAKE_CXXFLAGS_RELEASE -= -O2

	QMAKE_CXXFLAGS_RELEASE += -O3
	# LTO, aggressive loop unrolling, and x86-64-v2 baseline for broad CPU compatibility
	QMAKE_CXXFLAGS_RELEASE += -flto -funroll-loops -march=x86-64-v2
	QMAKE_LFLAGS_RELEASE += -flto
}

win32-msvc* {
	CONFIG(debug, debug|release) QMAKE_CXXFLAGS += /Od
	CONFIG(release, debug|release) QMAKE_CXXFLAGS += /O2 /GL /Qpar /fp:fast
	CONFIG(release, debug|release) QMAKE_LFLAGS += /LTCG
	QMAKE_CXXFLAGS += /bigobj
}

win32-g++ {
	# MinGW GCC does not understand MSVC-style /flags. /bigobj has no MinGW
	# equivalent flag, but Eigen produces very large translation
	# units that need the big-obj object format, enabled via the assembler.
	# Optimization (-O2) already comes from the default release CONFIG.
	QMAKE_CXXFLAGS += -Wa,-mbig-obj
}

DEFINES += _USE_MATH_DEFINES
DEFINES += NOMINMAX

# Qt's qyieldcpu.h calls the __yield ARM builtin without including
# <arm_acle.h>. Since clang 16, -Wimplicit-function-declaration is an error by
# default, which makes Qt headers fail to compile (seen with Qt 6 + clang on
# Apple Silicon). Downgrade it to a warning so the Qt headers build. Scoped to
# clang/gcc (MSVC in the win32 block ignores these flags).
!win32 {
    QMAKE_CXXFLAGS += -Wno-error=implicit-function-declaration
    QMAKE_CFLAGS += -Wno-error=implicit-function-declaration
}

include(thirdparty/QtAwesome/QtAwesome/QtAwesome.pri)

INCLUDEPATH += thirdparty/QtWaitingSpinner

SOURCES += thirdparty/QtWaitingSpinner/waitingspinnerwidget.cpp
HEADERS += thirdparty/QtWaitingSpinner/waitingspinnerwidget.h

INCLUDEPATH += thirdparty/eigen

INCLUDEPATH += core/include

SOURCES += app/main.cpp

SOURCES += app/logbrowser.cpp
HEADERS += app/logbrowser.h

SOURCES += app/logbrowserdialog.cpp
HEADERS += app/logbrowserdialog.h

SOURCES += app/spinnableawesomebutton.cpp
HEADERS += app/spinnableawesomebutton.h

SOURCES += app/util.cpp
HEADERS += app/util.h

SOURCES += app/mainwindow.cpp
HEADERS += app/mainwindow.h

SOURCES += app/aboutwidget.cpp
HEADERS += app/aboutwidget.h

SOURCES += app/theme.cpp
HEADERS += app/theme.h

SOURCES += app/graphicscontainerwidget.cpp
HEADERS += app/graphicscontainerwidget.h

SOURCES += app/graphicswidget.cpp
HEADERS += app/graphicswidget.h

SOURCES += app/modelshadermesh.cpp
HEADERS += app/modelshadermesh.h

SOURCES += app/modelshadermeshbinder.cpp
HEADERS += app/modelshadermeshbinder.h

SOURCES += app/modelshaderprogram.cpp
HEADERS += app/modelshaderprogram.h

HEADERS += app/modelshadervertex.h

SOURCES += app/monochromeopenglprogram.cpp
HEADERS += app/monochromeopenglprogram.h

SOURCES += app/monochromeopenglobject.cpp
HEADERS += app/monochromeopenglobject.h

HEADERS += app/openglbufferutil.h

HEADERS += app/monochromeopenglvertex.h

SOURCES += app/modelshaderwidget.cpp
HEADERS += app/modelshaderwidget.h

SOURCES += app/rendermeshgenerator.cpp
HEADERS += app/rendermeshgenerator.h

SOURCES += app/previewmeshgenerator.cpp
HEADERS += app/previewmeshgenerator.h

SOURCES += app/quadmeshgenerator.cpp
HEADERS += app/quadmeshgenerator.h

SOURCES += app/preferences.cpp
HEADERS += app/preferences.h

SOURCES += app/floatnumberwidget.cpp
HEADERS += app/floatnumberwidget.h

SOURCES += app/intnumberwidget.cpp
HEADERS += app/intnumberwidget.h

SOURCES += core/autoremesher.cpp
HEADERS += core/autoremesher.h

SOURCES += core/isotropicremesher.cpp
HEADERS += core/isotropicremesher.h

INCLUDEPATH += thirdparty/isotropicremesher
INCLUDEPATH += thirdparty/tinyobjloader
SOURCES += thirdparty/isotropicremesher/isotropicremesher.cpp
SOURCES += thirdparty/isotropicremesher/isotropichalfedgemesh.cpp
SOURCES += thirdparty/isotropicremesher/axisalignedboundingboxtree.cpp
HEADERS += thirdparty/isotropicremesher/isotropicremesher.h
HEADERS += thirdparty/isotropicremesher/isotropichalfedgemesh.h
HEADERS += thirdparty/isotropicremesher/axisalignedboundingboxtree.h
HEADERS += thirdparty/isotropicremesher/axisalignedboundingbox.h
HEADERS += thirdparty/isotropicremesher/vector3.h
HEADERS += thirdparty/isotropicremesher/vector2.h
HEADERS += thirdparty/isotropicremesher/double.h

INCLUDEPATH += thirdparty/meshoptimizer/src
SOURCES += thirdparty/meshoptimizer/src/simplifier.cpp
SOURCES += thirdparty/meshoptimizer/src/indexgenerator.cpp
HEADERS += thirdparty/meshoptimizer/src/meshoptimizer.h

SOURCES += core/parameterizer.cpp
HEADERS += core/parameterizer.h

SOURCES += core/surfacemesh.cpp
HEADERS += core/surfacemesh.h
HEADERS += core/include/AutoRemesher/SurfaceMesh
SOURCES += core/singularitysimplifier.cpp
HEADERS += core/singularitysimplifier.h
HEADERS += core/include/AutoRemesher/SingularitySimplifier
SOURCES += core/constrainedleastsquares.cpp
HEADERS += core/constrainedleastsquares.h
HEADERS += core/include/AutoRemesher/ConstrainedLeastSquares
SOURCES += core/mixedintegerleastsquares.cpp
HEADERS += core/mixedintegerleastsquares.h
HEADERS += core/include/AutoRemesher/MixedIntegerLeastSquares
SOURCES += core/framefield.cpp
HEADERS += core/framefield.h
HEADERS += core/include/AutoRemesher/FrameField
SOURCES += core/quadparameterizer.cpp
HEADERS += core/quadparameterizer.h
HEADERS += core/include/AutoRemesher/QuadParameterizer


SOURCES += core/quadextractor.cpp
HEADERS += core/quadextractor.h


SOURCES += core/positionkey.cpp
HEADERS += core/positionkey.h

SOURCES += core/meshseparator.cpp
HEADERS += core/meshseparator.h

unix {
    LIBS += -lz
}

macx {
    INCLUDEPATH += /opt/homebrew/opt/tbb/include
    LIBS += -L/opt/homebrew/opt/tbb/lib -ltbbmalloc_proxy -ltbbmalloc -ltbb
}
unix:!macx {
    LIBS += -ltbb -lz -ldl
}
win32-msvc* {
    INCLUDEPATH += thirdparty/tbb/include
    CONFIG(release, debug|release) LIBS += -Lthirdparty/tbb/build2/Release -ltbb
}
win32-g++ {
    # MinGW: link the MSYS2 system oneTBB (pacman -S mingw-w64-x86_64-tbb).
    # Its import library is libtbb12.dll.a and its headers are already on the
    # default include path, so no vendored TBB headers or -L path are needed.
    LIBS += -ltbb12
}

win32 {
    LIBS += -luser32
	LIBS += -lopengl32
	# ole32 + uuid: CoCreateInstance and the ITaskbarList3 CLSID/IID GUIDs used
	# for Windows taskbar progress on Qt 6 (winextras replacement).
	LIBS += -lole32 -luuid
}

target.path = ./
INSTALLS += target
