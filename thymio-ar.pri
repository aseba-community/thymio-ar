OPENCV_LIBS_MODULES = -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_imgproc -lopencv_video
android {
    !defined(OPENCV_SDK,var):error(undefined OPENCV_SDK variable)
    OPENCV_INCLUDE = $$OPENCV_SDK/sdk/native/jni/include
    OPENCV_LIBS = -L$$OPENCV_SDK/sdk/native/3rdparty/libs/armeabi-v7a -L$$OPENCV_SDK/sdk/native/libs/armeabi-v7a \
        -Wl,--start-group $$OPENCV_LIBS_MODULES -ltbb -Wl,--end-group
} else {
    !defined(OPENCV_SRC,var):warning(undefined OPENCV_SRC variable)
    !defined(OPENCV_BIN,var):warning(undefined OPENCV_BIN variable)
    OPENCV_INCLUDE = \
        $$OPENCV_SRC/modules/calib3d/include \
        $$OPENCV_SRC/modules/core/include \
        $$OPENCV_SRC/modules/features2d/include \
        $$OPENCV_SRC/modules/flann/include \
        $$OPENCV_SRC/modules/imgproc/include \
        $$OPENCV_SRC/modules/video/include
    OPENCV_LIBS = -L$$OPENCV_BIN/lib $$OPENCV_LIBS_MODULES
}

defined(THYMIO_AR_IMWRITE) {
    DEFINES += THYMIO_AR_IMWRITE
    android {
        OPENCV_LIBS += -lopencv_imgcodecs -lIlmImf -llibjpeg -llibwebp -llibtiff -llibpng -llibjasper
    } else {
        OPENCV_INCLUDE += $$OPENCV_SRC/modules/videoio/include $$OPENCV_SRC/modules/imgcodecs/include
        OPENCV_LIBS += -lopencv_videoio -lopencv_imgcodecs
    }
}

!defined(EIGEN_INCLUDE,var):error(undefined EIGEN_INCLUDE variable)

ASEBA_SOURCES = \
    $$PWD/dashel/dashel/dashel-common.cpp \
    $$PWD/aseba/common/utils/FormatableString.cpp \
    $$PWD/aseba/common/utils/utils.cpp \
    $$PWD/aseba/common/utils/HexFile.cpp \
    $$PWD/aseba/common/utils/BootloaderInterface.cpp \
    $$PWD/aseba/common/msg/msg.cpp \
    $$PWD/aseba/common/msg/descriptions-manager.cpp \
    $$PWD/aseba/compiler/compiler.cpp \
    $$PWD/aseba/compiler/errors.cpp \
    $$PWD/aseba/compiler/identifier-lookup.cpp \
    $$PWD/aseba/compiler/lexer.cpp \
    $$PWD/aseba/compiler/parser.cpp \
    $$PWD/aseba/compiler/analysis.cpp \
    $$PWD/aseba/compiler/tree-build.cpp \
    $$PWD/aseba/compiler/tree-expand.cpp \
    $$PWD/aseba/compiler/tree-dump.cpp \
    $$PWD/aseba/compiler/tree-typecheck.cpp \
    $$PWD/aseba/compiler/tree-optimize.cpp \
    $$PWD/aseba/compiler/tree-emit.cpp
win32 {
    ASEBA_SOURCES += $$PWD/dashel/dashel/dashel-win32.cpp
} else {
    ASEBA_SOURCES += $$PWD/dashel/dashel/dashel-posix.cpp
    macx {
        ASEBA_SOURCES += $$PWD/dashel/dashel/poll_emu.c
        ASEBA_LIBS = -framework CoreFoundation
    }
}
android {
    ASEBA_SOURCES += $$PWD/aseba/transport/dashel_plugins/android.cpp
    aseba_android.files = $$PWD/android/*
    aseba_android.path = /
    INSTALLS += aseba_android
} else {
    ASEBA_SOURCES += $$PWD/aseba/transport/dashel_plugins/none.cpp
}
ASEBA_INCLUDE = $$PWD/dashel $$PWD
ASEBA_CXXFLAGS = -Wno-unused-parameter -Wno-deprecated-declarations

TRACKER_SOURCES = \
    $$PWD/thymio-tracker/src/BlobInertia.cpp \
    $$PWD/thymio-tracker/src/Generic.cpp \
    $$PWD/thymio-tracker/src/GH.cpp \
    $$PWD/thymio-tracker/src/GHscale.cpp \
    $$PWD/thymio-tracker/src/Grouping.cpp \
    $$PWD/thymio-tracker/src/Landmark.cpp \
    $$PWD/thymio-tracker/src/Models.cpp \
    $$PWD/thymio-tracker/src/Robot.cpp \
    $$PWD/thymio-tracker/src/ThymioTracker.cpp

QT += quick multimedia sensors 3dcore 3drender
CONFIG += c++11
QMAKE_CXXFLAGS += $$ASEBA_CXXFLAGS
HEADERS += \
    $$PWD/thymio-ar.h \
    $$PWD/vision-video-filter.h \
    $$PWD/aseba.h
SOURCES += \
    $$ASEBA_SOURCES \
    $$TRACKER_SOURCES \
    $$PWD/thymio-ar.cpp \
    $$PWD/vision-video-filter.cpp \
    $$PWD/aseba.cpp
RESOURCES += $$PWD/thymio-ar.qrc
DEPENDPATH += $$OPENCV_INCLUDE $$EIGEN_INCLUDE $$ASEBA_INCLUDE
INCLUDEPATH += $$OPENCV_INCLUDE $$EIGEN_INCLUDE $$ASEBA_INCLUDE
LIBS += $$OPENCV_LIBS $$ASEBA_LIBS

include(thymio-vpl2/thymio-vpl2.pri)
