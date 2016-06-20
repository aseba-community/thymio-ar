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

defined(THYMIO_AR_IMWRITE,var) {
    DEFINES += THYMIO_AR_IMWRITE
    android {
        OPENCV_LIBS += -lopencv_imgcodecs -lIlmImf -llibjpeg -llibwebp -llibtiff -llibpng -llibjasper
    } else {
        OPENCV_INCLUDE += $$OPENCV_SRC/modules/videoio/include $$OPENCV_SRC/modules/imgcodecs/include
        OPENCV_LIBS += -lopencv_videoio -lopencv_imgcodecs
    }
}

TRACKER_SOURCES = \
    $$PWD/thymio-tracker/src/BlobInertia.cpp \
    $$PWD/thymio-tracker/src/Generic.cpp \
    $$PWD/thymio-tracker/src/GH.cpp \
    $$PWD/thymio-tracker/src/GHscale.cpp \
    $$PWD/thymio-tracker/src/Grouping.cpp \
    $$PWD/thymio-tracker/src/Landmark.cpp \
    $$PWD/thymio-tracker/src/Models.cpp \
    $$PWD/thymio-tracker/src/Robot.cpp \
    $$PWD/thymio-tracker/src/TrackingFcts.cpp \
    $$PWD/thymio-tracker/src/ThymioTracker.cpp

QT += quick multimedia sensors 3dcore 3drender
CONFIG += c++11
HEADERS += \
    $$PWD/thymio-ar.h \
	$$PWD/vision-video-filter.h
SOURCES += \
    $$ASEBA_SOURCES \
    $$TRACKER_SOURCES \
    $$PWD/thymio-ar.cpp \
	$$PWD/vision-video-filter.cpp
RESOURCES += $$PWD/thymio-ar.qrc
DEPENDPATH += $$OPENCV_INCLUDE $$EIGEN_INCLUDE
INCLUDEPATH += $$OPENCV_INCLUDE $$EIGEN_INCLUDE
LIBS += $$OPENCV_LIBS

include(thymio-vpl2/thymio-vpl2.pri)
