# -------------------------------------------------
# Project created by QtCreator 2015-10-12T18:55:22
# -------------------------------------------------
TARGET = SmartProjector 
TEMPLATE = app

Release:DESTDIR = ./output
Release:TARGET = SmartProjector 

Debug:DESTDIR = ./output
Debug:TARGET = SmartProjectorD 

PCL_PATH = "C:/Program Files/PCL 1.6.0" 
OPENCV_PATH = C:/_program/opencv/build
OPENCV_VER = 249


# OpenCV Libs for Debug mode
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_calib3d$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_contrib$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_core$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_features2d$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_flann$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_gpu$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_highgui$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_imgproc$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_legacy$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_ml$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_nonfree$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_objdetect$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_photo$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_stitching$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_superres$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_ts$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_video$${OPENCV_VER}d.lib
Debug:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_videostab$${OPENCV_VER}d.lib


# OpenCV Libs for Release
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_calib3d$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_contrib$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_core$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_features2d$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_flann$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_gpu$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_highgui$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_imgproc$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_legacy$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_ml$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_nonfree$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_objdetect$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_photo$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_stitching$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_superres$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_ts$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_video$${OPENCV_VER}.lib
Release:LIBS += $${OPENCV_PATH}/x64/vc12/lib/opencv_videostab$${OPENCV_VER}.lib

Release:LIBS += C:/_program/libfreenect2-master/build/lib/Release/freenect2.lib
Release:LIBS += C:/_program/libfreenect2-master/depends/glfw/lib-vc2013/glfw3dll.lib
Release:LIBS += C:/_program/libfreenect2-master/depends/libjpeg-turbo64/lib/turbojpeg.lib
Release:LIBS += C:/_program/libfreenect2-master/depends/libusb/x64/Release/dll/libusb-1.0.lib
Release:LIBS += "C:/Program Files (x86)/Intel/OpenCL SDK/4.6/lib/x64/OpenCL.lib"

Debug:LIBS += C:/_program/libfreenect2-master/build/lib/Debug/freenect2d.lib
Debug:LIBS += C:/_program/libfreenect2-master/depends/glfw/lib-vc2013/glfw3dll.lib
Debug:LIBS += C:/_program/libfreenect2-master/depends/libjpeg-turbo64/lib/turbojpeg.lib
Debug:LIBS += C:/_program/libfreenect2-master/depends/libusb/x64/Release/dll/libusb-1.0.lib
Debug:LIBS += "C:/Program Files (x86)/Intel/OpenCL SDK/4.6/lib/x64/OpenCL.lib"
# OpenCV includes
INCLUDEPATH += $${OPENCV_PATH}/include/opencv
INCLUDEPATH += $${OPENCV_PATH}/include

# Freenect2 includes
INCLUDEPATH += C:/_program/libfreenect2-master/include
INCLUDEPATH += C:/_program/libfreenect2-master/include/internal
INCLUDEPATH += C:/_program/libfreenect2-master/build
INCLUDEPATH += C:/_program/libfreenect2-master/src/tinythread
INCLUDEPATH += C:/_program/libfreenect2-master/depends/libusb/libusb
INCLUDEPATH += C:/_program/libfreenect2-master/depends/libjpeg-turbo64/include
INCLUDEPATH += C:/_program/libfreenect2-masterldepends/glfw/include

# OpenCL includes
INCLUDEPATH += C:/Program Files (x86)/Intel/OpenCL SDK/4.6/include


QT +=widgets
QT +=gui
SOURCES += main.cpp \
    SmartProjectorMainWindow.cpp \
    QImageWidget.cpp \
    optimalRectDDJ.cpp

    

HEADERS += SmartProjectorMainWindow.h \
           QImageWidget.h \
           optimalRectDDJ.h

FORMS += SmartProjectorMainWindow.ui \

RESOURCES += SmartProjectorMainWindow.qrc 
