QT       += core

QT       -= gui

TARGET = TJ
CONFIG   += console
CONFIG   -= app_bundle
CONFIG   +=C++11

TEMPLATE = app

INCLUDEPATH +=/usr/local/include              \
              /usr/local/include/opencv       \
              /usr/local/include/opencv2      \
              /usr/local/include/eigen3

LIBS += -L/usr/local/lib                        \
        -lopencv_features2d                     \
        -lopencv_calib3d                        \
        -lopencv_contrib                        \
        -lopencv_core                           \
        -lopencv_flann                          \
        -lopencv_gpu                            \
        -lopencv_ml                             \
        -lopencv_imgproc                        \
        -lopencv_video                          \
        -lopencv_legacy                         \
        -lopencv_highgui                        \
        -lopencv_stitching                      \
        -lopencv_photo                          \
        -lopencv_ocl                            \
        -lopencv_videostab                      \
        -lopencv_superres                       \
        -lopencv_nonfree                        \
        -lgdal


HEADERS += \
    core.h \
    match.h \
    delaunay.h \
    utils.h

SOURCES += \
    main.cpp \
    utils.cpp \
    core.cpp \
    match.cpp \
    delaunay.cpp

QMAKE_CXXFLAGS += -fopenmp
#QMAKE_MAC_SDK = macosx10.11

#LIBS += -fopenmp

