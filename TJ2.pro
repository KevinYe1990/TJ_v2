QT       += core

QT       -= gui

TARGET = TJ
CONFIG   += console
CONFIG   -= app_bundle
CONFIG   +=C++11

TEMPLATE = app

INCLUDEPATH +=/usr/local/include              \
              /usr/local/include/pcl-1.8      \
              /usr/local/include/eigen3

LIBS += -L/usr/local/lib                        \
        -lpcl_io                                \
        -lpcl_kdtree                            \
        -lpcl_search                            \
        -lpcl_surface                           \
        -lpcl_features                          \
        -lpcl_common                            \
        -lpcl_registration                      \
        -lpcl_outofcore                         \
        -lpcl_octree                            \
        -lpcl_people                            \
        -lpcl_sample_consensus                  \
        -lpcl_recognition                       \
        -lpcl_segmentation                      \
        -lpcl_2d                                \
        -lpcl_filters                           \
        -lpcl_stereo                            \
        -lpcl_io_ply                            \
        -lpcl_apps                              \
        -lpcl_keypoints                         \
        -lpcl_ml                                \
        -lpcl_tracking                          \
        -lpcl_visualization                     \
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
        -lgdal                                  \
        -lboost_system

HEADERS += \
    core.h \
    match.h \
    delaunay.h \
    utils.h \
    mls_overlap.hpp

SOURCES += \
    main.cpp \
    utils.cpp \
    core.cpp \
    match.cpp \
    delaunay.cpp

QMAKE_CXXFLAGS += -fopenmp
#QMAKE_MAC_SDK = macosx10.11

LIBS += -fopenmp

