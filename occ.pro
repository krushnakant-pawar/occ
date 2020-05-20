QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib -lopencv_optflow -lopencv_tracking -lopencv_text -lopencv_xfeatures2d -lopencv_shape \
                         -lopencv_video -lopencv_ximgproc -lopencv_xobjdetect -lopencv_objdetect -lopencv_features2d \
                         -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core
SOURCES += \
    main.cpp \
    mainwindow.cpp \
    occ.cpp \
    welcomescreen.cpp \
    worker.cpp

HEADERS += \
    mainwindow.h \
    messages.h \
    myocc.h \
    occ.h \
    welcomescreen.h \
    worker.h

FORMS += \
    mainwindow.ui \
    welcomescreen.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    resources.qrc