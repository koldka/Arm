#-------------------------------------------------
#
# Project created by QtCreator 2017-10-10T19:36:28
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Project
TEMPLATE = app

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

LIBS    += -L/lib64 -lgmp -lCGAL

SOURCES += main.cpp\
        mainwindow.cpp \
    filemanager.cpp \
    object.cpp \
    halfedge.cpp \
    vertex.cpp \
    face.cpp \
    displaywidget.cpp \
    geometryloader.cpp

HEADERS  += mainwindow.h \
    filemanager.h \
    object.h \
    halfedge.h \
    vertex.h \
    face.h \
    displaywidget.h \
    geometryloader.h \
    ImportOBJ.h

FORMS    += mainwindow.ui

RESOURCES += \
    shaders.qrc
