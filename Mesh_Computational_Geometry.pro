#-------------------------------------------------
#
# Project created by QtCreator 2018-08-28T10:55:17
#
#-------------------------------------------------

QT += core gui opengl openglwidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Mesh_Computational_Geometry
TEMPLATE = app

SOURCES += main.cpp \
    src/geometricworld.cpp \
    src/glcontroller.cpp \
    src/mainwindow.cpp \
    src/gldisplaywidget.cpp \
    src/mesh.cpp \
    src/meshcirculators.cpp \
    src/meshio.cpp

HEADERS  += src/mainwindow.h \
    src/geometricworld.h \
    src/glcontroller.h \
    src/gldisplaywidget.h \
    src/mainwindow.h \
    src/geometry.h \
    src/geometryComponents/point.h \
    src/geometryComponents/triangle.h \
    src/geometryComponents/vector.h \
    src/geometryComponents/vertex.h \
    src/mesh.h \
    src/meshinterface.h

FORMS    += mainwindow.ui

#---- Comment the following line on MacOS
#---- Uncomment it on Windows and Linux
LIBS = -lGLU

#---- Uncomment the following line on Windows
#---- Comment it on Linux and MacOS
#LIBS += -lglu32
#LIBS += -lOpengl32

