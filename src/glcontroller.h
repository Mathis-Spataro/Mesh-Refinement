#ifndef GLCONTROLLER_H
#define GLCONTROLLER_H

#include <QObject>
#include "gldisplaywidget.h" // Include the view header
#include "geometricworld.h" // Include the model header
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <filesystem>

class GLController : public QObject {
    Q_OBJECT
public:
    explicit GLController(GeometricWorld& world, MainWindow* window, QObject* parent = nullptr);

public slots:
    // View interactions
    void handleMousePressEvent(QMouseEvent *event);
    void handleWheelEvent(QWheelEvent *event);
    void handleKeyPressEvent(QKeyEvent *event);
    void handleMouseMoveEvent(QMouseEvent *event);

    void handleViewSwitchPlain();
    void handleViewSwitchWireframe();

    // Model interactions
    void handleMeshSwitch2DCore();
    void handleMeshSwitch2DNaive();
    void handleMeshSwitch2DTerrain();

    void handleMeshSwitch3DCube();
    void handleMeshSwitch3DQueen();

    void handleVertexAdd();
    void handleDelaunayCheckbox();
    void handleRuppertPushbutton();

private:
    QPoint lastPosMouse;
    QDoubleSpinBox* spinboxX;
    QDoubleSpinBox* spinboxY;
    QDoubleSpinBox* spinboxZ;
    double currentZoom;
    GeometricWorld& model;
    GLDisplayWidget* view;
    bool delaunize;
    bool meshIs2D;
    bool terrain;
};

#endif // GLCONTROLLER_H


