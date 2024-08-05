#ifndef GLDISPLAYWIDGET_H
#define GLDISPLAYWIDGET_H

#include <QOpenGLWidget> // Module QtOpengL (classes QGLxxx in Qt4),
                         // with widget and rendering classes descending from QGLWidget,
                         // deprecated for module QtGui (classes QOpenGLxxx )
                         // and module widgets (QWidget, QGraphicsView).

#include <QtWidgets>
#include <QTimer>
#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

#include "meshinterface.h"
//class GLController; // Forward declaration of GLController

class GLDisplayWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit GLDisplayWidget(QWidget *parent = nullptr);

    // Camera management
    void cameraTranslation(float moveX, float moveY);
    void cameraZoom(float zoomAmount);
    void cameraRotation(float rotaX, float rotaY);

    // widget management
    void initializeGL() override; // The scene may be initialized in this function since the GeometricWorld is a data member...
    void paintGL() override; // Display the scene Gl
    void resizeGL(int width, int height) override;

    // GL drawing
    void drawCoordinateSystem() const;
    void drawMesh() const;
    void drawTriangle(std::array<Vertex, 3>, bool) const;
    void drawTriangleColors(std::array<Vertex, 3>,  std::array<Vector, 3>, bool) const;

    // Utilities
    void setMeshInterface(MeshConstInterface* w);
    void setMeshWireframe(bool wf);
    void toggleColors();

signals:
    void mousePressed(QMouseEvent *event);
    void mouseMoved(QMouseEvent *event);
    void wheelScrolled(QWheelEvent *event);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    bool wireframe = false;
    bool colors = false;
    float _X = 0.f, _Y = 0.f ,_Z = 0.f; // Translation
    float _angleX, _angleY; // Rotation
    QTimer _timer; // To update the scene
    MeshConstInterface* _meshInterface;
};

#endif // GLDISPLAYWIDGET_H
