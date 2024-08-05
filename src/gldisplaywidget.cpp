#include "gldisplaywidget.h"
#ifdef __APPLE__
    #include <glu.h>
#else
    #include <GL/glu.h>
#endif

#include "QDebug"

GLDisplayWidget::GLDisplayWidget(QWidget *parent)
    : QOpenGLWidget(parent), _meshInterface(nullptr){}


// Camera management
void GLDisplayWidget::cameraTranslation(float moveX, float moveY) {
    _X += moveX; _Y += moveY;
    update();
}

void GLDisplayWidget::cameraZoom(float zoomAmount) {
    _Z += zoomAmount;
    update();
}

void GLDisplayWidget::cameraRotation(float rotaX, float rotaY){
    _angleX += rotaX; _angleY += rotaY;
    update();
}


// widget management
void GLDisplayWidget::initializeGL() {
    // background color
    glClearColor(0.2, 0.2, 0.2, 1);

    // Shader
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
}

void GLDisplayWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Center the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); //gluLookAt(eye, center, up)  //deprecated
                                          // Returns a 4x4 matrix that transforms world coordinates to eye coordinates.
    // Translation
    glTranslated(_X, _Y, _Z);

    // Rotation
    glRotatef(_angleY, 1.0f, 0.0f, 0.0f);
    glRotatef(_angleX, 0.0f, 1.0f, 0.0f);


    // Color for your _geomWorld
    glColor3f(0.9, 0.9, 0.9);

    // example
    drawCoordinateSystem();
    drawMesh();
}

void GLDisplayWidget::resizeGL(int width, int height) {

    glViewport(0, 0, width, height); // Viewport in the world window
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)width/(GLfloat)height, 0.1f, 100000.0f);

    update();
}


// GL Drawing
void glPointDraw(const Point & p) {
    glVertex3f(p.x, p.y, p.z);
}

void GLDisplayWidget::drawCoordinateSystem() const {
    glColor3d(0,1,0);
    glBegin(GL_LINE_STRIP);
    glPointDraw(Point(0.0, 0.0, 0.0));
    glPointDraw(Point(1.0, 0.0, 0.0));
    glEnd();
    glColor3d(0,0,1);
    glBegin(GL_LINE_STRIP);
    glPointDraw(Point(0.0, 0.0, 0.0));
    glPointDraw(Point(0.0, 1.0, 0.0));
    glEnd();
    glColor3d(1,0,0);
    glBegin(GL_LINE_STRIP);
    glPointDraw(Point(0.0, 0.0, 0.0));
    glPointDraw(Point(0.0, 0.0, 1.0));
    glEnd();
}

void GLDisplayWidget::drawMesh() const {
        glColor3f(0.9, 0.9, 0.9);
        for(unsigned i=0; i < _meshInterface->getNumberOfFaces(); i++){
            drawTriangle(_meshInterface->getFaceVertices(i), this->wireframe);
        }
}

void GLDisplayWidget::drawTriangle(std::array<Vertex, 3> vertices, bool wireframe) const {
    if(wireframe) glBegin(GL_LINE_LOOP);
    else glBegin(GL_TRIANGLES);
    glPointDraw(vertices[0].position());
    glPointDraw(vertices[1].position());
    glPointDraw(vertices[2].position());
    if(wireframe) glPointDraw(vertices[0].position());
    glEnd();
}

void GLDisplayWidget::drawTriangleColors(std::array<Vertex, 3> vertices, std::array<Vector, 3> colors, bool wireframe) const {
    if(wireframe) glBegin(GL_LINE_LOOP);
    else glBegin(GL_TRIANGLES);
    glColor3d(colors[0].x, colors[0].y, colors[0].z);
    glPointDraw(vertices[0].position());
    glColor3d(colors[1].x, colors[1].y, colors[1].z);
    glPointDraw(vertices[1].position());
    glColor3d(colors[2].x, colors[2].y, colors[2].z);
    glPointDraw(vertices[2].position());
    if(wireframe) {
        glColor3d(colors[0].x, colors[0].y, colors[0].z);
        glPointDraw(vertices[0].position());
    }
    glEnd();
}

// Utilities
void GLDisplayWidget::setMeshInterface(MeshConstInterface* w) {
    _meshInterface = w;
}

void GLDisplayWidget::setMeshWireframe(bool wf){
    wireframe = wf;
    update();
}

void GLDisplayWidget::toggleColors(){
    colors = colors ^ 1;
}


// --- protected

void GLDisplayWidget::mousePressEvent(QMouseEvent *event) {
    emit mousePressed(event);
}

void GLDisplayWidget::mouseMoveEvent(QMouseEvent *event) {
    emit mouseMoved(event);
}

void GLDisplayWidget::wheelEvent(QWheelEvent *event) {
    emit wheelScrolled(event);
}


