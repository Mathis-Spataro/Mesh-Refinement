#include "glcontroller.h"

extern const string projectPath;

GLController::GLController(GeometricWorld& world, MainWindow* window, QObject* parent)
    : QObject(parent), model(world), view(window->getDisplayWidget()), delaunize(false), meshIs2D(true) {
    view->setMeshInterface(world.getMeshConstInterface());

    currentZoom = -5;

    // Connect GLDisplayWidget signals to GLController slots
    connect(view, &GLDisplayWidget::mousePressed, this, &GLController::handleMousePressEvent);
    connect(view, &GLDisplayWidget::wheelScrolled, this, &GLController::handleWheelEvent);
    connect(view, &GLDisplayWidget::mouseMoved, this, &GLController::handleMouseMoveEvent);
    connect(window, &MainWindow::keyPressed, this, &GLController::handleKeyPressEvent);

    // Connect UI signals to GLController slots
    // View interactions
    connect(window->getUi()->radio_view_plain, &QRadioButton::clicked, this, &GLController::handleViewSwitchPlain);
    connect(window->getUi()->radio_view_wireframe, &QRadioButton::clicked, this, &GLController::handleViewSwitchWireframe);

    // Model interactions
    connect(window->getUi()->radio_2D_singleTriangle, &QRadioButton::clicked, this, &GLController::handleMeshSwitch2DCore);
    connect(window->getUi()->radio_2D_naive, &QRadioButton::clicked, this, &GLController::handleMeshSwitch2DNaive);
    connect(window->getUi()->radio_2D_terrain, &QRadioButton::clicked, this, &GLController::handleMeshSwitch2DTerrain);

    connect(window->getUi()->radio_3D_cube, &QRadioButton::clicked, this, &GLController::handleMeshSwitch3DCube);
    connect(window->getUi()->radio_3D_queen, &QRadioButton::clicked, this, &GLController::handleMeshSwitch3DQueen);

    connect(window->getUi()->button_addVertex, &QPushButton::clicked, this, &GLController::handleVertexAdd);
    spinboxX = window->getUi()->spinbox_X;
    spinboxX->setRange(-10000, 10000);
    spinboxY = window->getUi()->spinbox_Y;
    spinboxY->setRange(-10000, 10000);
    spinboxZ = window->getUi()->spinbox_Z;
    spinboxZ->setRange(-10000, 10000);
    connect(window->getUi()->checkBox_delaunize, &QCheckBox::clicked, this, &GLController::handleDelaunayCheckbox);
    connect(window->getUi()->pushbutton_ruppert, &QPushButton::clicked, this, &GLController::handleRuppertPushbutton);

    // Set UI default state
    window->getUi()->radio_view_plain->setChecked(true);
    window->getUi()->radio_2D_singleTriangle->setChecked(true);
}


// View interactions
void GLController::handleMousePressEvent(QMouseEvent *event) {
    lastPosMouse = event->pos();
}

void GLController::handleMouseMoveEvent(QMouseEvent *event)
{
    if( event != NULL )
    {
        int dx = event->x() - lastPosMouse.x();
        int dy = event->y() - lastPosMouse.y();
        view->cameraRotation(dx, dy);
        lastPosMouse = event->pos();
    }
}

void GLController::handleWheelEvent(QWheelEvent *event) {
    if (event != nullptr) {
        QPoint numDegrees = event->angleDelta();
        if (!numDegrees.isNull()) {
            if (numDegrees.x() > 0 || numDegrees.y() > 0){
                view->cameraZoom(-0.1 * currentZoom);
                currentZoom -= 0.1 * currentZoom;
            } else {
                view->cameraZoom(0.1* currentZoom);
                currentZoom += 0.1 * currentZoom;
            }
          }
    }
}

void GLController::handleKeyPressEvent(QKeyEvent *event) {
    if (event != nullptr) {
        switch (event->key()) {
            case Qt::Key_D:
                // Move camera left
                view->cameraTranslation(0.05 * currentZoom, 0.0);
                break;
            case Qt::Key_Q:
                // Move camera right
                view->cameraTranslation(-0.05 * currentZoom, 0.0);
                break;
            case Qt::Key_Z:
                // Move camera down
                view->cameraTranslation(0.0, 0.05 * currentZoom);
                break;
            case Qt::Key_S:
                // Move camera up
                view->cameraTranslation(0.0, -0.05 * currentZoom);
                break;
            default:
                break;
        }
    }
}


void GLController::handleViewSwitchPlain() {
    cout<<"view switch plain triangle"<<endl;
    view->setMeshWireframe(false);
}

void GLController::handleViewSwitchWireframe() {
    cout<<"view switch wireframe"<<endl;
    view->setMeshWireframe(true);
}


// Model interactions.
void GLController::handleMeshSwitch2DCore() {
    cout<<"load mesh 2D core"<<endl;
    meshIs2D = true; terrain = false;
    model.loadMesh(projectPath + "data/core2DMesh.off");
    view->update();
}

void GLController::handleMeshSwitch2DNaive() {
    cout<<"load mesh 2D naive"<<endl;
    meshIs2D = true; terrain = false;
    model.loadMesh(projectPath + "data/naive2DMesh.off");
    if (delaunize) model.enforceDelaunay();
    view->update();
}

void GLController::handleMeshSwitch2DTerrain() {
    cout<<"load mesh 2D terrain"<<endl;
    meshIs2D = true; terrain = true;
    model.loadMesh(projectPath + "data/alpes_poisson.txt");
    if (delaunize) model.enforceDelaunay();
    view->update();
}


void GLController::handleMeshSwitch3DCube() {
    cout<<"load mesh 3D cube"<<endl;
    meshIs2D = false;
    model.loadMesh(projectPath + "data/cube.off");
    view->update();
}

void GLController::handleMeshSwitch3DQueen() {
    cout<<"load mesh 3D queen"<<endl;
    meshIs2D = false;
    model.loadMesh(projectPath + "data/queen.off");
    view->update();
}


void GLController::handleVertexAdd() {
    if (meshIs2D) {
        cout<<"add vertex"<<endl;
        model.meshAddVertex(Point(spinboxX->value(), spinboxY->value(), spinboxZ->value()), delaunize);
        view->update();
    }
}

void GLController::handleDelaunayCheckbox(){
    if (delaunize) { // toggle down
        cout<<"delaunay checkbox down"<<endl;
        delaunize = false;
    } else { // toggle up
        cout<<"delaunay checkbox up"<<endl;
        if (meshIs2D && !model.meshIsDelaunay()) {
            cout<<"delaunize"<<endl;
            model.enforceDelaunay();
        }
        delaunize = true;
    }
    view->update();
}

void GLController::handleRuppertPushbutton(){
    if (meshIs2D){
        if (!model.meshIsDelaunay()) {
            model.enforceDelaunay();
        }
        if (!terrain) model.enforceRuppert();
    }
    view->update();
}











