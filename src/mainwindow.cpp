#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    glDisplayWidget = ui->widget;
}

MainWindow::~MainWindow()
{
    delete ui;
}

GLDisplayWidget* MainWindow::getDisplayWidget() {
    return glDisplayWidget;
}

Ui::MainWindow* MainWindow::getUi() {
    return ui;
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    emit keyPressed(event);
}


