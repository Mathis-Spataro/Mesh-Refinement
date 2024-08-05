#include "src/mainwindow.h"
#include "src/geometricworld.h"
#include "src/glcontroller.h"
#include <QApplication>
#include <iostream>
#include <fstream>

using namespace std;

extern const string projectPath = "/home/user/Documents/M2S1/gam/repo/gam-roullier-spataro/";

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow window;
    GeometricWorld& model = GeometricWorld::getInstance();
    GLController controller(model, &window);
    window.show();
    return a.exec();
}

