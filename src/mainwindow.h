#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "gldisplaywidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    GLDisplayWidget* getDisplayWidget();
    Ui::MainWindow* getUi();

signals:
    void keyPressed(QKeyEvent *event);

protected:
    void keyPressEvent(QKeyEvent *event);

private:
    Ui::MainWindow *ui;
    std::string _filepath;
    GLDisplayWidget* glDisplayWidget;
};

#endif // MAINWINDOW_H
