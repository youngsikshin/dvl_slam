#include <dedvo_gui/MainWindow.h>
#include <QApplication>
#include <QStyleFactory>
#include <QDebug>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w(argc, argv);
    w.show();

    return a.exec();
}
