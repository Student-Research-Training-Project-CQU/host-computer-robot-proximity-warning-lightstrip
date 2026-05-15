#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle("Fusion");  // 使用 Fusion 风格，在各平台上显示更一致

    // 可选：深色调色板
    // QPalette dark; dark.setColor(QPalette::Window, QColor(30,30,40));
    // a.setPalette(dark);

    MainWindow w;
    w.show();
    return a.exec();
}
