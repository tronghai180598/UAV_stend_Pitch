// Точка входа наземной станции Qt_Flix.
// Запускает полноэкранное окно управления стендом pitch (см. dialog.cpp).

#include "dialog.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    // Разрешить развёртывание окна на весь экран (удобно для лабораторного ПК).
    w.setWindowFlags(Qt::Window | Qt::WindowMaximizeButtonHint | Qt::WindowMinimizeButtonHint | Qt::WindowCloseButtonHint);
    w.showMaximized();
    return a.exec();
}
