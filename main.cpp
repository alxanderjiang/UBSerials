#include "UBSerials.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    UBSerials w;
    w.show();
    return a.exec();
}
