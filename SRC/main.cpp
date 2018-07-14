#include "qosgmainwindow.h"
int main( int argc, char** argv )
{
    QApplication app(argc, argv);
    QOSGMainWindow w;
    w.show();
    return app.exec();
}
