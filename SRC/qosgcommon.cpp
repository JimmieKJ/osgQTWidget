#include "qosgcommon.h"
#include <QDebug>
QOSGCommon::QOSGCommon(QObject *parent) : QObject(parent)
{

}
void QOSGCommon::_PrintMatrix(const osg::Matrix& m)
{
    for (size_t i = 0; i < 4; i++) {
        qDebug() << QString("Line '%1'= %2 %3 %4 %5\n").arg(i).arg(m(i,0)).arg(m(i,1)).arg(m(i,2)).arg(m(i,3));
    }
}
