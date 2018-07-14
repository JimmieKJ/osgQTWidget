#ifndef QOSGCOMMON_H
#define QOSGCOMMON_H

#include <QObject>
#include <osg/Matrix>
class QOSGCommon : public QObject
{
    Q_OBJECT
public:
    QOSGCommon(QObject *parent = NULL);
    void _PrintMatrix(const osg::Matrix& m);
signals:

public slots:

};

#endif // QOSGCOMMON_H
