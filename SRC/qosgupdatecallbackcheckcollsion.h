#ifndef QOSGUPDATECALLBACKCHECKCOLLSION_H
#define QOSGUPDATECALLBACKCHECKCOLLSION_H

#include <QObject>
#include <osg/NodeCallback>
#include <qosgcollsioncheck.h>
#include <boost/function.hpp>

class QOSGUpdateCallbackCheckCollsion : public QObject,public osg::NodeCallback
{
    Q_OBJECT
public:
    typedef boost::function<void (int , int, double)> _SetViewport;
    QOSGUpdateCallbackCheckCollsion(int widWidget,int widHeight,double metersinunit ,btCollisionWorld* collsionWorld,QOSGCollsionCheck* collsionCheck,const _SetViewport& setViewport=_SetViewport(),QObject *parent = NULL);
    virtual void operator ()(osg::Node* node,osg::NodeVisitor* nv);
signals:

public slots:

public:
    int m_widWidget;
    int m_widHeight;

private:
    btCollisionWorld* m_collsionWorld;
    QOSGCollsionCheck* m_collsionCheck;
    _SetViewport m_setViewport;
    double m_metersinunit;
};

#endif // QOSGUPDATECALLBACKCHECKCOLLSION_H
