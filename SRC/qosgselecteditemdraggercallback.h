#ifndef QOSGSELECTEDITEMDRAGGERCALLBACK_H
#define QOSGSELECTEDITEMDRAGGERCALLBACK_H
#include "qosgcollsioncheck.h"
#include <QObject>
#include <osgManipulator/Dragger>
#include <btBulletCollisionCommon.h>
#include <osgbCollision/Version.h>
#include <osgbCollision/Utils.h>
#include <osgbCollision/CollisionShapes.h>
class QOsgSelectedItemDraggerCallback : public QObject, public osgManipulator::DraggerCallback
{
    Q_OBJECT
public:
    QOsgSelectedItemDraggerCallback(btCollisionWorld* collisionWorld, osg::MatrixTransform* selectedItem, osgManipulator::Dragger* sourcetransform, osgManipulator::Dragger* updatetransform, QObject *parent = NULL);
    virtual bool receive(const osgManipulator::MotionCommand& command);
signals:

public slots:

public:
    btCollisionObject* m_co;
private:
    osg::observer_ptr<osgManipulator::Dragger> m_sourcetransform, m_updatetransform;
    osg::MatrixTransform* m_selectedItem;
    btCollisionWorld* m_collisionWorld;/* = initCollision();*/
    QOSGCollsionCheck * m_collsionCheck;

};
#endif // QOSGSELECTEDITEMDRAGGERCALLBACK_H
