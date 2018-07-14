#include "qosgupdatecallbackcheckcollsion.h"

QOSGUpdateCallbackCheckCollsion::QOSGUpdateCallbackCheckCollsion(int widWidget, int widHeight, double metersinunit, btCollisionWorld *collsionWorld, QOSGCollsionCheck *collsionCheck, const _SetViewport &setViewport, QObject *parent) :
    m_widWidget(widWidget),
    m_widHeight(widHeight),
    m_metersinunit(metersinunit),
    m_collsionWorld(collsionWorld),
    m_collsionCheck(collsionCheck),
    m_setViewport(setViewport),
    QObject(parent)
{

}

void QOSGUpdateCallbackCheckCollsion::operator ()(osg::Node *node, osg::NodeVisitor *nv)
{
    m_setViewport(m_widWidget,m_widHeight,m_metersinunit);
    bool lastColState = false;
    m_collsionWorld->performDiscreteCollisionDetection();
    m_collsionCheck->_excuteCollision(lastColState,m_collsionWorld);
    traverse( node, nv );
}
