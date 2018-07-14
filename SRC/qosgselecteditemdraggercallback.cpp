#include "qosgselecteditemdraggercallback.h"
#include "qosgcommon.h"
QOsgSelectedItemDraggerCallback::QOsgSelectedItemDraggerCallback( btCollisionWorld *collisionWorld, osg::MatrixTransform* selectedItem, osgManipulator::Dragger *sourcetransform, osgManipulator::Dragger *updatetransform, QObject *parent)
    : QObject(parent),
     m_collisionWorld(collisionWorld),
     m_selectedItem(selectedItem),
     m_sourcetransform(sourcetransform),
     m_updatetransform(updatetransform)
{

}

bool QOsgSelectedItemDraggerCallback::receive(const osgManipulator::MotionCommand &command)
{
    if( !!m_sourcetransform && !!m_updatetransform ) {
        if(command.getStage() == osgManipulator::MotionCommand::START){

        }else if(command.getStage() == osgManipulator::MotionCommand::MOVE){
            if(m_sourcetransform->getDraggerActive()){
                m_updatetransform->setNodeMask(0);
            }
            else if(m_updatetransform->getDraggerActive()){
                m_sourcetransform->setNodeMask(0);
            }
//            QOSGCommon *common = new QOSGCommon();
//            common->_PrintMatrix(m_selectedItem->getMatrix());
            m_co->setWorldTransform( osgbCollision::asBtTransform( m_selectedItem->getMatrix()) );
//            common->_PrintMatrix(osgbCollision::asOsgMatrix( m_co->getWorldTransform()));
        }else if(command.getStage() == osgManipulator::MotionCommand::FINISH){
            m_updatetransform->setMatrix(m_sourcetransform->getMatrix());
            m_updatetransform->setNodeMask(1);
            m_sourcetransform->setNodeMask(1);
        }
    }
    return false;
}
