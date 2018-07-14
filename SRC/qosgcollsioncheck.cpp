#include "qosgcollsioncheck.h"
#include <iostream>
QOSGCollsionCheck::QOSGCollsionCheck(QOSGEnvModel* envModel,QObject *parent) :
    QObject(parent),
    m_envModel(envModel)
{
m_co = NULL;
}

btCollisionWorld *QOSGCollsionCheck::_initCollision()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btCollisionWorld* collisionWorld = new btCollisionWorld( dispatcher, inter, collisionConfiguration );
    return( collisionWorld );
}

void QOSGCollsionCheck::_addStaicObjectForCollsion(const std::string &staticObjId,btCollisionWorld* cw)
{
    btCollisionObject* btBoxObject = new btCollisionObject;
    osg::MatrixTransform * tempMat = m_envModel->_getModelByID(staticObjId);
//    btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG(tempMat);
//    if(!!tempTrimeshCollsion){
//         btBoxObject->setCollisionShape(tempTrimeshCollsion);
//    }else {
        btConvexHullShape *tempConvexTriangleMesh = btConvexHullCollisionShapeFromOSG(tempMat);
        if(!!tempConvexTriangleMesh){
            btBoxObject->setCollisionShape(tempConvexTriangleMesh);
        }else{
            btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG(tempMat);
            btBoxObject->setCollisionShape(tempBoxShape);
        }
//    }
    btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
//    btBoxObject->setWorldTransform( osgbCollision::asBtTransform( tempMat->getMatrix() ) );
    btBoxObject->setUserPointer((void*)tempMat);
    cw->addCollisionObject( btBoxObject );
}


void QOSGCollsionCheck::_addDynamicObjectForCollsion(const std::string &staticObjId,btCollisionWorld* cw)
{
    if(!!m_co){
        cw->removeCollisionObject(m_co);
        m_co = NULL;
    }
    btCollisionObject* btBoxObject = new btCollisionObject;
    osg::MatrixTransform * tempMat = m_envModel->_getModelByID(staticObjId);
    btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG(tempMat);
    if(!!tempTrimeshCollsion){
        btBoxObject->setCollisionShape(tempTrimeshCollsion);
    }else {
        btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG(tempMat);
        if(!!tempConvexTriangleMesh){
            btBoxObject->setCollisionShape(tempConvexTriangleMesh);
        }else{
            btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG(tempMat);
            btBoxObject->setCollisionShape(tempBoxShape);
        }
    }
    btBoxObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject->setWorldTransform( osgbCollision::asBtTransform( tempMat->getMatrix() ) );
    btBoxObject->setUserPointer((void*)tempMat);
    cw->addCollisionObject( btBoxObject );
    m_co = btBoxObject;
}

void QOSGCollsionCheck::_addDynamicObjectForCollsion(const osg::Matrix& mat, osg::MatrixTransform *staticObj, btCollisionWorld* cw)
{
    if(!!m_co){
        cw->removeCollisionObject(m_co);
        m_co = NULL;
    }
    btCollisionObject* btBoxObject = new btCollisionObject;
//
//    btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG(staticObj);
//    if(!!tempConvexTriangleMesh){
//         btBoxObject->setCollisionShape(tempConvexTriangleMesh);
    btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG(staticObj);
    if(!!tempTrimeshCollsion){
        btBoxObject->setCollisionShape(tempTrimeshCollsion);
    }else {
        btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG(staticObj);
        if(!!tempConvexTriangleMesh){
            btBoxObject->setCollisionShape(tempConvexTriangleMesh);
        }else{
            btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG(staticObj);
            btBoxObject->setCollisionShape(tempBoxShape);
        }
    }
    btBoxObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject->setWorldTransform( osgbCollision::asBtTransform( mat ) );
    btBoxObject->setUserPointer((void*)staticObj);
    cw->addCollisionObject( btBoxObject );
    m_co = btBoxObject;
}

void QOSGCollsionCheck::_deleteDynamicCollsion(btCollisionWorld *cw)
{
    int numb = cw->getNumCollisionObjects();
    for(int i=0;i<numb;++i){
        if(cw->getCollisionObjectArray().at(i)->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT)
            cw->removeCollisionObject(cw->getCollisionObjectArray().at(i));
    }
}

void QOSGCollsionCheck::_deleteStaticCollsion(btCollisionWorld* cw)
{
    int numb = cw->getNumCollisionObjects();
    for(int i=0;i<numb;++i){
        if(cw->getCollisionObjectArray().at(i)->getCollisionFlags() & btCollisionObject::CF_STATIC_OBJECT)
            cw->removeCollisionObject(cw->getCollisionObjectArray().at(i));
    }
}

void QOSGCollsionCheck::_addStaicObjectForCollsion(QOSGEnvModel::DrawModeType drawMode,btCollisionWorld* cw)
{
    switch(drawMode){
    case QOSGEnvModel::PointCloud:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_cloudGroups.begin();iter != m_envModel->m_cloudGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::BoxModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_BoxGroups.begin();iter != m_envModel->m_BoxGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::CyliderModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_CyliderGroups.begin();iter != m_envModel->m_CyliderGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::PlaneModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_PlaneGroups.begin();iter != m_envModel->m_PlaneGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::BoxEdgeModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_BoxEdgeGroups.begin();iter != m_envModel->m_BoxEdgeGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::FrameModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_FrameGroups.begin();iter != m_envModel->m_FrameGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::GridModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_GridFloorGroups.begin();iter != m_envModel->m_GridFloorGroups.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    case QOSGEnvModel::ModelFromDisk:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_envModel->m_ModelFromDiskGroup.begin();iter != m_envModel->m_ModelFromDiskGroup.end();++iter){
            btCollisionObject* btBoxObject = new btCollisionObject;
//            btBoxObject->setCollisionShape(btTriMeshCollisionShapeFromOSG((*iter).second.get()));
//            btTriangleMeshShape* tempTrimeshCollsion = btTriMeshCollisionShapeFromOSG((*iter).second.get());
//            if(!!tempTrimeshCollsion){
//                 btBoxObject->setCollisionShape(tempTrimeshCollsion);
//            }else {
                btConvexTriangleMeshShape *tempConvexTriangleMesh = btConvexTriMeshCollisionShapeFromOSG((*iter).second.get());
                if(!!tempConvexTriangleMesh){
                    btBoxObject->setCollisionShape(tempConvexTriangleMesh);
                }else{
                    btBoxShape* tempBoxShape = btBoxCollisionShapeFromOSG((*iter).second.get());
                    btBoxObject->setCollisionShape(tempBoxShape);
                }
//            }
            btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
            btBoxObject->setUserPointer((void*)((*iter).second.get()));
            cw->addCollisionObject( btBoxObject );
        }
        break;
    }
    default:
        break;
    }

}

void QOSGCollsionCheck::_excuteCollision( bool& lastColState, btCollisionWorld* cw )
{
    unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
    if( ( numManifolds <= 0 ) && (lastColState == true ) )
    {
        std::cout << "No collision." << std::endl;
        lastColState = false;
        m_collsionObj = NULL;
    }
    else if(numManifolds > 0){
        for( unsigned int i = 0; i < numManifolds; i++ )
        {
            btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
            const btCollisionObject* tempObj0 = contactManifold->getBody0();
            const btCollisionObject* tempObj1 = contactManifold->getBody1();
            osg::MatrixTransform* tempTrans0 = (osg::MatrixTransform*)(tempObj0->getUserPointer());
            osg::MatrixTransform* tempTrans1 = (osg::MatrixTransform*)(tempObj1->getUserPointer());

            unsigned int numContacts = contactManifold->getNumContacts();
//            for( unsigned int j=0; j<numContacts; j++ )
            if(numContacts > 0)
            {
                btManifoldPoint& pt = contactManifold->getContactPoint( 0 );
                if( ( pt.getDistance() <= 0.f ) && ( lastColState == false ) )
                {
                    if(!!tempTrans0 && !!tempTrans1){
                        if(m_collsionObj != tempObj0){
                            m_envModel->_setCollisionItem(osg::ref_ptr<osg::MatrixTransform>(tempTrans0));
                            m_collsionObj = const_cast<btCollisionObject*>(tempObj0);
                        }
                    }
//                    // grab these values for the contact normal arrows:
//                    osg::Vec3 pos = osgbCollision::asOsgVec3( pt.getPositionWorldOnA() );
//                    osg::Vec3 normal = osgbCollision::asOsgVec3( pt.m_normalWorldOnB );
//                    float pen = pt.getDistance(); //penetration depth

//                    osg::Quat q;
//                    q.makeRotate( osg::Vec3( 0, 0, 1 ), normal );

//                    std::cout << "Collision detected." << std::endl;

//                    std::cout << "\tPosition: " << pos.x() << std::endl;
//                    std::cout << "\tNormal: " << normal.x() << std::endl;
//                    std::cout << "\tPenetration depth: " << pen << std::endl;
                    lastColState = true;
                }
                else if( ( pt.getDistance() > 0.f ) && ( lastColState == true ) )
                {
//                    osg::notify( osg::ALWAYS ) << "No collision." << std::endl;
                    m_envModel->_setCollisionItem(NULL);
                    lastColState = false;
                    m_collsionObj = NULL;
                }
            }
        }
    }
    else if(numManifolds <= 0){
        m_envModel->_setCollisionItem(NULL);
        lastColState = false;
        m_collsionObj = NULL;
    }
}
