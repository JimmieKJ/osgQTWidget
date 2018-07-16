#include "qosgcollsioncheck.h"
#include <iostream>
QOSGCollsionCheck::QOSGCollsionCheck(osg::ref_ptr<osgViewer::View> view, QOSGEnvModel* envModel, QObject *parent) :
    QObject(parent),
    m_envModel(envModel),
    m_view(view)
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

//void QOSGCollsionCheck::_excuteCollision( bool& lastColState, btCollisionWorld* cw )
//{
//    unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
//    if( ( numManifolds <= 0 ) && (lastColState == true ) )
//    {
//        std::cout << "No collision." << std::endl;
//        lastColState = false;
//        m_collsionObj = NULL;
//    }
//    else if(numManifolds > 0){
//        for( unsigned int i = 0; i < numManifolds; i++ )
//        {
//            btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
//            const btCollisionObject* tempObj0 = contactManifold->getBody0();
//            const btCollisionObject* tempObj1 = contactManifold->getBody1();
//            osg::MatrixTransform* tempTrans0 = (osg::MatrixTransform*)(tempObj0->getUserPointer());
//            osg::MatrixTransform* tempTrans1 = (osg::MatrixTransform*)(tempObj1->getUserPointer());

//            unsigned int numContacts = contactManifold->getNumContacts();
////            for( unsigned int j=0; j<numContacts; j++ )
//            if(numContacts > 0)
//            {
//                btManifoldPoint& pt = contactManifold->getContactPoint( 0 );
//                if( ( pt.getDistance() <= 0.f ) && ( lastColState == false ) )
//                {
//                    if(!!tempTrans0 && !!tempTrans1){
//                        if(m_collsionObj != tempObj0){
//                            m_envModel->_setCollisionItem(osg::ref_ptr<osg::MatrixTransform>(tempTrans0));
//                            m_collsionObj = const_cast<btCollisionObject*>(tempObj0);
//                        }
//                    }
////                    // grab these values for the contact normal arrows:
////                    osg::Vec3 pos = osgbCollision::asOsgVec3( pt.getPositionWorldOnA() );
////                    osg::Vec3 normal = osgbCollision::asOsgVec3( pt.m_normalWorldOnB );
////                    float pen = pt.getDistance(); //penetration depth

////                    osg::Quat q;
////                    q.makeRotate( osg::Vec3( 0, 0, 1 ), normal );

////                    std::cout << "Collision detected." << std::endl;

////                    std::cout << "\tPosition: " << pos.x() << std::endl;
////                    std::cout << "\tNormal: " << normal.x() << std::endl;
////                    std::cout << "\tPenetration depth: " << pen << std::endl;
//                    lastColState = true;
//                }
//                else if( ( pt.getDistance() > 0.f ) && ( lastColState == true ) )
//                {
////                    osg::notify( osg::ALWAYS ) << "No collision." << std::endl;
//                    m_envModel->_setCollisionItem(NULL);
//                    lastColState = false;
//                    m_collsionObj = NULL;
//                }
//            }
//        }
//    }
//    else if(numManifolds <= 0){
//        m_envModel->_setCollisionItem(NULL);
//        lastColState = false;
//        m_collsionObj = NULL;
//    }
//}
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
                            osg::Vec3 pos = osgbCollision::asOsgVec3( pt.getPositionWorldOnA() );
                            osg::Vec3 normal = osgbCollision::asOsgVec3( pt.m_normalWorldOnB );
                            float pen = pt.getDistance(); //penetration depth
                            osg::Vec3 secondPoint = (-normal)*10+pos;
                            osg::Vec3 firstPoint = (-normal)*pen*15+pos;
                            osg::Vec4 color(1,1,0,1);
                            osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
                                new osgUtil::LineSegmentIntersector(secondPoint,firstPoint);
                            osgUtil::IntersectionVisitor iv( intersector.get() );
                            m_view->getCamera()->accept( iv );
                            if ( intersector->containsIntersections() ){
//                                osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
//                                osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
//                                osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//                                coords->push_back(secondPoint);
//                                coords->push_back(firstPoint);
//                                colors->push_back(color);
//                                geometry->setVertexArray(coords.get());
//                                geometry->setColorArray(colors.get());
//                                geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
//                                geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES , 0 , 2)) ; //
//                                osg::StateSet* state = geometry->getOrCreateStateSet();
//                                state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
//                                osg::ref_ptr<osg::Geode>  geoTemp  = new osg::Geode;
//                                geoTemp->addDrawable(geometry.get());
//                                tempTrans0->getParent(0)->addChild(geoTemp);
                                osgUtil::LineSegmentIntersector::Intersection result = *(intersector->getIntersections().begin());
                                if (result.nodePath.size() >=2 ) {
                                    for(int i_n=0;i_n<result.nodePath.size();++i_n){
                                        if(result.nodePath.at(i_n)->getName() == tempTrans0->getName()){
                                            lastColState = true;
                                            m_envModel->_setCollisionItem(osg::ref_ptr<osg::MatrixTransform>(tempTrans0));
                                            m_collsionObj = const_cast<btCollisionObject*>(tempObj0);
                                        }
                                    }
                                }
                            }
                        }
                    }
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
