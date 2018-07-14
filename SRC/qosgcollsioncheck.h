#ifndef QOSGCOLLSIONCHECK_H
#define QOSGCOLLSIONCHECK_H

#include <QObject>
#include <btBulletCollisionCommon.h>
#include <osgbCollision/Version.h>
#include <osgbCollision/Utils.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgwTools/Shapes.h>
#include <osgwTools/Version.h>
#include <osg/ComputeBoundsVisitor>
#include <osgbCollision/ComputeTriMeshVisitor.h>
#include "qosgcollectverticesvisitor.h"
#include "qosgenvmodel.h"
#include "qosgcomputetrimeshvisitor.h"
class QOSGCollsionCheck : public QObject
{
    Q_OBJECT
public:
    QOSGCollsionCheck(QOSGEnvModel* envModel,QObject *parent = NULL);

    btCollisionWorld* _initCollision();
    void _excuteCollision( bool& lastColState, btCollisionWorld* cw );
    void _addStaicObjectForCollsion(const std::string &staticObjId,btCollisionWorld* cw);
    void _addStaicObjectForCollsion(QOSGEnvModel::DrawModeType drawMode,btCollisionWorld* cw);
    void _addDynamicObjectForCollsion(const std::string &staticObjId,btCollisionWorld* cw);
    void _addDynamicObjectForCollsion(const osg::Matrix &mat, osg::MatrixTransform* staticObj, btCollisionWorld* cw);
    void _deleteDynamicCollsion(btCollisionWorld* cw );
    void _deleteStaticCollsion(btCollisionWorld* cw);
signals:

public slots:

private:

    btBoxShape* btBoxCollisionShapeFromOSG( osg::Node* node, const osg::BoundingBox* bb=NULL  )
    {
        osg::BoundingBox bbox;
        if (bb)
            bbox = *bb;
        else
        {
            osg::ComputeBoundsVisitor visitor;
            node->accept( visitor );
            bbox = visitor.getBoundingBox();
        }

        btBoxShape* shape = new btBoxShape( btVector3( ( bbox.xMax() - bbox.xMin() ) * 0.5,
            ( bbox.yMax() - bbox.yMin() ) * 0.5, ( bbox.zMax() - bbox.zMin() ) * 0.5 ) );
        return( shape );
    }

    btTriangleMeshShape* btTriMeshCollisionShapeFromOSG( osg::Node* node )
    {
        QOSGComputeTrimeshVisitor visitor;
        node->accept( visitor );

        osg::Vec3Array* vertices = visitor.getTriMesh();
        if( vertices->size() < 3 )
        {
            osg::notify( osg::WARN ) << "osgbCollision::btTriMeshCollisionShapeFromOSG, no triangles found" << std::endl;
            return( NULL );
        }

        btTriangleMesh* mesh = new btTriangleMesh;
        for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
        {
            osg::Vec3& p1 = ( *vertices )[ i ];
            osg::Vec3& p2 = ( *vertices )[ i + 1 ];
            osg::Vec3& p3 = ( *vertices )[ i + 2 ];
            mesh->addTriangle( osgbCollision::asBtVector3( p1 ),
                osgbCollision::asBtVector3( p2 ), osgbCollision::asBtVector3( p3 ) );
        }

        btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape( mesh, true );
        return( meshShape );
    }

    btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromOSG( osg::Node* node )
    {
        QOSGComputeTrimeshVisitor visitor;
        node->accept( visitor );



        osg::Vec3Array* vertices = visitor.getTriMesh();

        btTriangleMesh* mesh = new btTriangleMesh;
        osg::Vec3 p1, p2, p3;
        for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
        {
            p1 = vertices->at( i );
            p2 = vertices->at( i + 1 );
            p3 = vertices->at( i + 2 );
            mesh->addTriangle( osgbCollision::asBtVector3( p1 ),
                osgbCollision::asBtVector3( p2 ), osgbCollision::asBtVector3( p3 ) );
        }

        btConvexTriangleMeshShape* meshShape = new btConvexTriangleMeshShape( mesh );
        return( meshShape );
    }

    btConvexHullShape* btConvexHullCollisionShapeFromOSG( osg::Node* node )
    {
        QOSGCollectVerticesVisitor cvv;
        node->accept( cvv );
        osg::Vec3Array* v = cvv.getVertices();
        osg::notify( osg::INFO ) << "CollectVerticesVisitor: " << v->size() << std::endl;

        // Convert verts to array of Bullet scalars.
        btScalar* btverts = new btScalar[ v->size() * 3 ];
        if( btverts == NULL )
        {
            osg::notify( osg::FATAL ) << "NULL btverts" << std::endl;
            return( NULL );
        }
        btScalar* btvp = btverts;

        osg::Vec3Array::const_iterator itr;
        for( itr = v->begin(); itr != v->end(); ++itr )
        {
            const osg::Vec3& s( *itr );
            *btvp++ = (btScalar)( s[ 0 ] );
            *btvp++ = (btScalar)( s[ 1 ] );
            *btvp++ = (btScalar)( s[ 2 ] );
        }
        btConvexHullShape* chs = new btConvexHullShape( btverts,
            (int)( v->size() ), (int)( sizeof( btScalar ) * 3 ) );
        delete[] btverts;

        return( chs );
    }
public:
//    btCollisionWorld* m_collisionWorld ;
    QOSGEnvModel* m_envModel;
    btCollisionObject* m_co;
    btCollisionObject* m_collsionObj;
};

#endif // QOSGCOLLSIONCHECK_H
