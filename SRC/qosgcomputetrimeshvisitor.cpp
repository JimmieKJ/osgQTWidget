#include "qosgcomputetrimeshvisitor.h"
#include <osg/Geode>
#include <osg/TriangleFunctor>
#include <osg/Geometry>
/* \cond */
struct ComputeTriMeshFunc
{
    ComputeTriMeshFunc()
    {
        vertices = new osg::Vec3Array;

        vertices->clear();
    }

    void inline operator()( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 v3, bool _temp )
    {
        vertices->push_back( v1 );
        vertices->push_back( v2 );
        vertices->push_back( v3 );
    }

    osg::ref_ptr< osg::Vec3Array > vertices;
};

QOSGComputeTrimeshVisitor::QOSGComputeTrimeshVisitor(osg::NodeVisitor::TraversalMode traversalMode, QObject *parent) :
    QObject(parent),
    osg::NodeVisitor( traversalMode )
{
    mesh = new osg::Vec3Array;
}

/* \endcond */

void QOSGComputeTrimeshVisitor::reset()
{
    mesh->clear();
}

void QOSGComputeTrimeshVisitor::apply( osg::Geometry & geode )
{
    unsigned int idx;
    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>( geode.getVertexArray() );
    for( idx = 0; idx < vertices->size(); idx++ ){
//        applyDrawable( geode.getDrawable( idx ) );
        osg::Matrix m = osg::computeLocalToWorld( getNodePath() );
        mesh->push_back( vertices->at(idx) * m );
    }
}

void QOSGComputeTrimeshVisitor::applyDrawable( osg::Drawable * drawable )
{
    osg::TriangleFunctor< ComputeTriMeshFunc > functor;
    drawable->accept( functor );

    osg::Matrix m = osg::computeLocalToWorld( getNodePath() );
    osg::Vec3Array::iterator iter;
    for( iter = functor.vertices->begin(); iter != functor.vertices->end(); ++iter )
    {
        mesh->push_back( *iter * m );
    }
}
