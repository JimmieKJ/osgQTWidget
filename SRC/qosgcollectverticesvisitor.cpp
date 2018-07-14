#include "qosgcollectverticesvisitor.h"
#include <osg/Geode>
#include <osg/Geometry>
#include <osgwTools/AbsoluteModelTransform.h>
#include "qosgabsolutemodeltransform.h"
QOSGCollectVerticesVisitor::QOSGCollectVerticesVisitor(TraversalMode traversalMode, QObject *parent) :
    QObject(parent),
    osg::NodeVisitor(traversalMode)
{
    verts_ = new osg::Vec3Array;
    reset();
}
void QOSGCollectVerticesVisitor::reset()
{
    verts_->clear();
}

void QOSGCollectVerticesVisitor::apply( osg::Transform& node )
{
    // Override apply(Transform&) to avoid processing AMT nodes.
    const bool nonAMT = ( dynamic_cast< QOSGAbsoluteModelTransform* >( &node ) == NULL );
    if( nonAMT )
        _localNodePath.push_back( &node );

    traverse( node );

    if( nonAMT )
        _localNodePath.pop_back();
}

void QOSGCollectVerticesVisitor::apply( osg::Geode& geode )
{
    unsigned int idx;
    for( idx = 0; idx < geode.getNumDrawables(); idx++ )
        applyDrawable( geode.getDrawable( idx ) );
}

void QOSGCollectVerticesVisitor::applyDrawable( osg::Drawable* drawable )
{
    osg::Geometry* geom = drawable->asGeometry();
    if( geom == NULL )
        return;

    const osg::Vec3Array* in = dynamic_cast< const osg::Vec3Array* >( geom->getVertexArray() );
    if( in == NULL )
    {
        osg::notify( osg::WARN ) << "QOSGCollectVerticesVisitor: Non-Vec3Array vertex array encountered." << std::endl;
        return;
    }

    const osg::Matrix m = osg::computeLocalToWorld( _localNodePath );

    unsigned int idx;
    for( idx=0; idx < geom->getNumPrimitiveSets(); idx++ )
    {
        osg::PrimitiveSet* ps = geom->getPrimitiveSet( idx );
        unsigned int jdx;
        for( jdx=0; jdx < ps->getNumIndices(); jdx++ )
        {
            unsigned int index = ps->index( jdx );
            if(index < in->size())
                verts_->push_back( (*in)[ index ] * m );
        }
    }

    /*
    osg::Vec3Array::const_iterator iter;
    for( iter = in->begin(); iter != in->end(); iter++ )
    {
        verts_->push_back( *iter * m );
    }
    */
}
