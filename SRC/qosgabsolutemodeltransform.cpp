#include "qosgabsolutemodeltransform.h"
#include <osgUtil/CullVisitor>
QOSGAbsoluteModelTransform::QOSGAbsoluteModelTransform(QObject *parent) : QObject(parent)
{
    setReferenceFrame( osg::Transform::ABSOLUTE_RF );
}

QOSGAbsoluteModelTransform::QOSGAbsoluteModelTransform( const osg::Matrix& m,QObject *parent ) : QObject(parent),
    _matrix( m )
{
    setReferenceFrame( osg::Transform::ABSOLUTE_RF );
}
QOSGAbsoluteModelTransform::QOSGAbsoluteModelTransform( const QOSGAbsoluteModelTransform& rhs, const osg::CopyOp& copyop,QObject *parent ) : QObject(parent),
    osg::Transform( rhs, copyop ),
    _matrix( rhs._matrix )
{
    setReferenceFrame( osg::Transform::ABSOLUTE_RF );
}
QOSGAbsoluteModelTransform::~QOSGAbsoluteModelTransform()
{
}


bool
QOSGAbsoluteModelTransform::computeLocalToWorldMatrix( osg::Matrix& matrix, osg::NodeVisitor* nv ) const
{
    if( getReferenceFrame() == osg::Transform::ABSOLUTE_RF )
    {
        osg::Matrix view;
        if( !nv )
            osg::notify( osg::INFO ) << "AbsoluteModelTransform: NULL NodeVisitor; can't get view." << std::endl;
        else if( nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR )
            osg::notify( osg::INFO ) << "AbsoluteModelTransform: NodeVisitor is not CullVisitor; can't get view." << std::endl;
        else
        {
            osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
#ifdef SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT
            // If OSG_STEREO=ON is in the environment, SceneView hides the view matrix
            // in a stack rather than placing it in a Camera node. Enable this code
            // (using CMake) to use a less-efficient way to compute the view matrix that
            // is compatible with SceneView's usage.
            osg::NodePath np = nv->getNodePath();
            np.pop_back();
            osg::Matrix l2w = osg::computeLocalToWorld( np );
            osg::Matrix invL2w = osg::Matrix::inverse( l2w );
            view = invL2w * *( cv->getModelViewMatrix() );
#else
            // Faster way to determine the view matrix, but not compatible with
            // SceneView anaglyphic stereo.
            osg::Camera* cam = cv->getCurrentCamera();
            cam->computeLocalToWorldMatrix( view, cv );
#endif
        }
        matrix = ( _matrix * view );
    }
    else
        // RELATIVE_RF
        matrix.preMult(_matrix);

    return( true );
}

bool
QOSGAbsoluteModelTransform::computeWorldToLocalMatrix( osg::Matrix& matrix, osg::NodeVisitor* nv ) const
{
    osg::Matrix inv( osg::Matrix::inverse( _matrix ) );
    if( getReferenceFrame() == osg::Transform::ABSOLUTE_RF )
    {
        osg::Matrix invView;
        if( !nv )
            osg::notify( osg::NOTICE ) << "AbsoluteModelTransform: NULL NodeVisitor; can't get invView." << std::endl;
        else if( nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR )
            osg::notify( osg::NOTICE ) << "AbsoluteModelTransform: NodeVisitor is not CullVisitor; can't get invView." << std::endl;
        else
        {
            osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
#ifdef SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT
            // If OSG_STEREO=ON is in the environment, SceneView hides the view matrix
            // in a stack rather than placing it in a Camera node. Enable this code
            // (using CMake) to use a less-efficient way to compute the view matrix that
            // is compatible with SceneView's usage.
            osg::NodePath np = nv->getNodePath();
            np.pop_back();
            osg::Matrix l2w = osg::computeLocalToWorld( np );
            invView = *( cv->getModelViewMatrix() ) * l2w;
#else
            osg::Camera* cam = cv->getCurrentCamera();
            cam->computeWorldToLocalMatrix( invView, cv );
#endif
        }
        matrix = ( invView * inv );
    }
    else
        // RELATIVE_RF
        matrix.postMult( inv );

    return( true );
}
