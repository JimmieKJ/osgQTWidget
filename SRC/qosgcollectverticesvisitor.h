#ifndef QOSGCOLLECTVERTICESVISITOR_H
#define QOSGCOLLECTVERTICESVISITOR_H

#include <QObject>
#include <osg/NodeVisitor>
class QOSGCollectVerticesVisitor : public QObject,public osg::NodeVisitor
{
    Q_OBJECT
public:
    QOSGCollectVerticesVisitor(osg::NodeVisitor::TraversalMode traversalMode = osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ,QObject *parent = NULL);
    virtual void reset();

    osg::Vec3Array* getVertices()
    {
        return( verts_.get() );
    }

    void apply( osg::Geode& geode );

    /** \brief Builds CollectVerticesVisitor::_localNodePath (a NodePath) from all Transforms,
    excluding AbsoluteModelTransform.

    This visitor saves the transformed (world space) vertices from the scene graph.
    However, in order to be compatible with the
    \link rigidbody rigid body creation utilities, \endlink the visitor can't consider
    AbsoluteModelTransforms in such a transformation, as they ignore all parent transforms.

    To support this, we override NodeVisitor::apply(osg::Transform&) to build our own
    NodePath (CollectVerticesVisitor::_localNodePath) that contains all Transform nodes encountered during traversal
    except AbsoluteModelTransform nodes. */
    void apply( osg::Transform& node );

protected:
    void applyDrawable( osg::Drawable* drawable );

    osg::ref_ptr< osg::Vec3Array > verts_;

    /** NodePath containing only Transform nodes, but excluding AbsoluteModelTransform. */
    osg::NodePath _localNodePath;
signals:

public slots:
};

#endif // QOSGCOLLECTVERTICESVISITOR_H
