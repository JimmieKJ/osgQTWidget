#ifndef QOSGCOMPUTETRIMESHVISITOR_H
#define QOSGCOMPUTETRIMESHVISITOR_H

#include <QObject>
#include <osg/NodeVisitor>
class QOSGComputeTrimeshVisitor : public QObject ,public osg::NodeVisitor
{
    Q_OBJECT
public:
    QOSGComputeTrimeshVisitor(osg::NodeVisitor::TraversalMode traversalMode = osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ,QObject *parent = NULL);


    virtual void reset();

    osg::Vec3Array* getTriMesh()
    {
        return( mesh.get() );
    }

    void apply(osg::Geometry &geode );
protected:
    void applyDrawable( osg::Drawable * drawable );

signals:

public slots:

protected:

    osg::ref_ptr< osg::Vec3Array > mesh;
};

#endif // QOSGCOMPUTETRIMESHVISITOR_H
