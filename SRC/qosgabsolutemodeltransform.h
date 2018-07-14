#ifndef QOSGABSOLUTEMODELTRANSFORM_H
#define QOSGABSOLUTEMODELTRANSFORM_H

#include <QObject>
#include <osg/Transform>
#include <osg/Matrix>

class QOSGAbsoluteModelTransform : public QObject, public osg::Transform
{
    Q_OBJECT

public:
    QOSGAbsoluteModelTransform(QObject *parent = NULL);
    QOSGAbsoluteModelTransform( const osg::Matrix& m ,QObject *parent = NULL);
    QOSGAbsoluteModelTransform( const QOSGAbsoluteModelTransform& rhs, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY,QObject *parent = NULL );

//    META_Node( osgwTools, AbsoluteModelTransform );

    virtual bool computeLocalToWorldMatrix( osg::Matrix& matrix, osg::NodeVisitor* nv ) const;
    virtual bool computeWorldToLocalMatrix( osg::Matrix& matrix, osg::NodeVisitor* nv ) const;

    inline void setMatrix( const osg::Matrix& m ) { _matrix = m; dirtyBound(); }
    inline const osg::Matrix& getMatrix() const { return _matrix; }


protected:
    virtual ~QOSGAbsoluteModelTransform();

    osg::Matrix _matrix;
signals:

public slots:
};

#endif // QOSGABSOLUTEMODELTRANSFORM_H
