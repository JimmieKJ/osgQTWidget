#ifndef QOSGOCTREEBUILDER_H
#define QOSGOCTREEBUILDER_H

#include <QObject>
#include <osg/Group>
#include <osg/LOD>
class QOSGOctreeBuilder : public QObject
{
    Q_OBJECT
public:
        typedef std::pair<osg::ref_ptr<osg::Group>, osg::BoundingBox> ElementInfo;

    QOSGOctreeBuilder(QObject *parent = NULL);
    osg::Group* _build(int depth,const osg::BoundingBox& total,std::vector<ElementInfo>& elements);
    osg::LOD* _createNewLevel( int level, const osg::Vec3& center, float radius );
signals:

public slots:


protected:
    int m_maxChildNumber;
    int m_maxTreeDepth;
    int m_maxLevel;
};

#endif // QOSGOCTREEBUILDER_H
