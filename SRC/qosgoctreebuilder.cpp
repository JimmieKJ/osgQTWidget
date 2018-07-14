#include "qosgoctreebuilder.h"

QOSGOctreeBuilder::QOSGOctreeBuilder(QObject *parent) :
    QObject(parent) ,
    m_maxChildNumber(16),
    m_maxTreeDepth(32),
    m_maxLevel(0)
{

}

osg::Group *QOSGOctreeBuilder::_build(int depth, const osg::BoundingBox &total, std::vector<QOSGOctreeBuilder::ElementInfo> &elements)
{
    int s[3];// (x,y,z)axis sides (0 or 1)
    osg::Vec3 extentSet[3] = {total._min,(total._min + total._max)*0.5f,total._max};
    std::vector<ElementInfo> childData;
    for(unsigned int i=0;i<elements.size();++i){
        const ElementInfo& obj = elements[i];
        if(total.contains(obj.second._min)&&total.contains(obj.second._max))
            childData.push_back(obj);
        else if(total.intersects(obj.second)){
            osg::Vec3 center = (obj.second._max + obj.second._min)*0.5f;
            if(total.contains(center)) childData.push_back(obj);
        }

    }
    bool isLeafNode = false;
    if((int)childData.size()<=m_maxChildNumber || depth > m_maxTreeDepth)
        isLeafNode = true;
    osg::ref_ptr<osg::Group> group = new osg::Group;
    if(!isLeafNode){
        osg::ref_ptr<osg::Group> childNodes[8];
        for ( s[0]=0; s[0]<2; ++s[0] ){
            for ( s[1]=0; s[1]<2; ++s[1] ){
                for ( s[2]=0; s[2]<2; ++s[2] ){
                    osg::Vec3 min,max;
                    for(int a=0;a<3;++a){
                        min[a] = (extentSet[s[a] + 0])[a];
                        max[a] = (extentSet[s[a] + 1])[a];
                    }
                    int id = s[0] + (2 * s[1]) + (4 * s[2]);
                    childNodes[id] = _build( depth+1, osg::BoundingBox(min, max), childData );
                }
            }
        }
        for ( unsigned int i=0; i<8; ++i )
        {
            if ( childNodes[i] && childNodes[i]->getNumChildren() )
                group->addChild( childNodes[i] );
        }
    }
    else{
        for ( unsigned int i=0; i<childData.size(); ++i )
        {
            const ElementInfo& obj = childData[i];
            group->addChild( obj.first );
        }
    }
    osg::Vec3 center = (total._max + total._min) * 0.5;
    float radius = (total._max - total._min).length() * 0.5f;
    osg::LOD* level = _createNewLevel( depth, center, radius );
//    level->insertChild( 0, createBoxForDebug(total._max, total._min) );  // For debug use
    level->insertChild( 0, group.get() );
    return level;
}

osg::LOD* QOSGOctreeBuilder::_createNewLevel( int level, const osg::Vec3& center, float radius )
{
    osg::ref_ptr<osg::LOD> lod = new osg::LOD;
    lod->setCenterMode( osg::LOD::USER_DEFINED_CENTER );
    lod->setCenter( center );
    lod->setRadius( radius );
//    lod->setRange( 0, radius * 5.0f, FLT_MAX );
    lod->setRange( 0, -1, radius * 60.0f );

    if ( m_maxLevel<level ) m_maxLevel = level;
    return lod.release();
}

//osg::Geode* OctreeBuilder::createBoxForDebug( const osg::Vec3& max, const osg::Vec3& min )
//{
//    osg::Vec3 dir = max - min;
//    osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array(10);
//    (*va)[0] = min + osg::Vec3(0.0f, 0.0f, 0.0f);
//    (*va)[1] = min + osg::Vec3(0.0f, 0.0f, dir[2]);
//    (*va)[2] = min + osg::Vec3(dir[0], 0.0f, 0.0f);
//    (*va)[3] = min + osg::Vec3(dir[0], 0.0f, dir[2]);
//    (*va)[4] = min + osg::Vec3(dir[0], dir[1], 0.0f);
//    (*va)[5] = min + osg::Vec3(dir[0], dir[1], dir[2]);
//    (*va)[6] = min + osg::Vec3(0.0f, dir[1], 0.0f);
//    (*va)[7] = min + osg::Vec3(0.0f, dir[1], dir[2]);
//    (*va)[8] = min + osg::Vec3(0.0f, 0.0f, 0.0f);
//    (*va)[9] = min + osg::Vec3(0.0f, 0.0f, dir[2]);

//    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
//    geom->setVertexArray( va.get() );
//    geom->addPrimitiveSet( new osg::DrawArrays(GL_QUAD_STRIP, 0, 10) );

//    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
//    geode->addDrawable( geom.get() );
//    geode->getOrCreateStateSet()->setAttribute(
//        new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE) );
//    geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
//    return geode.release();
//}
