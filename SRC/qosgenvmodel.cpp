#include "qosgenvmodel.h"
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture>
#include <osgText/Text>
#include <osg/TexGen>
#include <osg/BlendFunc>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osg/Material>
#include <osg/ComputeBoundsVisitor>
#include <osg/LineWidth>

QOSGEnvModel::QOSGEnvModel(osg::ref_ptr<osg::Group> osgSceneRoot, QObject *parent) :
    QObject(parent),
    m_osgSceneRoot(osgSceneRoot)
{

}

osg::MatrixTransform *QOSGEnvModel::_getModelByID(const std::string &modelID, bool bRemove)
{
    int typeMode = -1;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter ;
    for(iter = m_cloudGroups.begin();iter != m_cloudGroups.end();++iter){
        if(modelID == (*iter).first){
            typeMode = 0;
            break;
        }
    }
    if(typeMode == -1){
        for(iter = m_BoxGroups.begin();iter != m_BoxGroups.end();++iter){
            if(modelID == (*iter).first){
                typeMode = 1;
                break;
            }
        }
        if(typeMode == -1){
            for(iter = m_CyliderGroups.begin();iter != m_CyliderGroups.end();++iter){
                if(modelID == (*iter).first){
                    typeMode = 2;
                    break;
                }
            }
            if(typeMode == -1){
                for(iter = m_PlaneGroups.begin();iter != m_PlaneGroups.end();++iter){
                    if(modelID == (*iter).first){
                        typeMode = 3;
                        break;
                    }
                }
                if(typeMode == -1){
                    for(iter = m_BoxEdgeGroups.begin();iter != m_BoxEdgeGroups.end();++iter){
                        if(modelID == (*iter).first){
                            typeMode = 4;
                            break;
                        }
                    }
                    if(typeMode == -1){
                        for(iter = m_FrameGroups.begin();iter != m_FrameGroups.end();++iter){
                            if(modelID == (*iter).first){
                                typeMode = 5;
                                break;
                            }
                        }
                        if(typeMode == -1){
                            for(iter = m_GridFloorGroups.begin();iter != m_GridFloorGroups.end();++iter){
                                if(modelID == (*iter).first){
                                    typeMode = 6;
                                    break;
                                }
                            }
                            if(typeMode == -1){
                                for(iter = m_ModelFromDiskGroup.begin();iter != m_ModelFromDiskGroup.end();++iter){
                                    if(modelID == (*iter).first){
                                        typeMode = 7;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(typeMode != -1){
        return _getModelByID(modelID,DrawModeType(typeMode),bRemove);
    }
    return NULL;
}

osg::MatrixTransform* QOSGEnvModel::_getModelByID(const std::string & modelID, DrawModeType drawMode, bool bRemove)
{
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
    osg::ref_ptr<osg::MatrixTransform> tempGroup;
    switch(drawMode){
    case QOSGEnvModel::PointCloud:{
        for(iter = m_cloudGroups.begin();iter != m_cloudGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_cloudGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::BoxModel:{
        for(iter = m_BoxGroups.begin();iter != m_BoxGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_BoxGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::CyliderModel:{
        for(iter = m_CyliderGroups.begin();iter != m_CyliderGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_CyliderGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::PlaneModel:{
        for(iter = m_PlaneGroups.begin();iter != m_PlaneGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_PlaneGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::BoxEdgeModel:{
        for(iter = m_BoxEdgeGroups.begin();iter != m_BoxEdgeGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_BoxEdgeGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::FrameModel:{
        for(iter = m_FrameGroups.begin();iter != m_FrameGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_FrameGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::GridModel:{
        for(iter = m_GridFloorGroups.begin();iter != m_GridFloorGroups.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_GridFloorGroups.erase(iter);
                }
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::ModelFromDisk:{
        for(iter = m_ModelFromDiskGroup.begin();iter != m_ModelFromDiskGroup.end();++iter){
            if(modelID == (*iter).first){
                osg::ref_ptr<osg::MatrixTransform> tempGroup1;
                tempGroup1 = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                if(!_bIsRemainData(tempGroup1)){
                    tempGroup = tempGroup1;
                    if(bRemove)
                        m_ModelFromDiskGroup.erase(iter);
                }
                break;
            }
        }
        break;
    }
    default:
        break;
    }
    return tempGroup.release();
}

void QOSGEnvModel::_removeRemainData(const std::string & cloudID, DrawModeType drawMode)//1 cloud
{
    osg::ref_ptr<osg::MatrixTransform> tempGroup;
    switch(drawMode){
    case QOSGEnvModel::PointCloud:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_cloudGroups.begin();iter != m_cloudGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    default:
        break;
    }
    if(!!tempGroup){
        std::vector<osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_remainDataGroups.begin();iter!=m_remainDataGroups.end();++iter){
            if(tempGroup == (*iter)){
                m_remainDataGroups.erase(iter);
                break;
            }
        }
    }
}

void QOSGEnvModel::_addRemainData(const std::string & cloudID, DrawModeType drawMode)//1 cloud
{
    osg::ref_ptr<osg::MatrixTransform> tempGroup;
    switch(drawMode){
    case QOSGEnvModel::PointCloud:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_cloudGroups.begin();iter != m_cloudGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::BoxModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_BoxGroups.begin();iter != m_BoxGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::CyliderModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_CyliderGroups.begin();iter != m_CyliderGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::PlaneModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_PlaneGroups.begin();iter != m_PlaneGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::BoxEdgeModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_BoxEdgeGroups.begin();iter != m_BoxEdgeGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::FrameModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_FrameGroups.begin();iter != m_FrameGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::GridModel:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_GridFloorGroups.begin();iter != m_GridFloorGroups.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    case QOSGEnvModel::ModelFromDisk:{
        std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_ModelFromDiskGroup.begin();iter != m_ModelFromDiskGroup.end();++iter){
            if(cloudID == (*iter).first){
                tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
                break;
            }
        }
        break;
    }
    default:
        break;
    }
    if(!!tempGroup)
        m_remainDataGroups.push_back(tempGroup);
}

void QOSGEnvModel::_removeOneTypeModelData(DrawModeType drawMode)
{
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
    switch(drawMode){
    case QOSGEnvModel::PointCloud:{
        for(iter = m_cloudGroups.begin();iter != m_cloudGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_cloudGroups.erase(iter++);
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::BoxModel:{
        for(iter = m_BoxGroups.begin();iter != m_BoxGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_BoxGroups.erase(iter++);
//                iter = m_BoxGroups.begin();
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::CyliderModel:{
        for(iter = m_CyliderGroups.begin();iter != m_CyliderGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_CyliderGroups.erase(iter++);
//                iter = m_CyliderGroups.begin();
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::PlaneModel:{
        for(iter = m_PlaneGroups.begin();iter != m_PlaneGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_PlaneGroups.erase(iter++);
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::BoxEdgeModel:{
        for(iter = m_BoxEdgeGroups.begin();iter != m_BoxEdgeGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_BoxEdgeGroups.erase(iter++);
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::FrameModel:{
        for(iter = m_FrameGroups.begin();iter != m_FrameGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_FrameGroups.erase(iter++);
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::GridModel:{
        for(iter = m_GridFloorGroups.begin();iter != m_GridFloorGroups.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_GridFloorGroups.erase(iter++);
            }else
                iter++;
        }
        break;
    }
    case QOSGEnvModel::ModelFromDisk:{
        for(iter = m_ModelFromDiskGroup.begin();iter != m_ModelFromDiskGroup.end();){
            osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::dynamic_pointer_cast<osg::MatrixTransform>((*iter).second);
            if(!_bIsRemainData(tempGroup)){
                m_osgSceneRoot->removeChild(tempGroup.get());
                m_ModelFromDiskGroup.erase(iter++);
            }else
                iter++;
        }
        break;
    }
    default:
        break;
    }
}

bool QOSGEnvModel::_bIsRemainData(osg::ref_ptr<osg::MatrixTransform> tempGroup)
{
    bool bRes = false;

    if(!!tempGroup){
        std::vector<osg::ref_ptr<osg::MatrixTransform> >::iterator iter;
        for(iter = m_remainDataGroups.begin(); iter != m_remainDataGroups.end(); ++iter){
            if(tempGroup == (*iter)){
                bRes = true;
                break;
            }
        }
    }
    return bRes;
}

osg::Group* QOSGEnvModel::_createGridWide(int netWidth,int netResolution,double gridWidth)
{
    osg::ref_ptr<osg::Group> gridGroup = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geo = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vecarray = new osg::Vec3Array;     //
    osg::ref_ptr<osg::DrawElementsUInt> vecIndex = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
    osg::ref_ptr<osg::Vec4Array> vc = new osg::Vec4Array();
    osg::ref_ptr<osg::Vec3Array> nc = new osg::Vec3Array();

    //
    int w = netWidth / netResolution + 1;
    int h = netWidth / netResolution + 1;

//    float *net = new float[6 * (w + h - 2)];
//    int *net_indice = new int[2 * (w + h)];

    GWRect rect;
    rect.left = -(gridWidth*(1-gridWidth/w/2)) * (w / 2.0) / (w - 1);
    rect.right = (gridWidth*(1-gridWidth/w/2)) * (w / 2.0) / (w - 1);

    rect.top = (gridWidth*(1-gridWidth/w/2)) * (h / 2.0) / (h - 1);
    rect.bottom = -(gridWidth*(1-gridWidth/w/2)) * (h / 2.0) / (h - 1);

    double delta_x = (rect.right - rect.left) / (w - 1);
    double delta_z = (rect.top - rect.bottom) / (h - 1);

    //
    int count1 = 1;
    float cal_x = 0.0, cal_z = 0.0;
    for(int i = 0; i < 2 * (w + h - 2); ++i){
        if(i < w){//
            cal_x = rect.left + delta_x * i;
            cal_z = rect.bottom;
        }else{
            if( i < w + 2 * (h - 2)){//

                if(0 == ((i - w) % 2)){//
                    cal_x = rect.left;
                }else{//
                    cal_x = rect.right;
                }
                cal_z = rect.bottom + delta_z * count1;

                if(1 == ((i - w) % 2)) count1++;
            }else{//
                int count2 = i - w - 2 * (h - 2);
                cal_x = rect.left + delta_x * count2;
                cal_z = rect.top;
            }
        }
        vecarray->push_back(osg::Vec3(cal_x, cal_z, 0.0));
        vc->push_back(osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f));
    }
    //
    int front = 0, rear = 0;
    int count3 = w;
    for(int i = 0; i < w + h; ++i){
        if(i < w){
            front = i;
            rear = front + w + 2 * (h - 2);
        }else if(i == w){
            vecIndex->push_back(0);
            vecIndex->push_back(w-1);
//            net_indice[2 * i] = 0;
//            net_indice[2 * i + 1] = w - 1;
            continue;
        }else if((w + h - 1) == i){
            front = w + 2 * (h - 2);
            rear = 2 * (w + h - 2) - 1;
        }else{

            front = count3;
            rear = front + 1;
            count3 += 2;
        }
        vecIndex->push_back(front);
        vecIndex->push_back(rear);
    }

    vc->push_back(osg::Vec4(1.0,1.0,0.0,1.0));
    nc->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));

    geo->setVertexArray(vecarray.get());
    geo->addPrimitiveSet(vecIndex.get());
    geo->setColorArray(vc.get());
    geo->setColorBinding(osg::Geometry::BIND_OVERALL);

    geo->setNormalArray(nc.get());
    geo->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geo->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE);
    geode->addDrawable(geo.get());
    gridGroup->addChild(geode);
    return gridGroup.release();
//    m_osgGridFloorSwitch->addChild(m_osgGridFloorGeode);
//    m_osgSceneRoot->addChild(m_osgGridFloorSwitch);
}

void QOSGEnvModel::_addBillboard(osg::ref_ptr<osg::Group> ManipPeesep,const std::string& textStr)
{
    // add text
    {
        osg::ref_ptr<osg::Group> ptextsep = new osg::Group();
        osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
        ManipPeesep->addChild(ptextsep);
        osg::ref_ptr<osgText::Text> text = new osgText::Text();

        //Set the screen alignment - always face the screen
        text->setFont("render/arial.ttf");
        text->setAxisAlignment(osgText::Text::SCREEN);
        text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
        text->setCharacterSize(28.0);
        text->setColor(osg::Vec4(1,0.85,0,1));
        text->setEnableDepthWrites(false);
        text->setBackdropType(osgText::Text::NONE);
        text->setBackdropColor(osg::Vec4(1,1,1,1));
        text->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
//        osg::CullFace* cf = new osg::CullFace( osg::CullFace::BACK );
//        text->getOrCreateStateSet()->setAttribute( cf );
        text->setText(textStr);//str(boost::format("EE%d")%index));
        textGeode->addDrawable(text);
        ptextsep->addChild(textGeode);
    }
}

void QOSGEnvModel::_changeModelColor(osg::ref_ptr<osg::Group> osgColorImage, const osg::Vec3 &imageColor, int textureMode)
{

        const int tMode = textureMode;
        osg::ref_ptr<osg::Image> imagePtr= _createImage(256,256,imageColor);
        if (!!imagePtr)
        {
            osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D();
            texture->setImage(imagePtr.get());
            //
            osg::ref_ptr<osg::TexGen> texgen=new osg::TexGen();
            texgen->setMode(osg::TexGen::SPHERE_MAP);
            osg::ref_ptr<osg::BlendEquation> blendEquation = new osg::BlendEquation(osg::BlendEquation::FUNC_ADD);
            blendEquation->setEquation(m_equations[tMode]);
            blendEquation->setDataVariance(osg::Object::DYNAMIC);
            //
            osg::ref_ptr<osg::BlendFunc>blendFunc = new osg::BlendFunc();
            blendFunc->setSource(osg::BlendFunc::SRC_ALPHA);
            blendFunc->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);

            osg::ref_ptr<osg::TexEnv> texenv=new osg::TexEnv;
            texenv->setMode(osg::TexEnv::ADD);
            texenv->setColor(osg::Vec4(1,1,1,0.6));
            osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
            stateset->setTextureAttributeAndModes(0,texture.get(),osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
            stateset->setTextureAttributeAndModes(0,texgen.get(),osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
            stateset->setTextureAttribute(0,texenv.get(),osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
            stateset->setAttributeAndModes(blendEquation,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
            if(textureMode !=0 ){
                stateset->setAttributeAndModes(blendFunc);
                stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);//取消深度测试
            }
            stateset->setDataVariance(osg::Object::DYNAMIC);
            osgColorImage->setStateSet(stateset);
        }

}

osg::Image *QOSGEnvModel::_createImage(int width, int height, const osg::Vec3 &color)
{
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage( width, height, 1, GL_RGB, GL_UNSIGNED_BYTE );
    unsigned char* data = image->data();
    for ( int y=0; y<height; ++y )
    {
        for ( int x=0; x<width; ++x )
        {
            *(data++) = color.x();
            *(data++) = color.y();
            *(data++) = color.z();
        }
    }
    return image.release();
}

void QOSGEnvModel::_setSelectItem(osg::ref_ptr<osg::MatrixTransform> selectNode)
{
    if(!selectNode){
        if(!!m_osgwireframe){
            m_osgwireframe->getParent(0)->removeChild(m_osgwireframe);
            m_osgwireframe = NULL;
        }
    }else{
        _SetVisualizationMode("selected",selectNode);
    }
}

bool QOSGEnvModel::_checkIsPointCloud(osg::ref_ptr<osg::MatrixTransform> selectNode)
{
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> >::iterator iter ;
    for(iter = m_cloudGroups.begin();iter!=m_cloudGroups.end();++iter){
        if(selectNode == (*iter).second)
            return true;
    }
    return false;
}

void QOSGEnvModel::_setCollisionItem(osg::ref_ptr<osg::MatrixTransform> selectNode)
{
    if(!selectNode){
        if(!!m_osgcollisionframe){
            m_osgcollisionframe->getParent(0)->removeChild(m_osgcollisionframe);
            m_osgcollisionframe = NULL;
        }
        if(!!m_boundingBoxGroup){
            m_osgSceneRoot->removeChild(m_boundingBoxGroup);
            m_boundingBoxGroup = NULL;
        }
    }else{
        if(_checkIsPointCloud(selectNode)){
            _SetVisualizationMode("pointCloudCollisioned",selectNode);
        }else
            _SetVisualizationMode("collisioned",selectNode);
    }
}

void QOSGEnvModel::_cretateBoundingBox(osg::ref_ptr<osg::MatrixTransform> selectNode)
{
//    bool bShow = true;
    if(!!m_boundingBoxGroup){
        m_osgSceneRoot->removeChild(m_boundingBoxGroup);
        m_boundingBoxGroup = NULL;
    }
    if(!!selectNode)
    {
        m_boundingBoxGroup = new osg::Group;
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        osg::ComputeBoundsVisitor boundVisitor;
        selectNode->accept(boundVisitor);
        osg::BoundingBox boundingBox = boundVisitor.getBoundingBox();

        float length = boundingBox.xMax() - boundingBox.xMin();
        float width = boundingBox.yMax() - boundingBox.yMin();
        float height = boundingBox.zMax() - boundingBox.zMin();
        osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(
            new osg::Box(boundingBox.center(), length, width, height));
        drawable->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));

        osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
        stateset = drawable->getOrCreateStateSet();
        osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode(
            osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
        stateset->setAttributeAndModes(polygonMode);
        osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(3.0);
        stateset->setAttribute(linewidth);
        stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE);
        geode->addDrawable(drawable);
        m_boundingBoxGroup->addChild(geode);
        m_osgSceneRoot->addChild(m_boundingBoxGroup);
    }
//    return geode;
}

void QOSGEnvModel::_SetVisualizationMode(const std::string& visualizationmode,osg::ref_ptr<osg::MatrixTransform> osgWorldTransform)
{
    if(!!osgWorldTransform)
    {

        // start the new node
        if( visualizationmode == "selected" ) {
            // have to undo the previous mode
            if( !!m_osgwireframe ) {
                m_osgwireframe->getParent(0)->removeChild(m_osgwireframe);
                m_osgwireframe = NULL;
            }
            m_osgwireframe = new osg::Group;
            m_osgwireframe->addChild(osgWorldTransform->getChild(0));
            osgWorldTransform->addChild(m_osgwireframe);
            _createVisualizaNode(m_osgwireframe);
        }else if( visualizationmode == "collisioned" ){
            if( !!m_osgcollisionframe ) {
                m_osgcollisionframe->getParent(0)->removeChild(m_osgcollisionframe);
                m_osgcollisionframe = NULL;
            }
            m_osgcollisionframe = new osg::Group;
            m_osgcollisionframe->addChild(osgWorldTransform->getChild(0));
            osgWorldTransform->addChild(m_osgcollisionframe);
            _createVisualizaNode(m_osgcollisionframe);
        }else if(visualizationmode == "pointCloudCollisioned" ){
            _cretateBoundingBox(osgWorldTransform);
        }
    }
}

void QOSGEnvModel::_createVisualizaNode(osg::ref_ptr<osg::Group> osgwireframe)
{
    // set up the state so that the underlying color is not seen through
    // and that the drawing mode is changed to wireframe, and a polygon offset
    // is added to ensure that we see the wireframe itself, and turn off
    // so texturing too.

    osg::ref_ptr<osg::BlendFunc> pBlendFunc = new osg::BlendFunc();
    pBlendFunc->setSource(osg::BlendFunc::SRC_ALPHA);
    pBlendFunc->setDestination(osg::BlendFunc::ONE_MINUS_SRC_ALPHA);

    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
    osg::ref_ptr<osg::PolygonOffset> polyoffset = new osg::PolygonOffset;
    polyoffset->setFactor(-1.0f);
    polyoffset->setUnits(-1.0f);
    osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode;
    polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::FILL);
    stateset->setAttributeAndModes(polyoffset,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    stateset->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    stateset->setAttributeAndModes(pBlendFunc);
#if 0
    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4f(0.777,0.887,1));
    material->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4f(0.777,0.887,1));

    stateset->setAttributeAndModes(material,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
#else
    // version which sets the color of the wireframe.
    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setColorMode(osg::Material::OFF); // switch glColor usage off
    // turn all lighting off
    material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0,0.749,1,0.5f));
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0,0.749,1,0.5f));
    material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0,0.749,1,0.5f));
    // except emission... in which we set the color we desire_osgwireframe
    material->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.777,0.887,1,0.3f));
    stateset->setAttributeAndModes(material,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
#endif

    stateset->setTextureMode(0,GL_TEXTURE_2D,osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);

//    osg::ref_ptr<osg::LineStipple> linestipple = new osg::LineStipple;
//    linestipple->setFactor(1);
//    linestipple->setPattern(0xf0f0);
//    stateset->setAttributeAndModes(linestipple,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

    osgwireframe->setStateSet(stateset);
}
