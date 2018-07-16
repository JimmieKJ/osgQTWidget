#include "osgviewwidget.h"
#include "paramtrackballdragger.h"
#include "paramaxisdragger.h"
#include <QDebug>
#include <QGridLayout>
#include <boost/bind.hpp>
#include <osg/Vec4>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/CullFace>
#include <osgGA/MultiTouchTrackballManipulator>
#include <osg/ShapeDrawable>
#include <osg/Image>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/Material>
#include <QApplication>
#include <boost/format.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/Simplifier>
#define METERSINUNIT 0.01

OSGViewWidget::OSGViewWidget(QWidget *parent, Qt::WindowFlags f, ThreadingModel threadingModel)
     : QGLWidget(parent),
        m_osgview(new osgViewer::View),
        m_osghudview(new osgViewer::View),
        m_bIsSelectiveActive(true),
        m_osgFigureRoot(new osg::Group)
{
    setThreadingModel(threadingModel);
    setKeyEventSetsDone(0);
    _initOsgViewWidget();

}

void OSGViewWidget::_initOsgViewWidget()
{
    QWidget* viewWidget = _addViewWidget( osg::ref_ptr<osg::Camera>(_CreateCamera(0,0,100,100,METERSINUNIT)),osg::ref_ptr<osg::Camera>(_CreateHUDCamera(0,0,100,100, METERSINUNIT)));
    QGridLayout* grid = new QGridLayout;
    m_keyhandler = new QOSGKeyboardEventHandler(boost::bind(&OSGViewWidget::_HandleOSGKeyDown, this, _1, _2),boost::bind(&OSGViewWidget::_HandleOSGKeyUp, this, _1, _2));
    m_osgview->addEventHandler(m_keyhandler);
    m_osgSceneRoot = new osg::Group();
    grid->addWidget(viewWidget);
    grid->setContentsMargins(0, 0, 0, 0);
    setLayout(grid);
    m_osgEnvModel = new QOSGEnvModel(m_osgSceneRoot);
    m_osgSceneRoot->addChild(m_osgFigureRoot);
    m_picker = new OSGPickHandler(boost::bind(&OSGViewWidget::_HandleRayPick, this, _1, _2, _3), boost::bind(&OSGViewWidget::_pickDraggerFunc,this,_1,_2,_3,_4));
    m_osgview->addEventHandler(m_picker);

    //hud
    m_osgWorldAxis = new osg::MatrixTransform();
    m_osgWorldAxis->addChild(_CreateOSGXYZAxes(32, 2));
    if( !!m_osgCameraHUD ) {
        osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
        osg::ref_ptr<osg::Light> light(new osg::Light());
        // each light must have a unique number
        light->setLightNum(0);
        // we set the light's position via a PositionAttitudeTransform object
        light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
        light->setDiffuse(osg::Vec4(0, 0, 0, 1.0));
        light->setSpecular(osg::Vec4(0, 0, 0, 1.0));
        light->setAmbient( osg::Vec4(1, 1, 1, 1.0));
        lightSource->setLight(light.get());
        m_osgCameraHUD->addChild(lightSource.get());
        osg::Matrix m = m_osgCameraManipulator->getInverseMatrix();
        m.setTrans(this->width()/2 - 40, -this->height()/2 + 40, -50);
        m_osgWorldAxis->setMatrix(m);
        lightSource->addChild(m_osgWorldAxis.get());
    }
    m_collsionCheck = new QOSGCollsionCheck(m_osgview,m_osgEnvModel);
    m_collsionWorld = m_collsionCheck->_initCollision();
    connect( &m_timer, SIGNAL(timeout()), this, SLOT(updateGL()) );
    m_timer.start( 10 );
//    _SetHome();
}

void OSGViewWidget::_ChangeViewToXY()
{
    osg::Vec3d center = m_osgSceneRoot->getBound().center();
    osg::Matrix matCamera;
    matCamera.identity();
    matCamera.makeRotate( 0,osg::Vec3(1,0,0));
    matCamera.setTrans(center.x(), center.y(), center.z()+m_osgCameraManipulator->getDistance());
    _SetCameraTransform(matCamera);
}

void OSGViewWidget::_ChangeViewToXZ()
{
    osg::Vec3d center = m_osgSceneRoot->getBound().center();
    osg::Matrix matCamera;
    matCamera.identity();
    matCamera.makeRotate((M_PI/2.0),osg::Vec3(1,0,0));
    matCamera.setTrans(center.x(), center.y()-m_osgCameraManipulator->getDistance(),center.z());
    _SetCameraTransform(matCamera);
}

void OSGViewWidget::_SetCameraTransform(const osg::Matrix& matCamera)
{
    // has to come after setting distance because internally orbit manipulator uses the distance to deduct view center
    m_osgCameraManipulator->setByMatrix((matCamera));
}

void OSGViewWidget::_addDraggerToObject()
{
    m_draggerMatrix = new osg::MatrixTransform;
    m_osgSelectedNodeByDragger = new osg::Group;
    m_osgDraggerRoot = new osg::Group;
    m_draggers = _createDragger();
    for(size_t idragger = 0; idragger < m_draggers.size(); ++idragger){
        m_osgDraggerRoot->addChild(m_draggers.at(idragger));
    }
    m_osgDraggerRoot->addChild(m_draggerMatrix);
    osg::Matrixd selectedMatrix;
    m_osgSelectedNodeByDragger = m_selectedItem;
    selectedMatrix = m_selectedItem->getMatrix();
    m_osgSceneRoot->replaceChild(m_osgSelectedNodeByDragger,m_osgDraggerRoot);
    m_draggerMatrix->addChild(m_osgSelectedNodeByDragger);
    float scale = m_osgSelectedNodeByDragger->getBound().radius() * 0.8;
    selectedMatrix.preMult(osg::Matrix::scale(scale, scale, scale));
    for(size_t idragger = 0; idragger < m_draggers.size(); ++idragger) {
        m_draggers[idragger]->setMatrix(selectedMatrix);
    }
    for(size_t i_d = 0;i_d<m_draggers.size();++i_d) {
        m_draggers.at(i_d)->addTransformUpdating(m_draggerMatrix.get()); // in version 3.2 can specify what to transform
        // we want the dragger to handle it's own events automatically
        m_draggers.at(i_d)->setHandleEvents(true);
        m_draggers.at(i_d)->setActivationMouseButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON);
    }
//    m_draggerMatrix->remove();
    m_collsionCheck->_addDynamicObjectForCollsion(m_draggerMatrix->getMatrix(),m_draggerMatrix,m_collsionWorld);
    for(size_t idragger0 = 0; idragger0 < m_draggers.size(); ++idragger0) {
        for(size_t idragger1 = 0; idragger1 < m_draggers.size(); ++idragger1) {
            if( idragger0 != idragger1 ) {
                osg::ref_ptr<QOsgSelectedItemDraggerCallback> transRotsCallBack = new QOsgSelectedItemDraggerCallback(m_collsionWorld,m_draggerMatrix.get(),m_draggers[idragger0].get(), m_draggers[idragger1].get());
                m_draggers.at(idragger0)->addDraggerCallback(transRotsCallBack.get());
                transRotsCallBack->m_co = m_collsionCheck->m_co;
            }
        }
    }
}

std::vector<osg::ref_ptr<osgManipulator::Dragger> > OSGViewWidget::_createDragger()
{
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > draggers;
    {
        osg::ref_ptr<ParamTrackballDragger> d = new ParamTrackballDragger();
        d->setPickCylinderHeight(0.09f);
        d->setupDefaultGeometry();
        d->m_ControllItem = m_selectedItem;
        d->m_windowHeightPix = this->height();
        d->m_windowWithPix = this->width();
        draggers.push_back(d);
        osg::ref_ptr< ParamAxisDragger> d2 = new ParamAxisDragger();
        d2->setPickCylinderRadius(0.02);
        d2->setConeHeight(0.2);
        d2->setupDefaultGeometry();
        d2->m_windowHeightPix = this->height();
        d2->m_windowWithPix = this->width();
        // scale the axes so that they are bigger. since d and d2 need to share the same transform, have to add a scale node in between
        osg::ref_ptr<osg::MatrixTransform> pscaleparent(new osg::MatrixTransform);
        pscaleparent->setMatrix(osg::Matrix::scale(1.7, 1.7, 1.7));
        d2->setAxisLineWidth(10);
        for(size_t ichild = 0; ichild < d2->getNumChildren(); ++ichild) {
            pscaleparent->addChild(d2->getChild(ichild));
        }
        d2->removeChildren(0, d2->getNumChildren());
        d2->addChild(pscaleparent);
        draggers.push_back(d2);
    }
    return draggers;
}

void OSGViewWidget::_ChangeViewToYZ()
{
    osg::Vec3d center = m_osgSceneRoot->getBound().center();
    osg::Matrix matCamera;
    matCamera.identity();
    matCamera.makeRotate((osg::Quat((M_PI/2.0),osg::Vec3(0,0,1)) * osg::Quat((M_PI/2.0),osg::Vec3(1,0,0))));
    matCamera.setTrans(center.x(),center.y(),center.z());
    _SetCameraTransform(matCamera);
}

osg::Camera* OSGViewWidget::_CreateHUDCamera( int x, int y, int w, int h, double metersinunit)
{
    osg::ref_ptr<osg::Camera> camera(new osg::Camera());
    camera->setProjectionMatrix(osg::Matrix::ortho(-1,1,-1,1,1,10));

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setCullingMode(camera->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING); // need this for allowing small points with zero bunding voluem to be displayed correctly
    return camera.release();
}


void OSGViewWidget::_SelectOSGLink(osg::ref_ptr<osg::Node> node, int modkeymask)
{
    _ClearDragger();
    if (!!node) {
        osg::ref_ptr<osg::Transform> tempGroup = node->asTransform();
        m_selectedItem = osg::dynamic_pointer_cast<osg::MatrixTransform>(tempGroup);
        if(!(modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
            // user clicked on empty region, so remove selection
            m_osgEnvModel->_setSelectItem(m_selectedItem);
        }
        if(!!m_selectedItem){
            _addDraggerToObject();
        }
    }
    else{
       m_selectedItem = osg::ref_ptr<osg::MatrixTransform>();
       m_osgEnvModel->_setSelectItem(osg::ref_ptr<osg::MatrixTransform>());
   }
}

void OSGViewWidget::_pickDraggerFunc(int buttonPressed, const osgGA::GUIEventAdapter &ea, bool bMouseBtnPush, bool bMouseBtnRelease)
{
    if( !!m_selectedItem && 0 == buttonPressed) {
        // have to update the underlying openrave model since dragger is most likely changing the positions
//        m_selectedItem->UpdateFromOSG();
//        emit changeTransformParam(true);
    }
//    if("SelectItemMode" == m_draggerName && 0 == buttonPressed /*&& (modkeymask & osgGA::GUIEventAdapter::MODKEY_SHIFT)*/){
//        if(bMouseBtnRelease){
//            if(!!m_selectBoxGroupPtr){
//                m_osgCameraHUD->removeChild(m_selectBoxGroupPtr.get());
////                m_selectBoxGroupPtr.release();
//            }
//            return;
//        }
//        if(!m_bstartBoxSelect)
//            return;
//        osg::Viewport* viewport = m_osghudview->getCamera()->getViewport();
//        osg::Matrix localCameraVPW = m_osghudview->getCamera()->getViewMatrix() * m_osghudview->getCamera()->getProjectionMatrix();
//        if (viewport) localCameraVPW *= viewport->computeWindowMatrix();
//        osg::Matrix inveVPWmatrix(osg::Matrix::inverse(localCameraVPW));

//        osg::Matrix localMainCameraVPW = m_osgview->getCamera()->getViewMatrix() * m_osgview->getCamera()->getProjectionMatrix();
//        osg::Viewport* viewport1 = m_osgview->getCamera()->getViewport();
//        if (viewport1) localMainCameraVPW *= viewport1->computeWindowMatrix();
//        osg::Vec3d pushBtnPosInScreen = osg::Vec3d(ea.getX(),ea.getY(),0.0);
//        osg::Vec3d new_coord = pushBtnPosInScreen * inveVPWmatrix;
//        double sceneX = new_coord.x();
//        double sceneY = new_coord.y();
//        double sceneZ = new_coord.z();
//        osg::Vec3 mouseEndPos(sceneX,sceneY,sceneZ);
//        if(!bMouseBtnPush){
//            m_pushBtnPosInScreen = pushBtnPosInScreen;
//            m_pushBtnPosInScene = mouseEndPos;
//            bMouseBtnPush = true;
//        }else{
//            switch(m_iBoxSelect)
//            {
//            case 1:{
//                std::vector<OpenRAVE::KinBodyPtr> kinbodys ;
//                m_penv->GetBodies(kinbodys);
//                for(int i=0;i<kinbodys.size();i++){
//                    aqroserave::KinBodyItemPtr pitem = boost::dynamic_pointer_cast<aqroserave::KinBodyItem>(kinbodys[i]->GetUserData(m_userdatakey));
//                    if(!!pitem){
//                        pitem->SetVisualizationMode("");

//                        osg::Vec3d model_coord = pitem->GetOSGRoot()->getMatrix().getTrans() * localMainCameraVPW;
//                        if(model_coord.x()>m_pushBtnPosInScreen.x() &&
//                                model_coord.x()<pushBtnPosInScreen.x() &&
//                                model_coord.y()<m_pushBtnPosInScreen.y() &&
//                                model_coord.y()>pushBtnPosInScreen.y() ){
//                            pitem->SetVisualizationMode("selected");
//                            emit _heightLightTreeItem(pitem->GetName());
//                        }
//                    }
//                }
//                break;
//            }
//            case 2:{
//                std::vector<OpenRAVE::KinBodyPtr> kinbodys ;
//                m_pointSelector->_deleteSelectPoint();
//                m_penv->GetBodies(kinbodys);
//                for(int i=0;i<kinbodys.size();i++){
//                    if(QString::fromStdString(kinbodys[i]->GetName()).endsWith(".wor")){
//                        aqroserave::KinBodyItemPtr pitem = boost::dynamic_pointer_cast<aqroserave::KinBodyItem>(kinbodys[i]->GetUserData(m_userdatakey));
//                        if(!!pitem)
//                            m_pointSelector->_boxSelectPoint(pitem,m_pushBtnPosInScreen,pushBtnPosInScreen,localMainCameraVPW);
//                    }
//                }
//                break;
//            }
//            case 3:{
//                std::vector<OpenRAVE::KinBodyPtr> kinbodys ;
//                m_lineSelector->_deleteSelectPoint();
//                m_penv->GetBodies(kinbodys);
//                for(int i=0;i<kinbodys.size();i++){
//                    if(QString::fromStdString(kinbodys[i]->GetName()).endsWith(".wor")){
//                        aqroserave::KinBodyItemPtr pitem = boost::dynamic_pointer_cast<aqroserave::KinBodyItem>(kinbodys[i]->GetUserData(m_userdatakey));
//                        if(!!pitem)
//                            m_lineSelector->_boxSelectLine(pitem,m_pushBtnPosInScreen,pushBtnPosInScreen,localMainCameraVPW);
//                    }
//                }
//                break;
//            }
//            case 4:{
//                std::vector<OpenRAVE::KinBodyPtr> kinbodys ;
//                m_faceSelector->_deleteSelectPoint();
//                m_penv->GetBodies(kinbodys);
//                for(int i=0;i<kinbodys.size();i++){
//                    if(QString::fromStdString(kinbodys[i]->GetName()).endsWith(".wor")){
//                        aqroserave::KinBodyItemPtr pitem = boost::dynamic_pointer_cast<aqroserave::KinBodyItem>(kinbodys[i]->GetUserData(m_userdatakey));
//                        if(!!pitem)
//                            m_faceSelector->_boxSelectSurface(pitem,m_pushBtnPosInScreen,pushBtnPosInScreen,localMainCameraVPW);
//                    }
//                }

//                break;
//            }
//            default:
//                break;
//            }

//        }
//        if(!!m_selectBoxGroupPtr){
//            m_osgCameraHUD->removeChild(m_selectBoxGroupPtr.get());
////            m_selectBoxGroupPtr.release();
//        }
//        m_selectBoxGroupPtr = new osg::Group;
//        osg::ref_ptr<osg::Geode> geoSelectBox = new osg::Geode;
//        osg::Vec3d coord_1 = m_pushBtnPosInScene;
//        osg::Vec3d coord_4(mouseEndPos.x(),m_pushBtnPosInScene.y(),mouseEndPos.z());
//        osg::Vec3d coord_3 = mouseEndPos;
//        osg::Vec3d coord_2 = mouseEndPos + (coord_1 - coord_4);
//        osg::Vec4 color ;
//        color = osg::Vec4(0.8f, 0.8f, 0.0f,1.0f);
//        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
//        osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
//        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
//        coords->push_back(coord_1);
//        coords->push_back(coord_4);
//        coords->push_back(coord_3);
//        coords->push_back(coord_2);
//        colors->push_back(color);
//        colors->push_back(color);
//        colors->push_back(color);
//        colors->push_back(color);
//        geometry->setVertexArray(coords.get());
//        geometry->setColorArray(colors.get());
//        geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
//        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP , 0 , 4)) ; //
//        osg::StateSet* state = geometry->getOrCreateStateSet();
//        state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
//        osg::CullFace* cf = new osg::CullFace( osg::CullFace::BACK );
//        state->setAttribute( cf );
//        geoSelectBox->addDrawable(geometry.get());
//        m_selectBoxGroupPtr->addChild(geoSelectBox);
//        m_osgCameraHUD->addChild(m_selectBoxGroupPtr);
//    }
}

void OSGViewWidget::_ClearDragger()
{
    if( m_osgSelectedNodeByDragger.valid() && m_osgDraggerRoot.valid() ) {
        osg::ref_ptr<osg::Group> parent = new osg::Group;
        if( m_osgDraggerRoot->getNumParents() > 0 ) {
            parent = m_osgDraggerRoot->getParent(0);
        }
        else {
            parent = m_osgSceneRoot; // fallback
        }
        if(parent->replaceChild(m_osgDraggerRoot, m_osgSelectedNodeByDragger) ) {
            for(int i_d = 0;i_d<m_draggers.size();++i_d) {//w
                if (!!m_draggers.at(i_d) && m_draggers.at(i_d)->getNumParents() > 0) {
                    if(i_d == 0){
                        osg::Matrix temMatrix;
                        temMatrix.identity();
                        temMatrix.makeRotate(m_draggers.at(i_d)->getMatrix().getRotate());
                        temMatrix.setTrans(m_draggers.at(i_d)->getMatrix().getTrans());
                        m_selectedItem->setMatrix(temMatrix);
                    }
                    m_draggers.at(i_d)->getParent(0)->removeChild(m_draggers.at(i_d));
                }
            }
        }
    }
    if( !!m_draggerMatrix ) {
        m_draggerMatrix->setMatrix(osg::Matrix::identity());//w
    }
    m_collsionCheck->_deleteDynamicCollsion(m_collsionWorld);
    m_draggers.clear();
}

void OSGViewWidget::_HandleRayPick(osg::Node* node, int buttonPressed, int modkeymask)
{
    if (!node) {
//        if(buttonPressed == 3){
//            emit _showCenterMenu();
//            return;
//        }
        if( buttonPressed ) {
            m_osgCameraManipulator->_SetSeekMode(false);
        }
        if( m_bIsSelectiveActive && buttonPressed ) {
            _SelectOSGLink(osg::ref_ptr<osg::Node>(), modkeymask);
        }
    }
    else {
//        osg::ref_ptr<osg::Node> node = intersection.nodePath.front();
        // something hit
        if( buttonPressed ) {
//            if(buttonPressed == 3){
//                aqroserave::KinBodyItemPtr item = _FindKinBodyItemFromOSGNode(node);
//                if( !!item ) {
//                    emit _showWorldModeMenu(item);
//                }
//                return;
//            }
//            if(buttonPressed == 2 && !!node){
//                // draw the intersection point in the HUD
//                //aqroserave::KinBodyItemPtr item = _FindKinBodyItemFromOSGNode(node);
////                aqroserave::KinBodyItemPtr item = _FindKinBodyItemFromOSGNode(node);
//                /*if( !!node )*/ {
////                    KinBody::LinkPtr link = item->GetLinkFromOSG(node);
////                    KinBody::JointPtr joint = _FindJoint(item, link);
//                    std::string itemname = item->GetName();
//                    emit showDetailWidget(itemname,link,joint);
//                    return;
//                }
//            }
            if( m_bIsSelectiveActive &&!!node ) {
                _SelectOSGLink(osg::ref_ptr<osg::Node>(node), modkeymask);
            }
        }
        else {
            // draw the intersection point in the HUD
//            aqroserave::KinBodyItemPtr item = _FindKinBodyItemFromOSGNode(node);

//            if( !!item ) {
//                osg::Vec3d pos = intersection.getWorldIntersectPoint();
//                osg::Vec3d normal = intersection.getWorldIntersectNormal();
//                KinBody::LinkPtr link = item->GetLinkFromOSG(node);
//                std::string linkname;
//                if( !!link ) {
//                    linkname = link->GetName();
//                }
//                m_strRayInfoText = str(boost::format("mouse on %s:%s: (%.5f, %.5f, %.5f), n=(%.5f, %.5f, %.5f)")%item->GetName()%linkname%pos.x()%pos.y()%pos.z()%normal.x()%normal.y()%normal.z());
//            }
//            else {
//                m_strRayInfoText.clear();
//            }
//            _UpdateHUDText();
        }
    }

}

void OSGViewWidget::_createFrame()
{
//    m_osgWorldAxis = new osg::MatrixTransform();
//    m_osgWorldAxis->addChild(_CreateOSGXYZAxes(32.0, 2.0));
    m_osgWorldCenterAxisSwitch = new osg::Switch();
    m_osgWorldCenterAxis = new osg::MatrixTransform();
    m_osgWorldCenterAxis->addChild(osg::ref_ptr<osg::Group>(_CreateOSGXYZAxes(1.0, METERSINUNIT)));
    m_osgWorldCenterAxisSwitch->addChild(m_osgWorldCenterAxis);
    m_osgSceneRoot->addChild(m_osgWorldCenterAxisSwitch);
}

osg::Group* OSGViewWidget::_CreateOSGXYZAxes(double len, double axisthickness)
{
    osg::Vec4f colors[] = {
        osg::Vec4f(0,0,1,1),
        osg::Vec4f(0,1,0,1),
        osg::Vec4f(1,0,0,1)
    };
    osg::Quat rotations[] = {
        osg::Quat(0, osg::Vec3f(0,0,1)),
        osg::Quat(-M_PI/2.0, osg::Vec3f(1,0,0)),
        osg::Quat(M_PI/2.0, osg::Vec3f(0,1,0))
    };

    osg::ref_ptr<osg::Group> proot = new osg::Group();

    // add 3 cylinder+cone axes
    for(int i = 0; i < 3; ++i) {
        osg::ref_ptr<osg::MatrixTransform> psep = new osg::MatrixTransform();
        //psep->setMatrix(osg::Matrix::translate(-16.0f,-16.0f,-16.0f));

        // set a diffuse color
        osg::ref_ptr<osg::StateSet> state = psep->getOrCreateStateSet();
        osg::ref_ptr<osg::Material> mat = new osg::Material;
        mat->setDiffuse(osg::Material::FRONT, colors[i]);
        mat->setAmbient(osg::Material::FRONT, colors[i]);
        state->setAttribute( mat );
//        osg::CullFace* cf = new osg::CullFace( osg::CullFace::BACK );
//        state->setAttribute( cf );
        osg::Matrix matrix;
        osg::ref_ptr<osg::MatrixTransform> protation = new osg::MatrixTransform();
        matrix.makeRotate(rotations[i]);
        protation->setMatrix(matrix);

        matrix.makeIdentity();
        osg::ref_ptr<osg::MatrixTransform> pcyltrans = new osg::MatrixTransform();
        matrix.setTrans(osg::Vec3f(0,0,0.5*len));
        pcyltrans->setMatrix(matrix);

        // make SoCylinder point towards z, not y
        osg::ref_ptr<osg::Cylinder> cy = new osg::Cylinder();
        cy->setRadius(axisthickness);
        cy->setHeight(len);
        osg::ref_ptr<osg::Geode> gcyl = new osg::Geode;
        osg::ref_ptr<osg::ShapeDrawable> sdcyl = new osg::ShapeDrawable(cy);
        sdcyl->setColor(colors[i]);
        gcyl->addDrawable(sdcyl.get());

        osg::ref_ptr<osg::Cone> cone = new osg::Cone();
        cone->setRadius(axisthickness*2);
        cone->setHeight(len*0.25);

        osg::ref_ptr<osg::Geode> gcone = new osg::Geode;
        osg::ref_ptr<osg::ShapeDrawable> sdcone = new osg::ShapeDrawable(cone);
        gcone->addDrawable(sdcone.get());
        sdcone->setColor(colors[i]);

        matrix.makeIdentity();
        osg::ref_ptr<osg::MatrixTransform> pconetrans = new osg::MatrixTransform();
        matrix.setTrans(osg::Vec3f(0,0,len));
        pconetrans->setMatrix(matrix);

        psep->addChild(protation);
        protation->addChild(pcyltrans);
        pcyltrans->addChild(gcyl.get());
        protation->addChild(pconetrans);
        pconetrans->addChild(gcone.get());
        proot->addChild(psep);
    }

    return proot.release();
}

void OSGViewWidget::_loadFileNodeToSceneRoot(const Eigen::Affine3f &transfer,const std::string& modelId,const std::string& nodeName)
{
    _removeModelByID(modelId,7);
    osg::ref_ptr<osg::MatrixTransform> boxTrans = new osg::MatrixTransform;
    osg::Node* dataNode = osgDB::readNodeFile(nodeName) ;
    osg::Matrix tmpmat = _getMatrixFromAffine3f(transfer);
    boxTrans->setMatrix(tmpmat);
    boxTrans->addChild(dataNode);
    boxTrans->setName(modelId);
    m_osgEnvModel->m_ModelFromDiskGroup.insert(std::pair<std::string, osg::ref_ptr<osg::MatrixTransform> >(modelId,boxTrans));
    m_osgSceneRoot->addChild(boxTrans);
}

void OSGViewWidget::_SetHome()
{
    osg::BoundingSphere bs = m_osgSceneRoot->getBound();
     m_osgview->getCameraManipulator()->setHomePosition(osg::Vec3d(0.6*bs.radius(),0,0.6*bs.radius()),osg::Vec3d(0,0,0),osg::Vec3d(0.0,0.0,1.0));
//    m_osgview->getCameraManipulator()->setHomePosition(osg::Vec3d(0.6*bs.radius(),-0.8*bs.radius(),-1*bs.radius()),bs.center(),osg::Vec3d(0,0,1));
    m_osgview->home();
//    bs = m_osgSceneRoot->getBound();
//    m_osgview->getCameraManipulator()->setHomePosition(osg::Vec3d(0.3*bs.radius(),1,-1*bs.radius()),bs.center(),osg::Vec3d(0.0,0.0,1.0));
//    m_osgview->home();
}

bool OSGViewWidget::_HandleOSGKeyDown(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    int key = ea.getKey();
//    if( !!_onKeyDown ) {
//        if( _onKeyDown(key) ) {
//            return true;
//        }
//    }

    if( key == 'f' || key == 'F') {
        m_osgCameraManipulator->_SetSeekMode(!m_osgCameraManipulator->_InSeekMode());
    }/*else if(key == osgGA::GUIEventAdapter::KEY_Alt_L){
        emit changeKeyMod(-3);
    }*//*else if(key == osgGA::GUIEventAdapter::KEY_Delete){
        emit changeToRotateMod(true);
        _deleteKinBodyFromEnv();
    }*/
    return false;
}

bool OSGViewWidget::_HandleOSGKeyUp(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    int key = ea.getKey();

    if( key == osgGA::GUIEventAdapter::KEY_Alt_L ) {
//        emit changeKeyMod(-2);
//        _ClearDragger();
//        m_draggerName.clear();
    }
    return true;
}

void OSGViewWidget::_repaintView()
{
    osg::ref_ptr<osg::Group> rootscene(new osg::Group());
    //  Normalize object normals
    rootscene->getOrCreateStateSet()->setMode(GL_NORMALIZE,osg::StateAttribute::ON);
    rootscene->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
    rootscene->addChild(m_osgSceneRoot);
    m_updateCallBace = new QOSGUpdateCallbackCheckCollsion(this->width(),this->height(),METERSINUNIT,m_collsionWorld,m_collsionCheck,boost::bind(&OSGViewWidget::_SetViewport, this, _1, _2, _3));
    rootscene->addUpdateCallback(m_updateCallBace);
    m_osgview->setSceneData(rootscene);

}


void OSGViewWidget::_removeModelByID(const std::string & modelID, int drawMode)
{
    osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::ref_ptr<osg::MatrixTransform>(m_osgEnvModel->_getModelByID(modelID,QOSGEnvModel::DrawModeType(drawMode),true));
    if(!!tempGroup)
        m_osgSceneRoot->removeChild(tempGroup.get());
}

void OSGViewWidget::_removeAllData()
{
    _removeOneTypeModelData(0);
    _removeOneTypeModelData(1);
    _removeOneTypeModelData(2);
    _removeOneTypeModelData(3);
    _removeOneTypeModelData(4);
    _removeOneTypeModelData(5);
    _removeOneTypeModelData(6);
    _removeOneTypeModelData(7);
}


void OSGViewWidget::_addBoxModel(double length, double width, double height, const std::string &boxID, const Eigen::Affine3f &transfer, int r, int g, int b,bool bTransparent)
{
    _removeModelByID(boxID,1);
    osg::ref_ptr<osg::MatrixTransform> boxTrans = new osg::MatrixTransform;
    osg::ref_ptr<osg::Group> boxData = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(0.5f);
    geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0,0,0),length,width,height),hints));
    osg::Matrix tmpmat = _getMatrixFromAffine3f(transfer);
    boxTrans->setMatrix(tmpmat);
    boxData->addChild(geode);
    int iTrans = bTransparent ? 3 : 0;
    m_osgEnvModel->_changeModelColor(boxData,osg::Vec3(r,g,b),iTrans);
    boxTrans->addChild(boxData);
    boxTrans->setName(boxID);
    m_osgEnvModel->m_BoxGroups.insert(std::pair<std::string, osg::ref_ptr<osg::MatrixTransform> >(boxID,boxTrans));
    m_osgSceneRoot->addChild(boxTrans);
}



osg::Matrix OSGViewWidget::_getMatrixFromAffine3f(const Eigen::Affine3f &transfer)
{
    Eigen::Vector3f transEig = transfer.translation();
    Eigen::Matrix3f rotEig = transfer.rotation();
    Eigen::Quaternionf quatEig = Eigen::Quaternionf(rotEig);
    osg::Matrix tmpmat;
    tmpmat.identity();
    tmpmat.makeTranslate(osg::Vec3(transEig[0],transEig[1],transEig[2]));
    tmpmat.setRotate(osg::Quat(quatEig.x(), quatEig.y(), quatEig.z(), quatEig.w()));
//    _PrintMatrix(tmpmat);
    return tmpmat;
}

void OSGViewWidget::_addCylinder(double length, double radius, const std::string &cyLinderID, const Eigen::Affine3f &transfer, int r, int g, int b, bool bTransparent)
{
    _removeModelByID(cyLinderID,2);
    osg::ref_ptr<osg::MatrixTransform> boxTrans = new osg::MatrixTransform;
    osg::ref_ptr<osg::Group> boxData = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(0.5f);
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0,0,0),radius,length),hints));
    osg::Matrix tmpmat = _getMatrixFromAffine3f(transfer);
    boxTrans->setMatrix(tmpmat);
    boxData->addChild(geode);
    int iTrans = bTransparent ? 4 : 0;
    m_osgEnvModel->_changeModelColor(boxData,osg::Vec3(r,g,b),iTrans);
    boxTrans->addChild(boxData);
    boxTrans->setName(cyLinderID);
    m_osgEnvModel->m_CyliderGroups.insert(std::pair<std::string, osg::ref_ptr<osg::MatrixTransform> >(cyLinderID,boxTrans));
    m_osgSceneRoot->addChild(boxTrans);
}

void OSGViewWidget::_addPlane(double length, double width, const std::string &PlaneID, const Eigen::Affine3f& transfer , int r, int g, int b, bool bTransparent)
{
    _removeModelByID(PlaneID,3);
    osg::ref_ptr<osg::MatrixTransform> boxTrans = new osg::MatrixTransform;
    osg::ref_ptr<osg::Group> boxData = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geo = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vecarray = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colorarray = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec3Array> nc = new osg::Vec3Array;
    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,0.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,0.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,0.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,0.0));
    colorarray->push_back(osg::Vec4(r, g, b, 1.0f));
    nc->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
    geo->setVertexArray(vecarray.get());
    geo->setColorArray(colorarray.get());
    geo->setColorBinding(osg::Geometry::BIND_OVERALL);
    geo->setNormalArray(nc.get());
    geo->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS ,0,4)) ; //
    geode->addDrawable(geo);
    osg::Matrix tmpmat = _getMatrixFromAffine3f(transfer);
    boxTrans->setMatrix(tmpmat);
    boxData->addChild(geode);
    int iTrans = bTransparent ? 4 : 0;
    m_osgEnvModel->_changeModelColor(boxData,osg::Vec3(r,g,b),iTrans);
    boxTrans->addChild(boxData);
    boxTrans->setName(PlaneID);
    m_osgEnvModel->m_PlaneGroups.insert(std::pair<std::string, osg::ref_ptr<osg::MatrixTransform> >(PlaneID,boxTrans));
    m_osgSceneRoot->addChild(boxTrans);
}

void OSGViewWidget::_addBoxEdge(double length, double width, double height, const std::string &boxEdgeID, const Eigen::Affine3f &transfer, int r, int g, int b)
{
    _removeModelByID(boxEdgeID,4);
    osg::ref_ptr<osg::MatrixTransform> boxTrans = new osg::MatrixTransform;
    osg::ref_ptr<osg::Group> boxData = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geo = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vecarray = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colorarray = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec3Array> nc = new osg::Vec3Array;
    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,-height/2.0));

    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,height/2.0));

    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,-width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,height/2.0));

    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(length/2.0,width/2.0,-height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,height/2.0));
    vecarray->push_back(osg::Vec3(-length/2.0,-width/2.0,-height/2.0));

    colorarray->push_back(osg::Vec4(r, g, b, 1.0f));
    nc->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
    geo->setVertexArray(vecarray.get());
    geo->setColorArray(colorarray.get());
    geo->setColorBinding(osg::Geometry::BIND_OVERALL);
    geo->setNormalArray(nc.get());
    geo->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES ,0,34)) ; //
    geode->addDrawable(geo);
    osg::Matrix tmpmat = _getMatrixFromAffine3f(transfer);
    tmpmat.identity();
    boxTrans->setMatrix(tmpmat);
    boxData->addChild(geode);
    m_osgEnvModel->_changeModelColor(boxData,osg::Vec3(r,g,b));
    boxTrans->addChild(boxData);
    boxTrans->setName(boxEdgeID);
    m_osgEnvModel->m_BoxEdgeGroups.insert(std::pair<std::string, osg::ref_ptr<osg::MatrixTransform> >(boxEdgeID,boxTrans));
    m_osgSceneRoot->addChild(boxTrans);
}

void OSGViewWidget::_SetViewport(int width, int height, double metersinunit)
{
    m_osgview->getCamera()->setViewport(0,0,width,height);
    m_osghudview->getCamera()->setViewport(0,0,width,height);
    m_osghudview->getCamera()->setProjectionMatrix(osg::Matrix::ortho(-width/2, width/2, -height/2, height/2, METERSINUNIT/metersinunit, 100.0/metersinunit));

    osg::Matrix m = m_osgCameraManipulator->getInverseMatrix();
    m.setTrans(width/2 - 40, -height/2 + 40, -50);
    m_osgWorldAxis->setMatrix(m);

//    double textheight = (10.0/480.0)*height;
//    m_osgHudText->setPosition(osg::Vec3(-width/2+10, height/2-textheight, -50));
//    m_osgHudText->setCharacterSize(textheight);
}

void OSGViewWidget::_addGridFloor(int netWidth, int netResolution, double gridWidth, const std::string &id, const Eigen::Affine3f &transfer)
{
    _removeModelByID(id,6);
    osg::ref_ptr<osg::MatrixTransform> osgGridModeTrans = new osg::MatrixTransform();
    osgGridModeTrans->addChild(osg::ref_ptr<osg::Group>(m_osgEnvModel->_createGridWide(netWidth,netResolution,gridWidth)));
    osg::Matrix mat = _getMatrixFromAffine3f(transfer);
    osgGridModeTrans->setMatrix(mat);
    osg::ref_ptr<osg::Group> ManipPeesep = new osg::Group();
    osgGridModeTrans->addChild(ManipPeesep);
    osgGridModeTrans->setName(id);
    m_osgEnvModel->m_GridFloorGroups.insert(std::pair<std::string,osg::ref_ptr<osg::MatrixTransform> >(id,osgGridModeTrans));
    m_osgSceneRoot->addChild(osgGridModeTrans);
}

void OSGViewWidget::_addFrame(double length, const std::string &id, const Eigen::Affine3f &transfer)
{
    _removeModelByID(id,5);
    osg::ref_ptr<osg::MatrixTransform> osgWorldCenterAxis = new osg::MatrixTransform();
    osgWorldCenterAxis->addChild(osg::ref_ptr<osg::Group>(_CreateOSGXYZAxes(length, length/100.0)));
    osg::Matrix mat = _getMatrixFromAffine3f(transfer);
    osgWorldCenterAxis->setMatrix(mat);
    osg::ref_ptr<osg::Group> ManipPeesep = new osg::Group();
    osgWorldCenterAxis->addChild(ManipPeesep);
    osgWorldCenterAxis->setName(id);
    m_osgEnvModel->_addBillboard(ManipPeesep,id);
    m_osgEnvModel->m_FrameGroups.insert(std::pair<std::string,osg::ref_ptr<osg::MatrixTransform> >(id,osgWorldCenterAxis));
    m_osgSceneRoot->addChild(osgWorldCenterAxis);
}

void OSGViewWidget::_hideModelByID(const std::string &modelID, int drawMode)
{
    if(!m_envModelSwitch){
        m_envModelSwitch = new osg::Switch;
        m_osgSceneRoot->addChild(m_envModelSwitch);
    }
    osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::ref_ptr<osg::MatrixTransform>(m_osgEnvModel->_getModelByID(modelID,QOSGEnvModel::DrawModeType(drawMode)));
    if(!!tempGroup){
        m_osgSceneRoot->removeChild(tempGroup);
        m_envModelSwitch->addChild(tempGroup);
        m_envModelSwitch->setAllChildrenOff();
    }
}

void OSGViewWidget::_showModelByID(const std::string &modelID, int drawMode)
{
    if(!!m_envModelSwitch){
        osg::ref_ptr<osg::MatrixTransform> tempGroup = osg::ref_ptr<osg::MatrixTransform>(m_osgEnvModel->_getModelByID(modelID,QOSGEnvModel::DrawModeType(drawMode)));
        if(!!tempGroup){
            m_envModelSwitch->removeChild(tempGroup);
            m_envModelSwitch->setAllChildrenOn();
            m_osgSceneRoot->addChild(tempGroup);
        }
    }
}

void OSGViewWidget::_setSkyBox(const std::string &skyName)
{
    m_osgSkybox = new Skybox;
    _changeSkyBox(skyName);
    m_osgFigureRoot->addChild(m_osgSkybox);
}

void OSGViewWidget::_changeSkyBox(const std::string& bakName)
{

//    strBakPathNameR = QString(":/Skybox/picture/%1/right.jpg").arg(QString(bakName.c_str())).toStdString();
//    strBakPathNameL = QString(":/Skybox/picture/%1/left.jpg").arg(QString(bakName.c_str())).toStdString();
//    strBakPathNameD = QString(":/Skybox/picture/%1/down.jpg").arg(QString(bakName.c_str())).toStdString();
//    strBakPathNameT = QString(":/Skybox/picture/%1/top.jpg").arg(QString(bakName.c_str())).toStdString();
//    strBakPathNameB = QString(":/Skybox/picture/%1/back.jpg").arg(QString(bakName.c_str())).toStdString();
//    strBakPathNameF = QString(":/Skybox/picture/%1/front.jpg").arg(QString(bakName.c_str())).toStdString();
    QString appDir = QApplication::applicationDirPath();
    appDir+="/Skybox/picture/%s/right.jpg";
    std::string strBakPathNameR,strBakPathNameL,strBakPathNameD,strBakPathNameT,strBakPathNameB,strBakPathNameF;
    strBakPathNameR = str(boost::format(appDir.toStdString().c_str())%bakName);
    appDir = QApplication::applicationDirPath();
    appDir+="/Skybox/picture/%s/left.jpg";
    strBakPathNameL = str(boost::format(appDir.toStdString().c_str())%bakName);
    appDir = QApplication::applicationDirPath();
    appDir+="/Skybox/picture/%s/down.jpg";
    strBakPathNameD = str(boost::format(appDir.toStdString().c_str())%bakName);
    appDir = QApplication::applicationDirPath();
    appDir+="/Skybox/picture/%s/top.jpg";
    strBakPathNameT = str(boost::format(appDir.toStdString().c_str())%bakName);
    appDir = QApplication::applicationDirPath();
    appDir+="/Skybox/picture/%s/back.jpg";
    strBakPathNameB = str(boost::format(appDir.toStdString().c_str())%bakName);
    appDir = QApplication::applicationDirPath();
    appDir+="/Skybox/picture/%s/front.jpg";
    strBakPathNameF = str(boost::format(appDir.toStdString().c_str())%bakName);

    _SetTextureCubeMap(strBakPathNameR,strBakPathNameL,strBakPathNameD,strBakPathNameT,strBakPathNameB,strBakPathNameF);
}

void OSGViewWidget::_SetTextureCubeMap(const std::string& posx, const std::string& negx, const std::string& posy,
                                     const std::string& negy, const std::string& posz, const std::string& negz)
{
    m_osgSkybox->setTextureCubeMap(posx, negx, posy, negy, posz, negz);
}

void OSGViewWidget::_removeRemainData(const std::string & cloudID, int drawMode)//1 cloud
{
    m_osgEnvModel->_removeRemainData(cloudID,QOSGEnvModel::DrawModeType(drawMode));
}

void OSGViewWidget::_addRemainData(const std::string & cloudID, int drawMode)//1 cloud
{
    m_osgEnvModel->_addRemainData(cloudID,QOSGEnvModel::DrawModeType(drawMode));
}


void OSGViewWidget::_removeOneTypeModelData(int drawMode)
{
    m_osgEnvModel->_removeOneTypeModelData( QOSGEnvModel::DrawModeType(drawMode));
}

void OSGViewWidget::initializeGL()
{

}

void OSGViewWidget::paintGL()
{
    frame();
//    _SetViewport(this->width(),this->height(),METERSINUNIT);
//    bool lastColState = false;
//    m_collsionWorld->performDiscreteCollisionDetection();
//    m_collsionCheck->_excuteCollision(lastColState,m_collsionWorld);
}

void OSGViewWidget::resizeGL(int width, int height)
{
    m_updateCallBace->m_widHeight = height;
    m_updateCallBace->m_widWidget = width;
}

void OSGViewWidget::_loadCloudDataFromFile(QString fileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(fileName.mid(fileName.lastIndexOf(".")) == QString::fromStdString(".ply"))
    {
        if(pcl::io::loadPLYFile(fileName.toStdString() , *cloud)==-1)
        {
//            std::cout<<"read point cloud failed!!!"<<std::endl;
//            emit _showLogDialog(tr("(error):read point cloud failed!!!.\n"));
            return;
        }
    }else if(fileName.mid(fileName.lastIndexOf(".")) == QString::fromStdString(".pcd")){
        if(pcl::io::loadPCDFile(fileName.toStdString() , *cloud)==-1)
        {
//            std::cout<<"read point cloud failed!!!"<<std::endl;
//            emit _showLogDialog(tr("(error):read point cloud failed!!!.\n"));
            return;
        }
    }
    _addPointCloud(cloud,"temCloud",0,0,0);
    m_collsionCheck->_addStaicObjectForCollsion("temCloud",m_collsionWorld);
}

void OSGViewWidget::_addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string & cloudID, int r, int g, int b, osg::Vec3 normalVec3)
{
    _removeModelByID(cloudID,0);
    osg::ref_ptr<osg::MatrixTransform> pointCloudGroup = new osg::MatrixTransform;
    std::vector<QOSGOctreeBuilder::ElementInfo> globalElements;
    osg::BoundingBox globalBound;
    int cNum = cloud->size();
    int cellCountPoint = cNum * 0.0002;
    for (int i = 0 ; i < cNum ; i+=cellCountPoint)
    {
        osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        osg::ref_ptr<osg::Vec3Array> nors = new osg::Vec3Array;
        int j = i;
        for(;j<cNum && j<i+cellCountPoint;++j){
            coords->push_back(osg::Vec3(cloud->points[j].x , cloud->points[j].y , cloud->points[j].z)) ;
        }
        colors->push_back(osg::Vec4((float)r/255.0f, (float)g/255.0f, (float)b/255.0f,1.0f)) ;
        nors->push_back(normalVec3);
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setVertexArray(coords.get());
        geometry->setColorArray(colors.get());
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
        geometry->setNormalArray(nors.get());
        geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
        int endCnum = j-i;
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS , 0 , endCnum)) ; //
        osg::StateSet* state = geometry->getOrCreateStateSet();
        state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry.get());
        osg::ref_ptr<osg::Group> geodeGroup = new osg::Group;
        geodeGroup->addChild(geode);
        osg::BoundingSphere region = geode->getBound();
        osg::Vec3 pos = region.center();
        double radius = region.radius();
        osg::Vec3 min = pos - osg::Vec3(radius, radius, radius);
        osg::Vec3 max = pos + osg::Vec3(radius, radius, radius);
        osg::BoundingBox regionBox(min, max);
        globalBound.expandBy( regionBox );
        globalElements.push_back( QOSGOctreeBuilder::ElementInfo(geodeGroup, regionBox) );
    }
    QOSGOctreeBuilder octree;
    osg::ref_ptr<osg::Group> cloudGroup = octree._build( 0, globalBound, globalElements );
    pointCloudGroup->addChild(cloudGroup);
    pointCloudGroup->setName(cloudID);
    m_osgEnvModel->m_cloudGroups.insert(std::pair<std::string,osg::ref_ptr<osg::MatrixTransform> >(cloudID,pointCloudGroup));
    m_osgSceneRoot->addChild(pointCloudGroup);

}

void OSGViewWidget::_addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string &cloudID, osg::Vec3 normalVec3)
{
    _removeModelByID(cloudID,0);
    osg::ref_ptr<osg::MatrixTransform> pointCloudGroup = new osg::MatrixTransform;
    std::vector<QOSGOctreeBuilder::ElementInfo> globalElements;
    osg::BoundingBox globalBound;
    int cNum = cloud->size();
    int cellCountPoint = cNum * 0.0002;
    for (int i = 0 ; i < cNum ; i+=cellCountPoint)
    {
        osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        osg::ref_ptr<osg::Vec3Array> nors = new osg::Vec3Array;
        int j = i;
        for(;j<cNum && j<i+cellCountPoint;++j){
            coords->push_back(osg::Vec3(cloud->points[j].x , cloud->points[j].y , cloud->points[j].z)) ;
            uint32_t rgb_val_;
            memcpy(&rgb_val_, &(cloud->points[j].rgb), sizeof(uint32_t));
            uint32_t red,green,blue;
            blue=rgb_val_ & 0x000000ff;
            rgb_val_ = rgb_val_ >> 8;
            green=rgb_val_ & 0x000000ff;
            rgb_val_ = rgb_val_ >> 8;
            red=rgb_val_ & 0x000000ff;
            colors->push_back(osg::Vec4((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f)) ;
        }
        nors->push_back(normalVec3);
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setVertexArray(coords.get());
        geometry->setColorArray(colors.get());
        geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        geometry->setNormalArray(nors.get());
        geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
        int endCnum = j-i;
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS , 0 , endCnum)) ; //
        osg::StateSet* state = geometry->getOrCreateStateSet();
        state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry.get());
        osg::ref_ptr<osg::Group> geodeGroup = new osg::Group;
        geodeGroup->addChild(geode);
        osg::BoundingSphere region = geode->getBound();
        osg::Vec3 pos = region.center();
        double radius = region.radius();
        osg::Vec3 min = pos - osg::Vec3(radius, radius, radius);
        osg::Vec3 max = pos + osg::Vec3(radius, radius, radius);
        osg::BoundingBox regionBox(min, max);
        globalBound.expandBy( regionBox );
        globalElements.push_back( QOSGOctreeBuilder::ElementInfo(geodeGroup, regionBox) );
    }
    QOSGOctreeBuilder octree;
    osg::ref_ptr<osg::Group> cloudGroup = octree._build( 0, globalBound, globalElements );
    pointCloudGroup->addChild(cloudGroup);
    pointCloudGroup->setName(cloudID);
    m_osgEnvModel->m_cloudGroups.insert(std::pair<std::string,osg::ref_ptr<osg::MatrixTransform> >(cloudID,pointCloudGroup));
    m_osgSceneRoot->addChild(pointCloudGroup);
}

void OSGViewWidget::_addPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, const std::string & cloudID)
{
    _removeModelByID(cloudID,0);
    osg::ref_ptr<osg::MatrixTransform> pointCloudGroup = new osg::MatrixTransform;
    std::vector<QOSGOctreeBuilder::ElementInfo> globalElements;
    osg::BoundingBox globalBound;
    int cNum = cloud->size();
    int cellCountPoint = cNum * 0.0002;
    for (int i = 0 ; i < cNum ; i+=cellCountPoint)
    {
        osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        osg::ref_ptr<osg::Vec3Array> nors = new osg::Vec3Array;
        int j = i;
        for(;j<cNum && j<i+cellCountPoint;++j){
            coords->push_back(osg::Vec3(cloud->points[j].x , cloud->points[j].y , cloud->points[j].z)) ;
            nors->push_back(osg::Vec3(cloud->points[j].normal_x , cloud->points[j].normal_y , cloud->points[j].normal_z)) ;
            uint32_t rgb_val_;
            memcpy(&rgb_val_, &(cloud->points[j].rgb), sizeof(uint32_t));
            uint32_t red,green,blue;
            blue=rgb_val_ & 0x000000ff;
            rgb_val_ = rgb_val_ >> 8;
            green=rgb_val_ & 0x000000ff;
            rgb_val_ = rgb_val_ >> 8;
            red=rgb_val_ & 0x000000ff;
            colors->push_back(osg::Vec4((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f)) ;
        }
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setVertexArray(coords.get());
        geometry->setColorArray(colors.get());
        geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        geometry->setNormalArray(nors.get());
        geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        int endCnum = j-i;
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS , 0 , endCnum)) ; //
        osg::StateSet* state = geometry->getOrCreateStateSet();
        state->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry.get());
        osg::ref_ptr<osg::Group> geodeGroup = new osg::Group;
        geodeGroup->addChild(geode);
        osg::BoundingSphere region = geode->getBound();
        osg::Vec3 pos = region.center();
        double radius = region.radius();
        osg::Vec3 min = pos - osg::Vec3(radius, radius, radius);
        osg::Vec3 max = pos + osg::Vec3(radius, radius, radius);
        osg::BoundingBox regionBox(min, max);
        globalBound.expandBy( regionBox );
        globalElements.push_back( QOSGOctreeBuilder::ElementInfo(geodeGroup, regionBox) );
    }
    QOSGOctreeBuilder octree;
    osg::ref_ptr<osg::Group> cloudGroup = octree._build( 0, globalBound, globalElements );
    pointCloudGroup->addChild(cloudGroup);
    pointCloudGroup->setName(cloudID);
    m_osgEnvModel->m_cloudGroups.insert(std::pair<std::string,osg::ref_ptr<osg::MatrixTransform> >(cloudID,pointCloudGroup));
    m_osgSceneRoot->addChild(pointCloudGroup);
}



osg::Camera* OSGViewWidget::_CreateCamera( int x, int y, int w, int h, double metersinunit)
{
    osg::ref_ptr<osg::DisplaySettings> ds = osg::DisplaySettings::instance();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "";
    traits->windowDecoration = false;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = 2/*ds->getMultiSamples()*/;//_numMultiSamples osg default was 0
    traits->samples = 4/*ds->getNumMultiSamples()*/;//_numMultiSamples osg default was 0
    osg::ref_ptr<osg::Camera> camera(new osg::Camera());
    camera->setGraphicsContext(new osgQt::GraphicsWindowQt(traits.get()));
    camera->setClearColor(osg::Vec4(0.95, 0.95, 0.95, 1.0));
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    double fnear = METERSINUNIT/metersinunit;
    camera->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), fnear, 100.0/metersinunit);
    camera->setCullingMode(camera->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING); // need this for allowing small points with zero bunding voluem to be displayed correctly
    return camera.release();
}

//osg::Image *OSGViewWidget::_createImage(int width, int height, const osg::Vec3 &color)
//{

//}

QWidget *OSGViewWidget::_addViewWidget(osg::ref_ptr<osg::Camera> camera,osg::ref_ptr<osg::Camera> hudcamera)
{
    m_osgview->setCamera(camera.get());
    m_osghudview->setCamera( hudcamera.get() );
    m_osgview->addEventHandler(new osgViewer::StatsHandler);
    osgViewer::Viewer::Windows windows;
    this->getWindows(windows);
    m_osgCameraManipulator = new QOSGTrackball(windows);
    m_osgCameraManipulator->setWheelZoomFactor(0.2);
    m_osgview->setCameraManipulator(m_osgCameraManipulator.get());
    osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );
    addView(m_osgview);
    addView( m_osghudview.get() );
    m_osgCameraHUD = new osg::MatrixTransform();
    hudcamera->addChild( m_osgCameraHUD.get() );
    m_osgCameraHUD->setMatrix(osg::Matrix::identity());
    hudcamera->setGraphicsContext(gw);
    hudcamera->setViewport(0,0,gw->getTraits()->width, gw->getTraits()->height);
    return gw ? gw->getGLWidget() : NULL;
}

