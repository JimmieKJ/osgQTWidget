#include "qosgmainwindow.h"

#include <QDesktopWidget>
#include <QLayout>
//#include <eigen3/Eigen/Eigen>
//#include <osgGA/TrackballManipulator>
QOSGMainWindow::QOSGMainWindow(QWidget *parent) : QMainWindow(parent),m_leftSideBar(NULL)
{
    #if QT_VERSION >= 0x050000
        // Qt5 is currently crashing and reporting "Cannot make QOpenGLContext current in a different thread" when the viewer is run multi-threaded, this is regression from Qt4
        osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
    #else
        osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
    #endif
    m_viewWidget = new OSGViewWidget(this, Qt::Widget, threadingModel);
//
//    m_viewWidget->_loadFileNodeToSceneRoot(dataNode);
//    m_viewWidget->_addBoxEdge(20,20,20,"boxEdge",Eigen::Affine3f(),255,0,0);
    m_viewWidget->_repaintView();
//    m_viewWidget->_addGridFloor(30,2,2.0,"grid",Eigen::Affine3f::Identity());
//    m_viewWidget->_addBoxModel(2,2,0.2,"boxModel",Eigen::Affine3f::Identity(),0,255,0,false);
//    m_viewWidget->_addBoxModel(3,3,3,"boxModel1",Eigen::Affine3f::Identity(),255,0,0,false);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//    m_viewWidget->_loadFileNodeToSceneRoot(transform,"grasper1","/home/aqrose23/Pictures/simplegripperv2_handle.stl");
//    transform.translation() << 2.5, 0.0, 0.0;
//    m_viewWidget->_loadFileNodeToSceneRoot(transform,"grasper2","/home/aqrose23/Pictures/simplegripperv2_handle.stl");
//    transform.translation() << 5, 0.0, 0.0;
//    m_viewWidget->_loadFileNodeToSceneRoot(transform,"grasper3","/home/aqrose23/Pictures/simplegripperv2_handle.stl");
//    transform.translation() << 7.5, 0.0, 0.0;
//    m_viewWidget->_loadFileNodeToSceneRoot(transform,"grasper4","/home/aqrose23/Pictures/simplegripperv2_handle.stl");
//    transform.translation() << 10, 0.0, 0.0;
//    m_viewWidget->_loadFileNodeToSceneRoot(transform,"grasper5","/home/aqrose23/Pictures/simplegripperv2_handle.stl");
//    transform.translation() << 12.5, 0.0, 0.0;
    m_viewWidget->_loadFileNodeToSceneRoot(transform,"grasper","/home/aqrose23/Pictures/simplegripperv2_finger.stl");
    slot_loadPointCloudFromFile(true);
//    m_viewWidget->m_collsionCheck->_addStaicObjectForCollsion("grasper1",m_viewWidget->m_collsionWorld);
//    m_viewWidget->m_collsionCheck->_addStaicObjectForCollsion("grasper2",m_viewWidget->m_collsionWorld);
//    m_viewWidget->m_collsionCheck->_addStaicObjectForCollsion("grasper3",m_viewWidget->m_collsionWorld);
//    m_viewWidget->m_collsionCheck->_addStaicObjectForCollsion("grasper4",m_viewWidget->m_collsionWorld);
//    m_viewWidget->m_collsionCheck->_addStaicObjectForCollsion("grasper5",m_viewWidget->m_collsionWorld);

    m_viewWidget->setGeometry(0,0,500,400);
    QPushButton* createBox = _addButtonToWindow(this,100,700,100,30,"","button_createView1","createBox");
    connect(createBox,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateBoxOk(bool)));
    QPushButton* clear = _addButtonToWindow(this,200,700,100,30,"","button_createView2","clear");
    connect(clear,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateRemoveAllOk(bool)));
    QPushButton* createCylinder = _addButtonToWindow(this,100,740,100,30,"","button_createView3","createCylinder");
    connect(createCylinder,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateCylinderOk(bool)));
    QPushButton* createBoxEdge = _addButtonToWindow(this,200,740,100,30,"","button_createView4","createBoxEdge");
    connect(createBoxEdge,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateBoxEdgeOk(bool)));
    QPushButton* createFrame = _addButtonToWindow(this,100,780,100,30,"","button_createView5","createFrame");
    connect(createFrame,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateFrameOk(bool)));
    QPushButton* createGridFloor = _addButtonToWindow(this,200,780,100,30,"","button_createView6","createGridFloor");
    connect(createGridFloor,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateGridFloorOk(bool)));
    QPushButton* createPlane = _addButtonToWindow(this,100,820,100,30,"","button_createView7","createPlane");
    connect(createPlane,SIGNAL(clicked(bool)),this,SLOT(slot_calibratePlaneOk(bool)));
    QPushButton* createNewWidget = _addButtonToWindow(this,200,820,100,30,"","button_createView8","createNewWidget");
    connect(createNewWidget,SIGNAL(clicked(bool)),this,SLOT(slot_calibrateOSGWidgetOk(bool)));
    QPushButton* loadCloudFile = _addButtonToWindow(this,100,860,100,30,"","button_loadCloudFile","loadCloudFile");
    connect(loadCloudFile,SIGNAL(clicked(bool)),this,SLOT(slot_loadPointCloudFromFile(bool)));
    QPushButton* setRemainModel = _addButtonToWindow(this,200,860,100,30,"","button_setRemainModel","setRemainModel");
    connect(setRemainModel,SIGNAL(clicked(bool)),this,SLOT(slot_setRemainModel(bool)));
    QPushButton* hideMOdel = _addButtonToWindow(this,100,900,100,30,"","button_hideMOdel","hideMOdel");
    connect(hideMOdel,SIGNAL(clicked(bool)),this,SLOT(slot_hideMOdel(bool)));
    QPushButton* showModel = _addButtonToWindow(this,200,940,100,30,"","button_showModel","showModel");
    connect(showModel,SIGNAL(clicked(bool)),this,SLOT(slot_showModel(bool)));
    showMaximized();
    m_showhouseViewAct = new QAction(QIcon(":/images/startView_.png"),tr("start View"),this);
    connect(m_showhouseViewAct,SIGNAL(triggered(bool)),this,SLOT(slot_ShowHouseView(bool)));
    m_showxyViewAct = new QAction(tr("up View"),this);
    m_showxyViewAct->setIcon(QIcon(":/images/upView_.png"));
    connect(m_showxyViewAct,SIGNAL(triggered(bool)),this,SLOT(slot_ShowXyView(bool)));
    m_showxzViewAct = new QAction(tr("left View"),this);
    m_showxzViewAct->setIcon(QIcon(":/images/leftView_.png"));
    connect(m_showxzViewAct,SIGNAL(triggered(bool)),this,SLOT(slot_ShowXzView(bool)));
    m_showyzViewAct = new QAction(tr("Front view"),this);
    m_showyzViewAct->setIcon(QIcon(":/images/faceView_.png"));
    connect(m_showyzViewAct,SIGNAL(triggered(bool)),this,SLOT(slot_ShowYzView(bool)));

    _createLeftSideBar();
    m_viewWidget->_SetHome();
}

void QOSGMainWindow::slot_setRemainModel(bool flag)
{
    m_viewWidget->_addRemainData("boxModel",1);
}

void QOSGMainWindow::slot_hideMOdel(bool)
{
    m_viewWidget->_hideModelByID("grasper",QOSGEnvModel::ModelFromDisk);
}

void QOSGMainWindow::slot_showModel(bool)
{
    m_viewWidget->_showModelByID("grasper",QOSGEnvModel::ModelFromDisk);
}

void QOSGMainWindow::slot_loadPointCloudFromFile(bool flag)
{
    m_viewWidget->_loadCloudDataFromFile("/home/aqrose23/Downloads/data/rabbit_gra.pcd");
}

void QOSGMainWindow::slot_ShowHouseView(bool flag)
{
    m_viewWidget->_SetHome();
}

void QOSGMainWindow::slot_ShowXyView(bool flag)
{
    m_viewWidget->_ChangeViewToXY();
//    m_showxzViewAct->setChecked(false);
//    m_showyzViewAct->setChecked(false);
}

void QOSGMainWindow::slot_ShowXzView(bool flag)
{
    m_viewWidget->_ChangeViewToXZ();
//    m_showxyViewAct->setChecked(false);
//    m_showyzViewAct->setChecked(false);
}

void QOSGMainWindow::slot_ShowYzView(bool flag)
{
    m_viewWidget->_ChangeViewToYZ();
//    m_showxyViewAct->setChecked(false);
//    m_showxzViewAct->setChecked(false);
}

void QOSGMainWindow::slot_calibrateBoxOk(bool bClick)
{
    m_viewWidget->_addBoxModel(2,2,2,"boxModel",Eigen::Affine3f::Identity(),255,0,0,true);

}
void QOSGMainWindow::slot_calibrateRemoveAllOk(bool bClick)
{
    m_viewWidget->_removeAllData();
}

void QOSGMainWindow::slot_calibrateCylinderOk(bool bClick)
{
    m_viewWidget->_addCylinder(1,2,"cylider",Eigen::Affine3f::Identity(),0,255,0,false);
}

void QOSGMainWindow::slot_calibrateBoxEdgeOk(bool bClick)
{
    m_viewWidget->_addBoxEdge(2,3,3,"boxEdge",Eigen::Affine3f::Identity(),0,0,255);
}

void QOSGMainWindow::slot_calibratePlaneOk(bool bClick)
{
    m_viewWidget->_addPlane(1,1,"plane",Eigen::Affine3f::Identity(),255,255,255,true);
}

void QOSGMainWindow::slot_calibrateFrameOk(bool bClick)
{
    m_viewWidget->_addFrame(3,"frame",Eigen::Affine3f::Identity());
}

void QOSGMainWindow::slot_calibrateGridFloorOk(bool bClick)
{
    m_viewWidget->_addGridFloor(30,2,3.0,"gridFloor",Eigen::Affine3f::Identity());
}

void QOSGMainWindow::slot_calibrateOSGWidgetOk(bool bClick)
{
        _addViewWidget(900,0,500,400);
}
void QOSGMainWindow::_getScreenInfo()
{
    QDesktopWidget* desktopWidget = QApplication::desktop();
    QRect screenRect = desktopWidget->availableGeometry();
    m_screenWidth = screenRect.width();
    m_screenHeight = screenRect.height();
}

void QOSGMainWindow::_addViewWidget(int x,int y,int w,int h)
{
    #if QT_VERSION >= 0x050000
        // Qt5 is currently crashing and reporting "Cannot make QOpenGLContext current in a different thread" when the viewer is run multi-threaded, this is regression from Qt4
        osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
    #else
        osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::CullDrawThreadPerContext;
    #endif
    OSGViewWidget* widget = new OSGViewWidget(this, Qt::Widget, threadingModel);
    widget->_repaintView();
    widget->_addGridFloor(30,2,2.0,"grid",Eigen::Affine3f::Identity());
    widget->setGeometry(x,y,w,h);
    widget->_addCylinder(20,20,"cylinderModel",Eigen::Affine3f::Identity(),255,0,0);
    layout()->addWidget(widget);
}

void QOSGMainWindow::_createLeftSideBar()
{
    if(!!m_leftSideBar)
    {
        QObjectList objS = m_leftSideBar->children();
        for(int i=0;i<objS.length();++i){
            delete objS[i];
        }
        delete m_leftSideBar;
    }
    m_leftSideBar = new QtOSGLeftSideBar(m_viewWidget->height(),this);
    int x_l = m_viewWidget->geometry().x();
    int y_l = m_viewWidget->geometry().y();
    m_leftSideBar->setGeometry(x_l,y_l,30,m_viewWidget->height());
    m_leftSideBar->addAction(m_showhouseViewAct);
    m_leftSideBar->addAction(m_showxyViewAct);
    m_leftSideBar->addAction(m_showxzViewAct);
    m_leftSideBar->addAction(m_showyzViewAct);
//    m_leftSideBar->addAction(m_RotationAct);
//    m_leftSideBar->addAction(m_selectObjectAct);
//    m_leftSideBar->addAction(m_selectJointAct);
//    m_leftSideBar->addAction(m_selectTcpAct);
//    m_leftSideBar->addAction(m_movingEndingAct);
    m_leftSideBar->setWindowFlags(Qt::Widget | Qt::FramelessWindowHint | Qt::WindowSystemMenuHint | Qt::WindowStaysOnTopHint);
    m_leftSideBar->show();
}

QPushButton * QOSGMainWindow::_addButtonToWindow(QWidget* wid, int x, int y, int w, int h, QString iconStr, QString objName, QString textname)
{
    QPushButton *tbtn = new QPushButton(wid);
    tbtn->setGeometry(x,y,w,h);
    tbtn->setIcon(QIcon(iconStr));
    tbtn->setIconSize(QSize(w,h));
    tbtn->setObjectName(objName);
    tbtn->setText(textname);
    return tbtn;
}

//void QOSGMainWindow::_UpdateCameraTransform(float fTimeElapsed)
//{
//    // set the viewport size correctly so we can draw stuff on the hud using window coordinates
//    int width = centralWidget()->size().width();
//    int height = centralWidget()->size().height();
//    m_viewWidget->_SetViewport(width, height, 1.0);

////    m_Tcamera = aqroserave::GetRaveTransformFromMatrix(m_viewWidget->GetCameraManipulator()->getMatrix());
//    osg::ref_ptr<osgGA::TrackballManipulator> ptrackball = osg::dynamic_pointer_cast<osgGA::TrackballManipulator>(m_viewWidget->GetCameraManipulator());
//    if( !!ptrackball ) {
//        m_focalDistance = ptrackball->getDistance();
//    }
//    else {
//        m_focalDistance = 0;
//    }
//    if( fTimeElapsed > 0 ) {
//        // animate the camera if necessary
//        bool bTracking=false;
////        Transform tTrack;
////        KinBody::LinkPtr ptrackinglink = m_ptrackinglink;
////        if( !!ptrackinglink ) {
////            bTracking = true;
////            tTrack = ptrackinglink->GetTransform()*m_tTrackingLinkRelative;
////            //tTrack.trans = ptrackinglink->ComputeAABB().pos;
////        }
////        RobotBase::ManipulatorPtr ptrackingmanip=m_ptrackingmanip;
////        if( !!ptrackingmanip ) {
////            bTracking = true;
////            tTrack = ptrackingmanip->GetTransform();
////        }

//        if( bTracking ) {

////            RaveVector<float> vup(0,0,1); // up vector that camera should always be oriented to
////            if( !!m_penv->GetPhysicsEngine() ) {
////                Vector vgravity = m_penv->GetPhysicsEngine()->GetGravity();
////                if( vgravity.lengthsqr3() > g_fEpsilon ) {
////                    vup = -vgravity*(1.0/RaveSqrt(vgravity.lengthsqr3()));
////                }
////            }
//            RaveVector<float> vlookatdir = m_Tcamera.trans - tTrack.trans;
//            vlookatdir -= vup*vup.dot3(vlookatdir);
//            float flookatlen = sqrtf(vlookatdir.lengthsqr3());
//            vlookatdir = vlookatdir*cosf(m_fTrackAngleToUp) + flookatlen*sinf(m_fTrackAngleToUp)*vup; // flookatlen shouldn't change
//            if( flookatlen > g_fEpsilon ) {
//                vlookatdir *= 1/flookatlen;
//            }
//            else {
//                vlookatdir = Vector(1,0,0);
//            }
//            vup -= vlookatdir*vlookatdir.dot3(vup);
//            vup.normalize3();

//            float angle = normalizeAxisRotation(vup, m_Tcamera.rot).first;
//            RaveVector<float> vDestQuat = quatMultiply(quatFromAxisAngle(vup, -angle), quatRotateDirection(RaveVector<float>(0,1,0), vup));
//            // focal distance is the tracking radius. ie how far from the coord system camera shoud be
//            RaveVector<float> vDestPos = tTrack.trans + ExtractAxisFromQuat(vDestQuat,2)*m_focalDistance;
//            if(1) {
//                // PID animation
//                float pconst = 0.02;
//                float dconst = 0.2;
//                RaveVector<float> newtrans = m_Tcamera.trans + fTimeElapsed*m_tTrackingCameraVelocity.trans;
//                newtrans += pconst*(vDestPos - m_Tcamera.trans); // proportional term
//                newtrans -= dconst*m_tTrackingCameraVelocity.trans*fTimeElapsed; // derivative term (target is zero velocity)

//                pconst = 0.001;
//                dconst = 0.04;
//                RaveVector<float> newquat = m_Tcamera.rot + fTimeElapsed*m_tTrackingCameraVelocity.rot;
//                newquat += pconst*(vDestQuat - m_Tcamera.rot);
//                newquat -= dconst*m_tTrackingCameraVelocity.rot*fTimeElapsed;
//                newquat.normalize4();
//                // have to make sure newquat's y vector aligns with vup

//                m_tTrackingCameraVelocity.trans = (newtrans-m_Tcamera.trans)*(2/fTimeElapsed) - m_tTrackingCameraVelocity.trans;
//                m_tTrackingCameraVelocity.rot = (newquat-m_Tcamera.rot)*(2/fTimeElapsed) - m_tTrackingCameraVelocity.rot;
//                m_Tcamera.trans = newtrans;
//                m_Tcamera.rot = newquat;
//            }
//            else {
//                m_Tcamera.trans = vDestPos;
//                m_Tcamera.rot = vDestQuat;
//            }

//            _SetCameraTransform();
//        }
//    }

//    double fovy, aspectRatio, zNear, zFar;
//    m_viewWidget->GetCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
//    int camwidth = m_viewWidget->GetCamera()->getViewport()->width();
//    int camheight = m_viewWidget->GetCamera()->getViewport()->height();

//    m_camintrinsics.fy = 0.5*camheight/RaveTan(0.5f*fovy*M_PI/180.0);
//    m_camintrinsics.fx = m_camintrinsics.fy*float(camwidth)/(float)camheight/aspectRatio;
//    m_camintrinsics.cx = (float)camwidth/2;
//    m_camintrinsics.cy = (float)camheight/2;
//    m_camintrinsics.focal_length = zNear;
//    m_camintrinsics.distortion_model = "";
//}

//void QOSGMainWindow::_SetCameraTransform()
//{
//    osg::ref_ptr<osgGA::TrackballManipulator> ptrackball = osg::dynamic_pointer_cast<osgGA::TrackballManipulator>(m_osgViewWidget->_GetCameraManipulator());
//    if( !!ptrackball ) {
//        ptrackball->setDistance(m_focalDistance);
//    }

//    // has to come after setting distance because internally orbit manipulator uses the distance to deduct view center
//    m_viewWidget->_GetCameraManipulator()->setByMatrix((m_Tcamera);
//}
