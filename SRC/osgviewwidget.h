#ifndef OSGVIEWWIDGET_H
#define OSGVIEWWIDGET_H

#include "qosgtrackball.h"
#include "qosgenvmodel.h"
#include "osgpick.h"
#include "osgskybox.h"
#include "qosgkeyboardeventhandler.h"
#include "qosgselecteditemdraggercallback.h"
#include "qosgupdatecallbackcheckcollsion.h"
#include "qosgoctreebuilder.h"
#include <QObject>
#include <QTimer>
#include <QMetaEnum>
#include <osgViewer/CompositeViewer>
#include <osgQt/GraphicsWindowQt>
#include <osg/Group>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/BlendEquation>
#include <osg/Switch>
#include <osgManipulator/Dragger>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>
class OSGViewWidget :  public QGLWidget,public osgViewer::CompositeViewer
{
    Q_OBJECT
public:


    OSGViewWidget(QWidget *parent = NULL, Qt::WindowFlags f = 0, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::CompositeViewer::SingleThreaded);
    void _SetViewport(int width, int height, double metersinunit);
//    osg::ref_ptr<osgGA::CameraManipulator> _GetCameraManipulator();
    /// add root node to scene
    void _repaintView();

    /// add point cloud with x, y, z, color and normal value
    void _addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& cloudID, int r=255, int g=255, int b=255,osg::Vec3 normalVec3=osg::Vec3(0,0,1));

    /// add point cloud with x, y, z, normal value
    void _addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,const std::string& cloudID,osg::Vec3 normalVec3=osg::Vec3(0,0,1));

    /// add point cloud only x,y,z value
    void _addPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, const std::string &cloudID);

    /// remove some model by id
    void _removeModelByID(const std::string & modelID, int drawMode);

    /// back to original location
    void _SetHome();

    /// add 3D MODEL from disk
    void _loadFileNodeToSceneRoot(const Eigen::Affine3f &transfer,const std::string& modelId,const std::string& nodeName);

    /// remove one type(PointCloud or BoxModel or CyliderModel or PlaneModel or BoxEdgeModel or FrameModel or GridModel) data from scene
    void _removeOneTypeModelData(int drawMode);

    /// make this model(by id) cannot be remove
    void _addRemainData(const std::string & cloudID, int drawMode);

    /// remove this model(by id) from remain data array,so that can be remove
    void _removeRemainData(const std::string & cloudID,int drawMode);

    /// remove all model except remain data
    void _removeAllData();

    /// add box model
    void _addBoxModel(double length, double width, double height, const std::string &boxID, const Eigen::Affine3f &transfer, int r = 255, int g=255, int b=255, bool bTransparent=false);

    /// add cylinder model
    void _addCylinder(double length,double radius,const std::string &cyLinderID,const Eigen::Affine3f & transfer,int r = 255,int g=255,int b=255,bool bTransparent=false);

    /// add plane model
    void _addPlane(double length, double width, const std::string &PlaneID, const Eigen::Affine3f &transfer, int r, int g, int b,bool bTransparent=false);

    /// add box edge model
    void _addBoxEdge(double length,double width,double height,const std::string& id,const Eigen::Affine3f& transfer ,int r = 255,int g=255,int b=255);

    /// add Coordinate system model
    void _addFrame(double length,const std::string& id,const Eigen::Affine3f& transfer);

    /// add floor mesh model
    void _addGridFloor(int netWidth, int netResolution, double gridWidth, const std::string &id, const Eigen::Affine3f &transfer);

    /// hide one model by id
    void _hideModelByID(const std::string& modelID, int drawMode);

    /// show one model by id(this model was hiden)
    void _showModelByID(const std::string& modelID, int drawMode);

    void _setSkyBox(const std::string& skyName);

    void _ChangeViewToXZ();

    void _ChangeViewToXY();

    void _ChangeViewToYZ();

    void _loadCloudDataFromFile(QString fileName);
protected:
    void initializeGL();
signals:

public slots:
    void paintGL();
    void resizeGL(int width, int height);
private:
    void _initOsgViewWidget();
    bool _HandleOSGKeyDown(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);
    bool _HandleOSGKeyUp(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
    QWidget *_addViewWidget(osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osg::Camera> hudcamera);
    osg::Camera *_CreateCamera( int x, int y, int w, int h, double metersinunit);
//    osg::Image* _createImage(int width, int height, const osg::Vec3 &color );
    void _changeSkyBox(const std::string& bakName);
    osg::Matrix _getMatrixFromAffine3f(const Eigen::Affine3f &transfer);
    void _createFrame();
    osg::Group *_CreateOSGXYZAxes(double len, double axisthickness);
    void _SetTextureCubeMap(const std::string& posx, const std::string& negx, const std::string& posy,
                                         const std::string& negy, const std::string& posz, const std::string& negz);
//    void _PrintMatrix(const osg::Matrix& m);
    void _HandleRayPick(osg::Node *node, int buttonPressed, int modkeymask);
    void _pickDraggerFunc(int buttonPressed, const osgGA::GUIEventAdapter &ea, bool bMouseBtnPush, bool bMouseBtnRelease);
    void _SelectOSGLink(osg::ref_ptr<osg::Node> node, int modkeymask);
    osg::Camera *_CreateHUDCamera( int x, int y, int w, int h, double metersinunit);
    void _SetCameraTransform(const osg::Matrix& matCamera);
    void _addDraggerToObject();
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _createDragger();
    void _ClearDragger();
public:
    btCollisionWorld* m_collsionWorld;
    QOSGCollsionCheck* m_collsionCheck;
protected:
    QTimer m_timer;
private:
    osg::ref_ptr<osgViewer::View> m_osgview;
    osg::ref_ptr<osgViewer::View> m_osghudview;
    osg::ref_ptr<QOSGTrackball> m_osgCameraManipulator;
    osg::ref_ptr<osgGA::GUIEventHandler> m_keyhandler;
    osg::ref_ptr<osg::Group> m_osgSceneRoot;
    QOSGEnvModel *m_osgEnvModel;
    osg::ref_ptr<osg::MatrixTransform> m_osgWorldAxis; ///< the node that draws the rgb axes on the lower right corner
    osg::ref_ptr<osg::MatrixTransform> m_osgWorldCenterAxis; ///< this node that draw world center rgb axes
    osg::ref_ptr<osg::Switch> m_osgWorldCenterAxisSwitch;///< this node controll "m_osgWorldCenterAxis" show or not
    osg::ref_ptr<osg::Switch> m_envModelSwitch;
    osg::ref_ptr<OSGPickHandler> m_picker;
    bool m_bIsSelectiveActive;
    osg::ref_ptr<osg::MatrixTransform> m_selectedItem;
    osg::ref_ptr<Skybox> m_osgSkybox;
    osg::ref_ptr<osg::Group> m_osgFigureRoot;
    osg::ref_ptr<osg::MatrixTransform> m_osgCameraHUD; ///< MatrixTransform node that gets displayed in the heads up display
    osg::ref_ptr<osg::MatrixTransform> m_draggerMatrix;
    osg::ref_ptr<osg::Group> m_osgSelectedNodeByDragger;
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > m_draggers;
    osg::ref_ptr<osg::Group> m_osgDraggerRoot;
    QOSGUpdateCallbackCheckCollsion *m_updateCallBace;
};

#endif // OSGVIEWWIDGET_H
