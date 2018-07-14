#ifndef PARAMTRACKBALLDRAGGER_H
#define PARAMTRACKBALLDRAGGER_H

#include <QObject>
#include <osgManipulator/TrackballDragger>
#include <osgModeling/Curve>
#include <osgModeling/Loft>
class ParamTrackballDragger : public QObject,public osgManipulator::TrackballDragger
{
    Q_OBJECT
public:
    ParamTrackballDragger(QObject *parent = NULL);
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    virtual bool handle(const osgManipulator::PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

//    osg::Geometry* createCircleGeometry(float radius, unsigned int numSegments)
    osg::ref_ptr<osgModeling::Loft> createCircleGeometry(float radius, unsigned int numSegments,float SegmentsLen,osg::Vec3 &conCenter);

    virtual void setupDefaultGeometry();
    void _changeDraggersColor();
    void _Transform_Point(double out[4], const double m[16], const double in[4]);
    bool _handleRotate(const osg::ref_ptr<osgManipulator::RotateCylinderDragger> rotateDragger, const osgManipulator::PointerInfo& pointer, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    osg::Vec3d _WorldToScreen(osg::Camera *rootCamera, osg::Vec3 worldpoint);
signals:

public slots:

public:
    bool m_bSendLoadIKFastSolver;
    std::string m_draggerName;
    osg::Matrix m_iniDraggerMatrixInvert;
    bool m_bControllEndEffector;
    osg::Matrix m_prevDraggerMatrixInvet;

    osg::Vec3f m_screenPoint;
    int m_windowWithPix; //
    int m_windowHeightPix;//
    osg::ref_ptr<osg::MatrixTransform> m_ControllItem;
    double m_oldZ;

    float m_eaX;//record for last mouse posi x
    float m_eaY;//record for last mouse posi y

    osg::Vec3d  m_prevWorldProjPt;
    osg::Matrix m_startLocalToWorld, m_startWorldToLocal;
    osg::Quat   m_prevRotation;
    osg::ref_ptr<osgManipulator::CylinderPlaneProjector> m_projector;

    osg::Camera *m_rootCamera;

    bool m_bClockWise;//to change rotate's angle value bigger or smaller
    double m_lastAngleValue;
    double m_resAngleValue;
    osg::Vec3d m_draggerAxis;
};

#endif // PARAMTRACKBALLDRAGGER_H
