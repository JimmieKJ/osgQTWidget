#ifndef PARAMAXISDRAGGER_H
#define PARAMAXISDRAGGER_H

#include <QObject>
#include <osgManipulator/TranslateAxisDragger>
#include <osgModeling/Curve>
#include <osgModeling/Loft>
class ParamAxisDragger : public QObject,public osgManipulator::TranslateAxisDragger
{
    Q_OBJECT
public:
    ParamAxisDragger(QObject *parent = NULL);
    class ForceCullCallback : public osg::Drawable::CullCallback
    {
        public:
            virtual bool cull(osg::NodeVisitor*, osg::Drawable*, osg::State*) const
            {
                return true;
            }
    };

    virtual void setDrawableToAlwaysCull(osg::Drawable& drawable);

    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    virtual bool handle(const osgManipulator::PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    virtual void setupDefaultGeometry();
    void _changeDraggersColor();
//    void _ControlEndEffector();
    int _getActiveDragger();
    void _Transform_Point(double out[4], const double m[16], const double in[4]);
    osg::Vec3d _WorldToScreen(osg::Camera *rootCamera, osg::Vec3 worldpoint);
signals:

public slots:

public:
    int m_draggerActiveIndex;
    bool m_bSendLoadIKFastSolver;
    std::string m_draggerName;
    osg::Matrix m_iniDraggerMatrixInvert;
    bool m_bControllEndEffector;
    osg::Matrix m_prevDraggerMatrixInvet;
    osg::Vec3f m_screenPoint;
    int m_windowWithPix; //
    int m_windowHeightPix;//
    osg::Vec3f m_centerPointToWindowPosi;
    float m_eaX;
    float m_eaY;
};

#endif // PARAMAXISDRAGGER_H
