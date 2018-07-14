#ifndef QOSGTRACKBALL_H
#define QOSGTRACKBALL_H
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
const osg::Node::NodeMask OSG_IS_PICKABLE_MASK = 0x2;

class QOSGTrackball: public osgGA::TrackballManipulator
{
public:
    QOSGTrackball(osgViewer::ViewerBase::Windows &windows);

    void _SetSeekMode(bool bInSeekMode);///< seek pick oject

    bool _InSeekMode() const ;

protected:
    class OpenRAVEAnimationData : public OrbitAnimationData {
    public:
        osg::Vec3d _eyemovement;
    };

    virtual void allocAnimationData();

    virtual bool performMovement();

    // make zooming faster
    bool performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy );
    bool performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy);
    bool performMovementLeftMouseButton( const double eventTimeDelta, const double dx, const double dy );
    void applyAnimationStep( const double currentProgress, const double prevProgress );
    /** Performs trackball rotation based on two points given, for example,
        by mouse pointer on the screen.

        Scale parameter is useful, for example, when manipulator is thrown.
        It scales the amount of rotation based, for example, on the current frame time.*/
    void _trackball( osg::Vec3d& axis, float& angle, float p1x, float p1y, float p2x, float p2y );
    void rotateTrackball( const float px0, const float py0,
                                            const float px1, const float py1, const float scale );
    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );
    bool _getPickCenter(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us, osg::Vec3d& resCenter, osg::Vec3d &resPickLine);
    bool setCenterByMousePointerIntersection( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );


    virtual bool seekToMousePointer( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

    virtual bool handleMouseDoubleClick( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

    virtual bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

    virtual bool handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us );

    virtual bool handleMouseWheel(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);
private:
    float m_old_dx,m_old_dy;
    bool m_bInSeekMode; ///< if true, in seek mode
    osgViewer::Viewer::Windows m_windows;
};

#endif // QOSGTRACKBALL_H
