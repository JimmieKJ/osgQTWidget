#include "qosgtrackball.h"
#include <assert.h>
#include <osgViewer/GraphicsWindow>
QOSGTrackball::QOSGTrackball(osgViewer::ViewerBase::Windows& windows)
{
    m_old_dx = 0.0f;
    m_old_dy = 0.0f;
    m_bInSeekMode = false;
    m_windows = windows;
}

void QOSGTrackball::_SetSeekMode(bool bInSeekMode)
{
    m_bInSeekMode = bInSeekMode;
//    if( m_bInSeekMode ) {
        osgViewer::GraphicsWindow::MouseCursor cursortype = m_bInSeekMode ? osgViewer::GraphicsWindow::CrosshairCursor : osgViewer::GraphicsWindow::LeftArrowCursor;
        for(osgViewer::Viewer::Windows::iterator itr = m_windows.begin(); itr != m_windows.end(); ++itr) {
            (*itr)->setCursor(cursortype);
        }
//    }
}

bool QOSGTrackball::_InSeekMode() const
{
    return m_bInSeekMode;
}

void QOSGTrackball::allocAnimationData()
{
    _animationData = new OpenRAVEAnimationData();
}

bool QOSGTrackball::performMovement()
{
    // return if less then two events have been added
    if( _ga_t0.get() == NULL || _ga_t1.get() == NULL ) {
        return false;
    }
    // get delta time
    double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
    if( eventTimeDelta < 0. ) {
        OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
        eventTimeDelta = 0.;
    }

    // get deltaX and deltaY
    float dx = (_ga_t0->getXnormalized() - _ga_t1->getXnormalized());
    float dy = (_ga_t0->getYnormalized() - _ga_t1->getYnormalized());


    // return if there is no movement.
    if( dx == 0. && dy == 0. ) {
        return false;
    }else if(m_old_dx == dx && m_old_dy == dy){
        return false;
    }

//    if(!!_pviewer->m_draggerMatrix)
//        _pviewer->m_draggerMatrix->setMatrix(osg::Matrix::identity());
    m_old_dx = dx;
    m_old_dy = dy;
    unsigned int buttonMask = _ga_t1->getButtonMask();
//    if((buttonMask & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) && (_ga_t1->getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL)){
//        _pviewer->_setDraggerCircleWidth(dx);
//        return true;
//    }
//    else if( (buttonMask & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) && (_ga_t1->getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_SHIFT) ) {
//        return performMovementMiddleMouseButton( eventTimeDelta, dx, dy );
    if((buttonMask & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) && (_ga_t1->getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL)){
        return true;
    }else if(buttonMask & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON){
        return true;
    }else if(buttonMask & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON){

        return performMovementLeftMouseButton(eventTimeDelta, dx, dy) ;
    }

    return osgGA::TrackballManipulator::performMovement();
}

bool QOSGTrackball::performMovementRightMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
    return osgGA::TrackballManipulator::performMovementRightMouseButton(eventTimeDelta, dx, dy*4);
}

bool QOSGTrackball::performMovementMiddleMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
    return osgGA::TrackballManipulator::performMovementMiddleMouseButton(eventTimeDelta, dx*2.5, dy*1.3);
}

bool QOSGTrackball::performMovementLeftMouseButton(const double eventTimeDelta, const double dx, const double dy)
{
    // rotate camera
    if( getVerticalAxisFixed() )
        rotateWithFixedVertical( dx, dy );
    else
        rotateTrackball( _ga_t0->getXnormalized(), _ga_t0->getYnormalized(),
                         _ga_t1->getXnormalized(), _ga_t1->getYnormalized(),
                         getThrowScale( eventTimeDelta ) );
    return true;
}

void QOSGTrackball::applyAnimationStep(const double currentProgress, const double prevProgress)
{
    OpenRAVEAnimationData *ad = dynamic_cast< OpenRAVEAnimationData* >( _animationData.get() );
    assert( ad );

    // compute new center
    osg::Vec3d prevCenter, prevEye, prevUp;
    getTransformation( prevEye, prevCenter, prevUp );
    osg::Vec3d newCenter = osg::Vec3d(prevCenter) + (ad->_movement * (currentProgress - prevProgress));
    osg::Vec3d newEye = osg::Vec3d(prevEye) + (ad->_eyemovement * (currentProgress - prevProgress));

    // fix vertical axis
    if( getVerticalAxisFixed() )
    {
        osg::CoordinateFrame coordinateFrame = getCoordinateFrame( newCenter );
        osg::Vec3d localUp = getUpVector( coordinateFrame );

        fixVerticalAxis( newCenter - newEye, prevUp, prevUp, localUp, false );
    }

    // apply new transformation
    setTransformation( newEye, newCenter, prevUp );
}

void QOSGTrackball::_trackball(osg::Vec3d &axis, float &angle, float p1x, float p1y, float p2x, float p2y)
{
    /*
        * First, figure out z-coordinates for projection of P1 and P2 to
        * deformed sphere
        */

    osg::Matrixd rotation_matrix(_rotation);

    osg::Vec3d uv = osg::Vec3d(0.0f,1.0f,0.0f)*rotation_matrix;
    osg::Vec3d sv = osg::Vec3d(1.0f,0.0f,0.0f)*rotation_matrix;
    osg::Vec3d lv = osg::Vec3d(0.0f,0.0f,-1.0f)*rotation_matrix;
    osg::Vec3d p1 = sv * p1x + uv * p1y - lv * tb_project_to_sphere(_trackballSize, p1x, p1y);
    osg::Vec3d p2 = sv * p2x + uv * p2y - lv * tb_project_to_sphere(_trackballSize, p2x, p2y);
    /*
        *  Now, we want the cross product of P1 and P2
        */

    axis = p2^p1;
    axis.normalize();

    /*
        *  Figure out how much to rotate around that axis.
        */
    float t = (p2 - p1).length() / (2.0 * _trackballSize);

    /*
        * Avoid problems with out-of-control values...
        */
    if (t > 1.0) t = 1.0;
    if (t < -1.0) t = -1.0;
    angle = osg::inRadians(asin(t));
}

void QOSGTrackball::rotateTrackball(const float px0, const float py0, const float px1, const float py1, const float scale)
{
    const osg::Vec3d zero(0.0,0.0,0.0);
    osg::Vec3d axis;
    float angle;

//    trackball( axis, angle, px0 + (px1-px0)*scale, py0 + (py1-py0)*scale, px0, py0 );
        trackball( axis, angle, px1, py1, px0, py0 );

    osg::Quat new_rotate;

    new_rotate.makeRotate( angle, axis );

    _rotation = _rotation * new_rotate;
}

bool QOSGTrackball::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    switch( ea.getEventType() )
    {
//    case osgGA::GUIEventAdapter::DOUBLECLICK:
//        return handleMouseDoubleClick( ea, us );
    default:
        break;
    }
    return osgGA::TrackballManipulator::handle(ea, us);
}

bool QOSGTrackball::_getPickCenter(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us, osg::Vec3d &resCenter, osg::Vec3d &resPickLine)
{
    osg::ref_ptr< osg::View> view = us.asView();
     if( !view )
         return false;

     osg::Camera *camera = view->getCamera();
     if( !camera )
         return false;

     // prepare variables
     float x = ( ea.getX() - ea.getXmin() ) / ( ea.getXmax() - ea.getXmin() );
     float y = ( ea.getY() - ea.getYmin() ) / ( ea.getYmax() - ea.getYmin() );
     osgUtil::LineSegmentIntersector::CoordinateFrame cf;
     osg::Viewport *vp = camera->getViewport();
     if( vp ) {
         cf =osgUtil::Intersector::WINDOW;
         x *= vp->width();
         y *= vp->height();
     } else
         cf = osgUtil::Intersector::PROJECTION;

     // perform intersection computation
     osg::ref_ptr<osgUtil::LineSegmentIntersector > picker = new osgUtil::LineSegmentIntersector( cf, x, y );

     osgUtil::IntersectionVisitor iv( picker.get() );
     iv.setTraversalMask(OSG_IS_PICKABLE_MASK); // different from OSG implementation!
     camera->accept( iv );

     // return on no intersections
     if( !picker->containsIntersections() )
         return false;

     // get all intersections
     osgUtil::LineSegmentIntersector::Intersections& intersections = picker->getIntersections();
     resPickLine = picker->getEnd() - picker->getStart();
     // new center
     resCenter = (*intersections.begin()).getWorldIntersectPoint();
     return true;
}

bool QOSGTrackball::setCenterByMousePointerIntersection(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    // get current transformation
    osg::Vec3d eye, oldCenter, up;
    getTransformation( eye, oldCenter, up );

    osg::Vec3d newCenter, pickLine;
    if(!_getPickCenter(ea,us,newCenter,pickLine))
        return false;
    // new center
//    osg::Vec3d newCenter = (*intersections.begin()).getWorldIntersectPoint();

    // make vertical axis correction
    if( getVerticalAxisFixed() )
    {

        osg::CoordinateFrame coordinateFrame = getCoordinateFrame( newCenter );
        osg::Vec3d localUp = getUpVector( coordinateFrame );

        fixVerticalAxis( newCenter - eye, up, up, localUp, true );

    }

    // set the new center
    setTransformation( eye, newCenter, up );


    // warp pointer
    // note: this works for me on standard camera on GraphicsWindowEmbedded and Qt,
    //       while it was necessary to implement requestWarpPointer like follows:
    //
    // void QOSGWidget::requestWarpPointer( float x, float y )
    // {
    //    osgViewer::Viewer::requestWarpPointer( x, y );
    //    QCursor::setPos( this->mapToGlobal( QPoint( int( x+.5f ), int( y+.5f ) ) ) );
    // }
    //
    // Additions of .5f are just for the purpose of rounding.
    centerMousePointer( ea, us );

    return true;
}

bool QOSGTrackball::seekToMousePointer(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    _SetSeekMode(false);
    if( !isAnimating() ) {
        setAnimationTime(0.25);

        // get current transformation
        osg::Vec3d prevCenter, prevEye, prevUp;
        getTransformation( prevEye, prevCenter, prevUp );

        // center by mouse intersection
        if( !setCenterByMousePointerIntersection( ea, us ) ) {
            return false;
        }

        OpenRAVEAnimationData *ad = dynamic_cast< OpenRAVEAnimationData*>( _animationData.get() );
//        BOOST_ASSERT( !!ad );

        // setup animation data and restore original transformation
        ad->start( osg::Vec3d(_center) - prevCenter, ea.getTime() );
        ad->_eyemovement = (osg::Vec3d(_center) - prevEye)*0.5;
        setTransformation( prevEye, prevCenter, prevUp );
        return true;
    }
    return false;
}

bool QOSGTrackball::handleMouseDoubleClick(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    return true;
}

bool QOSGTrackball::handleMousePush(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    if( m_bInSeekMode ) {
        if (seekToMousePointer(ea, us)) {
            return true;
        }
    }
    return osgGA::TrackballManipulator::handleMousePush(ea, us);
}

bool QOSGTrackball::handleKeyDown(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
    int key = ea.getKey();
    int modkeymask = ea.getModKeyMask();
    switch(key) {
    case osgGA::GUIEventAdapter::KEY_Left:
    case osgGA::GUIEventAdapter::KEY_Right:
    case osgGA::GUIEventAdapter::KEY_Up:
    case osgGA::GUIEventAdapter::KEY_Down: {
        osg::Matrixd m = getMatrix();
        osg::Vec3d center = getCenter();
        osg::Vec3d dir;
        if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_SHIFT) ) {
            if( key == osgGA::GUIEventAdapter::KEY_Up ) {
                dir = osg::Vec3d(-m(2,0), -m(2,1), -m(2,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Down ) {
                dir = osg::Vec3d(m(2,0), m(2,1), m(2,2));
            }

        }
        else {
            if( key == osgGA::GUIEventAdapter::KEY_Left ) {
                dir = osg::Vec3d(-m(0,0), -m(0,1), -m(0,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Right ) {
                dir = osg::Vec3d(m(0,0), m(0,1), m(0,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Down ) {
                dir = osg::Vec3d(-m(1,0), -m(1,1), -m(1,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Up ) {
                dir = osg::Vec3d(m(1,0), m(1,1), m(1,2));
            }
        }
        setCenter(center + dir*getDistance()*0.05);
        return true;
    }
    }

    return osgGA::TrackballManipulator::handleKeyDown(ea, us);
}

bool QOSGTrackball::handleMouseWheel(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
{
     return osgGA::TrackballManipulator::handleMouseWheel(ea,us);
}
