// -*- coding: utf-8 -*-

#include "osgpick.h"

#include <osgUtil/LineSegmentIntersector>
#include <iostream>

OSGPickHandler::OSGPickHandler(const HandleRayPickFn& handleRayPickFn, const DragFn& dragfn) :
    _handleRayPickFn(handleRayPickFn),
    _dragfn(dragfn),
    _bDoPickCallOnButtonRelease(false),
    iPushBtn(-1),
    m_bMouseBtnPush(false),
    m_bRightCtrl(false)
{
}

OSGPickHandler::~OSGPickHandler()
{
}

bool OSGPickHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    osg::ref_ptr<osgViewer::View> view(dynamic_cast<osgViewer::View*>(&aa));
    switch(ea.getEventType())
    {
    case osgGA::GUIEventAdapter::DOUBLECLICK:
    {
        if(m_bRightCtrl)
            return false;
        if (!!view) {
            _Pick(view, ea, 2);
        }
        return false;
    }
    case osgGA::GUIEventAdapter::RELEASE:
    {
        if( (ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) && _bDoPickCallOnButtonRelease ) {
            _bDoPickCallOnButtonRelease = false;
            if (!!view) {
                _Pick(view, ea, 1);
            }
        }else if((ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)){
            _dragfn(iPushBtn,ea,m_bMouseBtnPush,true);
        }
        m_bMouseBtnPush = false;
        return false;
    }
    case osgGA::GUIEventAdapter::PUSH:
        if( ea.getButton() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) {
            if(m_bRightCtrl)
                return false;
            _bDoPickCallOnButtonRelease = true;
            iPushBtn = 0;
        } else if( ea.getButton() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON )
        {
            _bDoPickCallOnButtonRelease = false; // mouse moved, so cancel any button presses
            iPushBtn = 1;
            if(m_bRightCtrl)
                m_bRightCtrl = false;
            if (!!view && (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
                _Pick(view, ea, 3);
                m_bRightCtrl = true;
            }
        }
        else {
            if(m_bRightCtrl)
                return false;
            _bDoPickCallOnButtonRelease = false;
            iPushBtn = -1;
        }
        return false;
    case osgGA::GUIEventAdapter::MOVE: {
        if(m_bRightCtrl)
            return false;
        if (!!view) {
            _Pick(view, ea, 0);
        }
        return false;
    }
    case osgGA::GUIEventAdapter::DRAG:{
        if(m_bRightCtrl)
            return false;
        _bDoPickCallOnButtonRelease = false; // mouse moved, so cancel any button presses
        if( !!_dragfn) {
            _dragfn(iPushBtn,ea,m_bMouseBtnPush,false);
            m_bMouseBtnPush = true;
        }
        return false;
    }
    default:
        return false;
    }
}

void OSGPickHandler::_Pick(osg::ref_ptr<osgViewer::View> view, const osgGA::GUIEventAdapter& ea, int buttonPressed)
{
    if( !_handleRayPickFn ) {
        return;
    }

    try {
        float x = ea.getX();
        float y = ea.getY();
        osgUtil::LineSegmentIntersector::Intersections intersections;
        if (view->computeIntersections(x,y,intersections)) {
            for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin(); hitr != intersections.end(); ++hitr) {
                if (!hitr->nodePath.empty() ) {
                    // if any node in the path has userdata that casts to OSGItemUserData, then we have hit a real item and should call _handleRayPickFn
                    for(int i_n=0;i_n<hitr->nodePath.size();++i_n){//std::vector< Node* >
                        osg::Node* itnode = hitr->nodePath.at(i_n);
                        if( (itnode)->getName().size()>0 ) {
//                            std::cout<<(itnode)->getName()<<std::endl;
//                            OSGItemUserData* pdata = dynamic_cast<OSGItemUserData*>((itnode)->getUserData());
                            /*if( !!pdata )*/ {
                                _handleRayPickFn(itnode, buttonPressed, ea.getModKeyMask());
                                return;
                            }
                        }
                    }
                }
            }
        }
        // if still here, then no intersection
        _handleRayPickFn(NULL, buttonPressed, ea.getModKeyMask());
    }
    catch(const std::exception& ex) {
        OSG_WARN << "RAVELOG_WARN_FORMAT: "<<"exception in osg picker:" << ex.what() << std::endl;
//        RAVELOG_WARN_FORMAT("exception in osg picker: %s", ex.what());
    }
}
