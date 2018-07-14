// -*- coding: utf-8 -*-

#ifndef OPENRAVE_QTOSG_PICK_H_
#define OPENRAVE_QTOSG_PICK_H_

#include <sstream>
#include <osg/Group>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>
#include <boost/function.hpp>
//const osg::Node::NodeMask OSG_IS_PICKABLE_MASK = 0x2;



/// \brief class to handle events with a pick
class OSGPickHandler : public osgGA::GUIEventHandler
{
public:
    /// select(node, modkeymask) where node is the ray-picked node, and modkeymask is the modifier key mask currently pressed
    typedef boost::function<void (osg::Node * , int, int)> HandleRayPickFn;
    typedef boost::function<void(int, const osgGA::GUIEventAdapter&,bool,bool)> DragFn;

    OSGPickHandler(const HandleRayPickFn& handleRayPickFn=HandleRayPickFn(), const DragFn& dragfn=DragFn());
    virtual ~OSGPickHandler();

    /// \brief override from base class
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    /// \brief Active joint selection
    //void ActivateSelection(bool active);
    //bool IsSelectionActive() const { return _select; }

protected:
    virtual void _Pick(osg::ref_ptr<osgViewer::View> view, const osgGA::GUIEventAdapter& ea, int buttonPressed);
    HandleRayPickFn _handleRayPickFn;
    DragFn _dragfn;
    //bool _select; ///< if true, then will call the _selectLinkFn with the raypicked node
    bool _bDoPickCallOnButtonRelease; ///< if true, then on button release can call _Pick
    int iPushBtn;
    bool m_bMouseBtnPush;
    bool m_bRightCtrl;
};



#endif /* OSGPICK_H_ */
