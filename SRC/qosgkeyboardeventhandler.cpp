#include "qosgkeyboardeventhandler.h"

QOSGKeyboardEventHandler::QOSGKeyboardEventHandler(const boost::function<bool (const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &)> &onKeyDown, const boost::function<bool (const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &)> &onKeyUp):
    _onKeyDown(onKeyDown),
    _onKeyUp(onKeyUp)
{

}

bool QOSGKeyboardEventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    switch(ea.getEventType())
    {
    case (osgGA::GUIEventAdapter::KEYDOWN): {
        return _onKeyDown(ea, aa);
    }
    case (osgGA::GUIEventAdapter::KEYUP):{
        return _onKeyUp(ea,aa);
    }
    default:
        return false;
    }
    return false;
}
