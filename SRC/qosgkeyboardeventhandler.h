#ifndef QOSGKEYBOARDEVENTHANDLER_H
#define QOSGKEYBOARDEVENTHANDLER_H
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>
#include <boost/function.hpp>
class QOSGKeyboardEventHandler: public osgGA::GUIEventHandler
{
public:
    QOSGKeyboardEventHandler(const boost::function<bool(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)>& onKeyDown,const boost::function<bool(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)>& onKeyUp);

    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
private:
    boost::function<bool(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)> _onKeyDown;
    boost::function<bool(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)> _onKeyUp;
};

#endif // QOSGKEYBOARDEVENTHANDLER_H
