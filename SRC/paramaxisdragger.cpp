#include "paramaxisdragger.h"

#include <osgViewer/View>

#define M_M(row,col) m[col * 4 + row]
ParamAxisDragger::ParamAxisDragger(QObject *parent) : QObject(parent)
{

}

void ParamAxisDragger::setDrawableToAlwaysCull(osg::Drawable &drawable)
{
    {
        osg::ref_ptr<ForceCullCallback> cullCB = new ForceCullCallback;
        drawable.setCullCallback (cullCB);
    }
}

bool ParamAxisDragger::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    {
        if (ea.getHandled()) return false;

        osg::ref_ptr<osgViewer::View> view = dynamic_cast<osgViewer::View*>(&aa);
        if (!view) return false;

        bool handled = false;

        bool activationPermitted = true;
        if (_activationModKeyMask!=0 || _activationMouseButtonMask!=0 || _activationKeyEvent!=0)
        {
            _activationPermittedByModKeyMask = (_activationModKeyMask!=0) ?
                ((ea.getModKeyMask() & _activationModKeyMask)!=0) :
                false;

            _activationPermittedByMouseButtonMask = (_activationMouseButtonMask!=0) ?
                ((ea.getButtonMask() & _activationMouseButtonMask)!=0) :
                false;

            if (_activationKeyEvent!=0)
            {
                switch (ea.getEventType())
                {
                    case osgGA::GUIEventAdapter::KEYDOWN:
                    {
                        if (ea.getKey()==_activationKeyEvent) _activationPermittedByKeyEvent = true;
                        break;
                    }
                    case osgGA::GUIEventAdapter::KEYUP:
                    {
                        if (ea.getKey()==_activationKeyEvent) _activationPermittedByKeyEvent = false;
                        break;
                    }
                    default:
                        break;
                }
            }

            activationPermitted =  _activationPermittedByModKeyMask || _activationPermittedByMouseButtonMask || _activationPermittedByKeyEvent;

        }

        if (activationPermitted || _draggerActive)
        {
            switch (ea.getEventType())
            {
                case osgGA::GUIEventAdapter::PUSH:
                {
                    osgUtil::LineSegmentIntersector::Intersections intersections;

                    _pointer.reset();

                    if (view->computeIntersections(ea ,intersections, _intersectionMask))
                    {
                        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
                            hitr != intersections.end();
                            ++hitr)
                        {
                            _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
                        }
                        int listSize = _pointer._hitList.size();
                        while((--listSize) > -1)
                        {
                            for (osg::NodePath::iterator itr = _pointer._hitList.front().first.begin();
                                    itr != _pointer._hitList.front().first.end();
                                    ++itr)
                            {
                                osg::ref_ptr<ParamAxisDragger> dragger = dynamic_cast<ParamAxisDragger*>(*itr);
                                if (dragger)
                                {
                                    if (dragger==this)
                                    {
                                        osg::Camera *rootCamera = view->getCamera();
                                        osg::NodePath nodePath = _pointer._hitList.front().first;
                                        osg::NodePath::reverse_iterator ritr;
                                        for(ritr = nodePath.rbegin();
                                            ritr != nodePath.rend();
                                            ++ritr)
                                        {
                                            osg::ref_ptr<osg::Camera> camera = dynamic_cast<osg::Camera*>(*ritr);
                                            if (camera && (camera->getReferenceFrame()!=osg::Transform::RELATIVE_RF || camera->getParents().empty()))
                                            {
                                                 rootCamera = camera;
                                                 break;
                                            }
                                        }
                                        _pointer._hitIter = _pointer._hitList.begin();
                                        _pointer.setCamera(rootCamera);
                                        m_centerPointToWindowPosi = _WorldToScreen(rootCamera,dragger->getBound().center());
//                                        _pointer.setMousePosition(m_centerPointToWindowPosi.x(), m_centerPointToWindowPosi.y());
                                        m_eaX = ea.getX();
                                        m_eaY = ea.getY();
                                        _pointer.setMousePosition(ea.getX(), ea.getY());
                                        m_bControllEndEffector = true;

//                                        if(!!m_endEffectorControllItem && "EndEffectorsDragger" == m_draggerName)
//                                        {
//                                            m_prevDraggerMatrixInvet.invert(dragger->getMatrix());
//                                            RobotBasePtr probot = m_endEffectorControllItem->_probot;
//                                            for(size_t i = 0; i < probot->GetManipulators().size(); ++i) {
//                                                if( probot->GetManipulators()[i]->GetName().find(this->getName()) != string::npos ) {
//                                                    OpenRAVE::Transform t = probot->GetManipulators()[i]->GetEndEffectorTransform();
//                                                    m_iniDraggerMatrixInvert.invert(aqroserave::GetMatrixFromRaveTransform(t));
//                                                    break;
//                                                }
//                                            }
//                                        }
                                        if(dragger->handle(_pointer, ea, aa))
                                        {
                                            dragger->setDraggerActive(true);

                                            handled = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            if(handled)
                                listSize = 0;
                            else
                            {
                                _pointer._hitList.pop_front();
                            }

                        }
                    }
                    break;
                }
                case osgGA::GUIEventAdapter::DRAG:
                {
                    if (_draggerActive)
                    {
                        _pointer._hitIter = _pointer._hitList.begin();
                        _pointer.setCamera(view->getCamera());
                        _pointer.setMousePosition(ea.getX(), ea.getY());

//                        _ControlEndEffector();

                        if(handle(_pointer, ea, aa))
                        {
                            handled = true;
                        }
                    }
                    break;
                 }
                case osgGA::GUIEventAdapter::RELEASE:
                {
                    if (_draggerActive)
                    {
                        _pointer._hitIter = _pointer._hitList.begin();
    //                    _pointer.setCamera(view->getCamera());
                        _pointer.setMousePosition(ea.getX(), ea.getY());
                        if(handle(_pointer, ea, aa))
                        {
                            handled = true;
                        }
                        int draggers_index=0;
                        for (DraggerList::iterator itr=_draggerList.begin(); itr!=_draggerList.end(); ++itr)
                        {
//                            osg::ref_ptr<osgManipulator::Dragger> dd = osg::ref_ptr<osgManipulator::Dragger>(*itr);

                            osg::ref_ptr<osgManipulator::Translate1DDragger> draggerx = osg::dynamic_pointer_cast<osgManipulator::Translate1DDragger>(*itr);
                            if(!!draggerx)
                            {
                                draggerx->setNodeMask(1);
                                switch(draggers_index)
                                {
                                    case 0:
                                        draggerx->setColor(osg::Vec4(1.0,0.0,0.0,1.0));
                                    break;
                                    case 1:
                                        draggerx->setColor(osg::Vec4(0.0,1.0,0.0,1.0));
                                    break;
                                    case 2:
                                        draggerx->setColor(osg::Vec4(0.0,0.0,1.0,1.0));
                                    break;
                                    default:
                                        draggerx->setColor(osg::Vec4(1.0,1.0,1.0,1.0));
                                    break;
                                }

                            }
                            draggers_index += 1;
                        }
//                        if(!!m_endEffectorControllItem && "EndEffectorsDragger" == m_draggerName)
//                        {
//                            m_endEffectorControllItem->m_vsolution1.clear();
//                            RobotBasePtr probot = m_endEffectorControllItem->_probot;
//                            for(size_t i = 0; i < probot->GetManipulators().size(); ++i) {
//                                if( probot->GetManipulators()[i]->GetName().find(this->getName()) != string::npos ) {
//                                    OpenRAVE::Transform t = probot->GetManipulators()[i]->GetEndEffectorTransform();
//                                    osg::Matrix tmx = aqroserave::GetMatrixFromRaveTransform(t);
//                                    tmx = tmx * m_iniDraggerMatrixInvert;
//                                    for(unsigned int j = 0; j< m_endEffectorControllItem->m_draggers.size();++j)
//                                        m_endEffectorControllItem->m_draggers[j]->setMatrix(tmx.identity());
//                                    break;
//                                }
//                            }
//                        }

                    }
                    break;
                }
                default:
                    break;
            }

            if (_draggerActive && ea.getEventType() == osgGA::GUIEventAdapter::RELEASE)
            {
                setDraggerActive(false);
                _pointer.reset();
            }
        }

        return handled;
    }
}

bool ParamAxisDragger::handle(const osgManipulator::PointerInfo &pi, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    {
        bool bHandle = false;
        // Check if the dragger node is in the nodepath.

//        if(this && _pointer._hitIter != _pointer._hitList.end())
//        {
//            bHandle = std::find((*(_pointer._hitIter)).first.begin(), (*(_pointer._hitIter)).first.end(), this) != (*(_pointer._hitIter)).first.end();
//        }

        if (!pi.contains(this))
        {
            bHandle = false;
            return bHandle;
        }
        if(!m_bControllEndEffector)
        {
            bHandle = false;
            return bHandle;
        }
        int i=0;
        for (DraggerList::iterator itr=_draggerList.begin(); itr!=_draggerList.end(); ++itr,i++)
        {
                if ((*itr)->handle(pi, ea, aa))
                {
                    m_draggerActiveIndex = i;
                    bHandle = true;
                    (*itr)->setNodeMask(1);
                }
                else
                {
                    (*itr)->setNodeMask(0);
                }
//            }
        }



        return bHandle;
    }
}

void ParamAxisDragger::setupDefaultGeometry()
{
    {
        osg::ref_ptr<osg::Geode> geodeX = new osg::Geode;
        osg::ref_ptr<osg::Geode> geodeY = new osg::Geode;
        osg::ref_ptr<osg::Geode> geodeZ = new osg::Geode;

        // Create a cone.

            _cone = new osg::Cone (osg::Vec3(0.0f, 0.0f, 1.0f+_coneHeight * 0.25f), _coneHeight * 0.25f, _coneHeight);
            osg::ref_ptr<osg::ShapeDrawable> coneDrawableX = new osg::ShapeDrawable(_cone.get());
            osg::ref_ptr<osg::ShapeDrawable> coneDrawableY = new osg::ShapeDrawable(_cone.get());
            osg::ref_ptr<osg::ShapeDrawable> coneDrawableZ = new osg::ShapeDrawable(_cone.get());
            coneDrawableX->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
            coneDrawableY->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
            coneDrawableZ->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
            geodeX->addDrawable(coneDrawableX);
            geodeY->addDrawable(coneDrawableY);
            geodeZ->addDrawable(coneDrawableZ);

            // This ensures correct lighting for scaled draggers.
    #if !defined(OSG_GLES2_AVAILABLE)
            geodeX->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
            geodeY->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
            geodeZ->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
//            osg::CullFace* cf = new osg::CullFace( osg::CullFace::BACK );
//            geodeX->getOrCreateStateSet()->setAttribute( cf );
//            geodeY->getOrCreateStateSet()->setAttribute( cf );
//            geodeZ->getOrCreateStateSet()->setAttribute( cf );
    #endif

        // Create an invisible cylinder for picking the line.
        {
            _cylinder = new osg::Cylinder (osg::Vec3(0.0f,0.0f,0.5f), _pickCylinderRadius, 1.0f);
            osg::ref_ptr<osg::ShapeDrawable> geometryX = new osg::ShapeDrawable(_cylinder.get());
            osg::ref_ptr<osg::ShapeDrawable> geometryY = new osg::ShapeDrawable(_cylinder.get());
            osg::ref_ptr<osg::ShapeDrawable> geometryZ = new osg::ShapeDrawable(_cylinder.get());
            geometryX->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
            geometryY->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
            geometryZ->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
            geodeX->addDrawable(geometryX);
            geodeY->addDrawable(geometryY);
            geodeZ->addDrawable(geometryZ);
        }

        // Add geode to all 1D draggers.
        _xDragger->addChild(geodeX.get());
        _yDragger->addChild(geodeY.get());
        _zDragger->addChild(geodeZ.get());

        // Rotate X-axis dragger appropriately.
        {
            osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
            _xDragger->setMatrix(osg::Matrix(rotation));
        }

        // Rotate Y-axis dragger appropriately.
        {
            osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
            _yDragger->setMatrix(osg::Matrix(rotation));
        }

        // Send different colors for each dragger.
        _xDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
        _yDragger->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
        _zDragger->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
    }
}

void ParamAxisDragger::_changeDraggersColor()
{
    {
        _xDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
        _yDragger->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
        _zDragger->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
    }
}

int ParamAxisDragger::_getActiveDragger()
{
    return m_draggerActiveIndex;
}

void ParamAxisDragger::_Transform_Point(double out[], const double m[], const double in[])
{
    out[0] = M_M(0, 0) * in[0] + M_M(0, 1) * in[1] + M_M(0, 2) * in[2] + M_M(0, 3) * in[3];
    out[1] = M_M(1, 0) * in[0] + M_M(1, 1) * in[1] + M_M(1, 2) * in[2] + M_M(1, 3) * in[3];
    out[2] = M_M(2, 0) * in[0] + M_M(2, 1) * in[1] + M_M(2, 2) * in[2] + M_M(2, 3) * in[3];
    out[3] = M_M(3, 0) * in[0] + M_M(3, 1) * in[1] + M_M(3, 2) * in[2] + M_M(3, 3) * in[3];
}

osg::Vec3d ParamAxisDragger::_WorldToScreen(osg::Camera *rootCamera, osg::Vec3 worldpoint)
{
    double in[4], out[4];

    in[0] = worldpoint._v[0];
    in[1] = worldpoint._v[1];
    in[2] = worldpoint._v[2];
    in[3] = 1.0;
    //

    osg::Matrix projectMatrix= rootCamera->getProjectionMatrix();
    osg::Matrix viewprojectMatrix = rootCamera->getViewMatrix();
    //
    double modelViewMatrix[16];
    memcpy(modelViewMatrix,viewprojectMatrix.ptr(),sizeof(GLdouble) * 16);
    _Transform_Point(out, modelViewMatrix, in);

    double myprojectMatrix[16];
    memcpy(myprojectMatrix,projectMatrix.ptr(),sizeof(GLdouble) * 16);

    _Transform_Point(in, myprojectMatrix, out);

    if(int(in[3] * 100000) == 0){
       return osg::Vec3d(0,0,0);
    }

    in[0] /= in[3];
    in[1] /= in[3];
    in[2] /= in[3];

    int viewPort[4];
    //osg::Viewport* myviewPort = rootCamera->getViewport();
    viewPort[0] = 0;
    viewPort[1] = 0;
    viewPort[2] = m_windowWithPix; //
    viewPort[3] = m_windowHeightPix;//
//
    osg::Vec3d sceenPoint;
    sceenPoint._v[0] = (int)(viewPort[0] + (1 + in[0]) * viewPort[2] / 2 + 0.5);
    sceenPoint._v[1] = (int)(viewPort[1] + (1 + in[1]) * viewPort[3] / 2 + 0.5);
    sceenPoint._v[2] = 0;
    return sceenPoint;
}
