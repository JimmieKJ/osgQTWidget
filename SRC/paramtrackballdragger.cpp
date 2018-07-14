#include "paramtrackballdragger.h"
#include <osgViewer/View>

#define M_M(row,col) m[col * 4 + row]

ParamTrackballDragger::ParamTrackballDragger(QObject *parent) : QObject(parent)
{
    m_projector = new osgManipulator::CylinderPlaneProjector();
}

bool ParamTrackballDragger::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
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
                                osg::ref_ptr<ParamTrackballDragger> dragger = dynamic_cast<ParamTrackballDragger*>(*itr);
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
                                        m_rootCamera = rootCamera;
                                        _pointer._hitIter = _pointer._hitList.begin();
                                        _pointer.setCamera(rootCamera);
                                        _pointer.setMousePosition(ea.getX(), ea.getY());
                                        m_eaX = ea.getX();
                                        m_eaY = ea.getY();
                                        m_bControllEndEffector = true;
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
                    osg::Vec3f centerPointToWindowPosi;
                    if (_draggerActive)
                    {
                        _pointer._hitIter = _pointer._hitList.begin();
                        osg::Matrix eeTransForFirstLink;
                        if(!!m_ControllItem){
                            eeTransForFirstLink = m_ControllItem->getMatrix();
                        }
                        centerPointToWindowPosi = _WorldToScreen(m_rootCamera,osg::Vec3(eeTransForFirstLink.getTrans().x(),eeTransForFirstLink.getTrans().y(),eeTransForFirstLink.getTrans().z()));
                        osg::Vec3d va(m_eaX - centerPointToWindowPosi.x(),m_eaY - centerPointToWindowPosi.y(),0);
                        osg::Vec3d vb(ea.getX() - centerPointToWindowPosi.x(),ea.getY() - centerPointToWindowPosi.y(),0);
                        osg::Vec3d vc = va^vb;
                        if(vc.z()>0)
                            m_bClockWise = true;
                        else if(vc.z()<0)
                            m_bClockWise = false;

                        _pointer.setMousePosition(ea.getX(), ea.getY());
//                        _ControlEndEffector();
                        if(handle(_pointer, ea, aa))
                        {
                            m_eaX = ea.getX();
                            m_eaY = ea.getY();
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
                            osg::ref_ptr<osgManipulator::RotateCylinderDragger> draggerx = osg::dynamic_pointer_cast<osgManipulator::RotateCylinderDragger>(*itr);

                            if(!!draggerx)
                            {
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
                                draggerx->setNodeMask(1);

                            }
                            draggers_index += 1;
                        }
//                        if(!!m_endEffectorControllItem && "EndEffectorsDragger" == m_draggerName)
//                        {
//                            RobotBasePtr probot = m_endEffectorControllItem->_probot;
//                            m_endEffectorControllItem->m_vsolution1.clear();
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

bool ParamTrackballDragger::handle(const osgManipulator::PointerInfo &pi, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    {
        bool bHandle = false;
        // Check if the dragger node is in the nodepath.
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

        for (DraggerList::iterator itr=_draggerList.begin(); itr!=_draggerList.end(); ++itr)
        {
            if ((*itr)->handle(pi, ea, aa))
            {
               bHandle = true;
               (*itr)->setNodeMask(1);
            }
            else
            {
               (*itr)->setNodeMask(0);
            }
//            osg::ref_ptr<osgManipulator::RotateCylinderDragger> draggerx = osg::dynamic_pointer_cast<osgManipulator::RotateCylinderDragger>(*itr);
//            if(!!draggerx){
//                if (_handleRotate((draggerx),pi, ea, aa))
//                {
//                    bHandle = true;
//                    draggerx->setNodeMask(1);
//                }
//                else
//                {
//                    draggerx->setNodeMask(0);
//                }
//            }
        }


        return bHandle;
    }
}

osg::ref_ptr<osgModeling::Loft> ParamTrackballDragger::createCircleGeometry(float radius, unsigned int numSegments, float SegmentsLen, osg::Vec3 &conCenter)
{
    {
        const float angleDelta = 2.0f*osg::PI/(float)numSegments;
        const float r = radius;
        float angle = 0.0f;
        osg::ref_ptr<osgModeling::Curve> helix = new osgModeling::Curve;

        for(unsigned int i = 0; i < numSegments * SegmentsLen; ++i,angle+=angleDelta)
        {
            if(angle > M_PI*2*SegmentsLen)
                break;
            float c = cosf(angle);
            float s = sinf(angle);

            conCenter = osg::Vec3(c*r,s*r,0.0f);
            helix->addPathPoint( conCenter);

        }

        osg::ref_ptr<osgModeling::Curve> section = new osgModeling::Curve;
        for(unsigned int i = 0; i < numSegments+1; ++i,angle+=angleDelta)
        {
            float c = cosf(angle);
            float s = sinf(angle);
            section->addPathPoint(osg::Vec3(c*r*0.04,s*r*0.04,0.0f));
        }

        osg::ref_ptr<osgModeling::Loft> circleLoft =new osgModeling::Loft( helix.get(), section.get() );
        return circleLoft;
    }
}

void ParamTrackballDragger::setupDefaultGeometry()
{
    {
        _geode = new osg::Geode;
        osg::ref_ptr<osg::Geode> conGeoX = new osg::Geode;
        osg::ref_ptr<osg::Geode> conGeoY = new osg::Geode;
        osg::ref_ptr<osg::Geode> conGeoZ = new osg::Geode;
        osg::ref_ptr<osg::MatrixTransform > contransX = new osg::MatrixTransform ;
        osg::ref_ptr<osg::MatrixTransform > contransY = new osg::MatrixTransform ;
        osg::ref_ptr<osg::MatrixTransform > contransZ = new osg::MatrixTransform ;
        float circleRe = 1.0f;
        {
            osg::Vec3 conCenter;
            _geode->addDrawable(createCircleGeometry(circleRe, 100,0.22222,conCenter).get());

            osg::ref_ptr<osg::Cone> _cone = new osg::Cone (conCenter, 1.0 *circleRe*0.3* 0.25f, 1.0*circleRe*0.2);
            osg::ref_ptr<osg::ShapeDrawable> coneDrawableX = new osg::ShapeDrawable(_cone.get());
            osg::ref_ptr<osg::ShapeDrawable> coneDrawableY = new osg::ShapeDrawable(_cone.get());
            osg::ref_ptr<osg::ShapeDrawable> coneDrawableZ = new osg::ShapeDrawable(_cone.get());
            coneDrawableX->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
            coneDrawableY->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
            coneDrawableZ->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
            conGeoX->addDrawable(coneDrawableX);
            conGeoY->addDrawable(coneDrawableY);
            conGeoZ->addDrawable(coneDrawableZ);
            contransX->addChild(conGeoX);
            contransY->addChild(conGeoY);
            contransZ->addChild(conGeoZ);

//            #if !defined(OSG_GLES2_AVAILABLE)
//                _geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
//                osg::CullFace* cf = new osg::CullFace( osg::CullFace::BACK );
//                _geode->getOrCreateStateSet()->setAttribute( cf );
//            #endif

        }
        // Add line to all the individual 1D draggers.
        _xDragger->addChild(_geode.get());
        _yDragger->addChild(_geode.get());
        _zDragger->addChild(_geode.get());

        // Rotate z-axis dragger appropriately.
        {
            osg::Quat rotation,rot;
            rotation.makeRotate(osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f));
            rot.makeRotate(osg::Vec3(0.0f, 1.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
            rotation = rotation * rot;
            _zDragger->setMatrix(osg::Matrix(rotation));
            contransZ->setMatrix(osg::Matrix::rotate(-(M_PI*0.5),osg::Vec3(0.0,1.0,0.0)) * osg::Matrix::translate(0.0,0.0,-circleRe*0.19)*osg::Matrix::translate(circleRe*0.18,0.0,0.0));
            _zDragger->addChild(contransZ);
        }

        // Rotate Y-axis dragger appropriately.
        {
            osg::Quat rotation;
            rotation.makeRotate(osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f));
            rotation *= osg::Matrix::rotate(M_PI,osg::Vec3(0.0,1.0,1.0)).getRotate();
            _yDragger->setMatrix(osg::Matrix(rotation));
            contransY->setMatrix(osg::Matrix::rotate(-(M_PI*0.5),osg::Vec3(0.0,1.0,0.0)) * osg::Matrix::translate(0.0,0.0,-circleRe*0.19)*osg::Matrix::translate(circleRe*0.18,0.0,0.0));
            _yDragger->addChild(contransY);
        }

        contransX->setMatrix(osg::Matrix::rotate(-(M_PI*0.5),osg::Vec3(0.0,1.0,0.0)) * osg::Matrix::translate(0.0,0.0,-circleRe*0.19)*osg::Matrix::translate(circleRe*0.18,0.0,0.0));
        _xDragger->addChild(contransX);

//         Send different colors for each dragger.
        _xDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
        _yDragger->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
        _zDragger->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

        // Add invisible sphere for pick the spherical dragger.
//        {
//            osg::Drawable* sphereDrawable = new osg::ShapeDrawable(new osg::Sphere());
//            osgManipulator::setDrawableToAlwaysCull(*sphereDrawable);
//            osg::Geode* sphereGeode = new osg::Geode;
//            sphereGeode->addDrawable(sphereDrawable);

//            _xyzDragger->addChild(sphereGeode);
//        }
    }
}

void ParamTrackballDragger::_changeDraggersColor()
{
    {
        _xDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
        _yDragger->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
        _zDragger->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
    }
}

void ParamTrackballDragger::_Transform_Point(double out[], const double m[], const double in[])
{
    out[0] = M_M(0, 0) * in[0] + M_M(0, 1) * in[1] + M_M(0, 2) * in[2] + M_M(0, 3) * in[3];
    out[1] = M_M(1, 0) * in[0] + M_M(1, 1) * in[1] + M_M(1, 2) * in[2] + M_M(1, 3) * in[3];
    out[2] = M_M(2, 0) * in[0] + M_M(2, 1) * in[1] + M_M(2, 2) * in[2] + M_M(2, 3) * in[3];
    out[3] = M_M(3, 0) * in[0] + M_M(3, 1) * in[1] + M_M(3, 2) * in[2] + M_M(3, 3) * in[3];
}

bool ParamTrackballDragger::_handleRotate(const osg::ref_ptr<osgManipulator::RotateCylinderDragger> rotateDragger, const osgManipulator::PointerInfo &pointer, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    // Check if the dragger node is in the nodepath.
    if (!pointer.contains(rotateDragger)) return false;

    switch (ea.getEventType())
    {
        // Pick start.
        case (osgGA::GUIEventAdapter::PUSH):
            {
                // Get the LocalToWorld matrix for this node and set it for the projector.
                osg::NodePath nodePathToRoot;
                osgManipulator::computeNodePathToRoot(*rotateDragger,nodePathToRoot);
                osg::Matrix localToWorld = osg::computeLocalToWorld(nodePathToRoot);
                m_projector->setLocalToWorld(localToWorld);

                m_startLocalToWorld = m_projector->getLocalToWorld();
                m_startWorldToLocal = m_projector->getWorldToLocal();

                if (m_projector->isPointInFront(pointer, m_startLocalToWorld))
                    m_projector->setFront(true);
                else
                    m_projector->setFront(false);

                osg::Vec3d projectedPoint;
                m_lastAngleValue = 0.0;
                m_resAngleValue = 0.0;
                m_draggerAxis.set(0.0,0.0,0.0);
                m_oldZ = 0.0;
                if (m_projector->project(pointer, projectedPoint))
                {
                    // Generate the motion command.
                    osg::ref_ptr<osgManipulator::Rotate3DCommand> cmd = new osgManipulator::Rotate3DCommand();
                    cmd->setStage(osgManipulator::MotionCommand::START);
                    cmd->setLocalToWorldAndWorldToLocal(m_startLocalToWorld,m_startWorldToLocal);

                    // Dispatch command.
                    dispatch(*cmd);

                    m_prevWorldProjPt = projectedPoint * m_projector->getLocalToWorld();
//                    m_prevWorldProjPt = rotateDragger->getBound().center();
                    m_prevRotation = osg::Quat();

                    aa.requestRedraw();
                }
                return true;
            }

        // Pick move.
        case (osgGA::GUIEventAdapter::DRAG):
            {
                // Get the LocalToWorld matrix for this node and set it for the m_projector.
                osg::Matrix localToWorld = osg::Matrix(m_prevRotation) * m_startLocalToWorld;
                m_projector->setLocalToWorld(localToWorld);

                osg::Vec3d projectedPoint;
                if (m_projector->project(pointer, projectedPoint))
                {
                    osg::Vec3d prevProjectedPoint = m_prevWorldProjPt * m_projector->getWorldToLocal();
                    osg::Quat  deltaRotation = m_projector->getRotation(prevProjectedPoint,
                                                                      projectedPoint);
                    osg::Quat rotation = deltaRotation * m_prevRotation;
                    double angle;
                    osg::Vec3d aaxis;
                    rotation.getRotate(angle,aaxis);
                    aaxis.set(0,0,abs(aaxis.z()));
                    if(m_bClockWise )
                        m_resAngleValue = m_resAngleValue + abs(angle - m_lastAngleValue);
                    else
                        m_resAngleValue = m_resAngleValue - abs(angle - m_lastAngleValue);
                    m_lastAngleValue = angle;

                    rotation = osg::Quat(m_resAngleValue,aaxis);


                    // Generate the motion command.
                    osg::ref_ptr<osgManipulator::Rotate3DCommand> cmd = new osgManipulator::Rotate3DCommand();
                    cmd->setStage(osgManipulator::MotionCommand::MOVE);
                    cmd->setLocalToWorldAndWorldToLocal(m_startLocalToWorld,m_startWorldToLocal);
                    cmd->setRotation(rotation);

                    // Dispatch command.
                    dispatch(*cmd);

                    m_prevWorldProjPt = projectedPoint * m_projector->getLocalToWorld();
                    m_prevRotation = rotation;
                    aa.requestRedraw();
                }
                return true;
            }

        // Pick finish.
        case (osgGA::GUIEventAdapter::RELEASE):
            {
                osg::ref_ptr<osgManipulator::Rotate3DCommand> cmd = new osgManipulator::Rotate3DCommand();

                cmd->setStage(osgManipulator::MotionCommand::FINISH);
                cmd->setLocalToWorldAndWorldToLocal(m_startLocalToWorld,m_startWorldToLocal);

                // Dispatch command.
                dispatch(*cmd);

                // Reset color.
//                osgManipulator::setMaterialColor(color,*rotateDragger);

                aa.requestRedraw();

                return true;
            }
        default:
            return false;
    }
}

osg::Vec3d ParamTrackballDragger::_WorldToScreen(osg::Camera *rootCamera, osg::Vec3 worldpoint)
{
    double in[4], out[4];
    in[0] = worldpoint._v[0];
    in[1] = worldpoint._v[1];
    in[2] = worldpoint._v[2];
    in[3] = 1.0;

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
