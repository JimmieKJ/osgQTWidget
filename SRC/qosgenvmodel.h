#ifndef QOSGENVMODEL_H
#define QOSGENVMODEL_H

#include <QObject>
#include <osg/MatrixTransform>
#include <QMetaEnum>
#include <osg/BlendEquation>

class FindNode : public osg::NodeVisitor
{
public:
    // Traverse all children.
    FindNode( const std::string& nodeId ) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ), m_nodeId( nodeId ) {
        m_foundTrans = NULL;
        _found = false;
    }
    // This method gets called for every node in the scene
    //   graph. Check each node to see if its name matches
    //   our target. If so, save the node's address.
    virtual void apply( osg::MatrixTransform& node )
    {
        if (node.getName() == m_nodeId) {
            m_foundTrans = &node;
            _found = true;
        }
        else {
            // Keep traversing the rest of the scene graph.
            traverse( node );
        }
    }
    osg::MatrixTransform* IsFound() const {
        return m_foundTrans;
    }

protected:
    std::string m_nodeId;
    bool _found;
    osg::MatrixTransform* m_foundTrans;
};

class QOSGEnvModel : public QObject
{
    Q_OBJECT
public:

    struct GWRect{
        float top;
        float bottom;

        float left;
        float right;
    };

    enum DrawModeType {
        PointCloud,
        BoxModel,
        CyliderModel,
        PlaneModel,
        BoxEdgeModel,
        FrameModel,
        GridModel,
        ModelFromDisk
    };
    QMetaEnum m_metaDrawModeTypeEnum;
    Q_ENUM(DrawModeType)
    QOSGEnvModel(osg::ref_ptr<osg::Group> osgSceneRoot,QObject *parent = NULL);
    osg::MatrixTransform* _getModelByID(const std::string & modelID, bool bRemove=false);
    osg::MatrixTransform* _getModelByID(const std::string & modelID, DrawModeType drawMode, bool bRemove=false);
    void _removeRemainData(const std::string & cloudID, DrawModeType drawMode);//1 cloud
    void _addRemainData(const std::string & cloudID, DrawModeType drawMode);
    void _removeOneTypeModelData(DrawModeType drawMode);
    bool _bIsRemainData(osg::ref_ptr<osg::MatrixTransform> tempGroup);
    osg::Group* _createGridWide(int netWidth,int netResolution,double gridWidth);
    void _addBillboard(osg::ref_ptr<osg::Group> ManipPeesep,const std::string& textStr);
    void _changeModelColor(osg::ref_ptr<osg::Group> osgColorImage, const osg::Vec3 &imageColor, int textureMode = 0);
    osg::Image *_createImage(int width, int height, const osg::Vec3 &color );
    void _SetVisualizationMode(const std::string& visualizationmode,osg::ref_ptr<osg::MatrixTransform> osgWorldTransform);
    void _setSelectItem(osg::ref_ptr<osg::MatrixTransform> selectNode);
    void _setCollisionItem(osg::ref_ptr<osg::MatrixTransform> selectNode);
    bool _checkIsPointCloud(osg::ref_ptr<osg::MatrixTransform> selectNode);
    void _cretateBoundingBox(osg::ref_ptr<osg::MatrixTransform> selectNode);
signals:

public slots:

private:
    void _createVisualizaNode(osg::ref_ptr<osg::Group> osgwireframe);

public:
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_cloudGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_BoxGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_CyliderGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_PlaneGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_BoxEdgeGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_FrameGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_GridFloorGroups;
    std::map<std::string,osg::ref_ptr<osg::MatrixTransform> > m_ModelFromDiskGroup;
    std::vector<osg::ref_ptr<osg::MatrixTransform> > m_remainDataGroups;
    osg::ref_ptr<osg::Group> m_osgSceneRoot;
    const osg::BlendEquation::Equation m_equations[8]=
    {
        osg::BlendEquation::FUNC_ADD,
        osg::BlendEquation::FUNC_SUBTRACT,
        osg::BlendEquation::FUNC_REVERSE_SUBTRACT,
        osg::BlendEquation::RGBA_MIN,
        osg::BlendEquation::RGBA_MAX,
        osg::BlendEquation::ALPHA_MIN,
        osg::BlendEquation::ALPHA_MAX,
        osg::BlendEquation::LOGIC_OP
    };
private:
    osg::ref_ptr<osg::Group> m_osgwireframe;
    osg::ref_ptr<osg::Group> m_osgcollisionframe;
    osg::ref_ptr<osg::Group> m_boundingBoxGroup;
};

#endif // QOSGENVMODEL_H
