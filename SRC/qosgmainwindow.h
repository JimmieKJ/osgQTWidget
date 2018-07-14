#ifndef QOSGMAINWINDOW_H
#define QOSGMAINWINDOW_H
#include "osgviewwidget.h"
#include "qtosgleftsidebar.h"
#include <QMainWindow>
#include <QApplication>
#include <QPushButton>
class QOSGMainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit QOSGMainWindow(QWidget *parent = NULL);

signals:

public slots:
    void slot_calibrateBoxOk(bool bClick);
    void slot_calibrateRemoveAllOk(bool bClick);
    void slot_calibrateCylinderOk(bool bClick);
    void slot_calibrateBoxEdgeOk(bool bClick);
    void slot_calibratePlaneOk(bool bClick);
    void slot_calibrateFrameOk(bool bClick);
    void slot_calibrateGridFloorOk(bool bClick);
    void slot_calibrateOSGWidgetOk(bool bClick);
    void slot_ShowHouseView(bool);
    void slot_ShowXyView(bool);
    void slot_ShowXzView(bool);
    void slot_ShowYzView(bool);
    void slot_loadPointCloudFromFile(bool flag);
    void slot_setRemainModel(bool flag);
    void slot_hideMOdel(bool);
    void slot_showModel(bool);
private:
    void _getScreenInfo();
    void _addViewWidget(int x, int y, int w, int h);
    QPushButton * _addButtonToWindow(QWidget *wid, int x, int y, int w, int h, QString iconStr, QString objName, QString textname);
    void _createLeftSideBar();
private:
    int m_screenWidth;
    int m_screenHeight;
    OSGViewWidget* m_viewWidget;
    float m_focalDistance;
    osg::Matrix m_Tcamera;
    QtOSGLeftSideBar* m_leftSideBar;
    QAction* m_showhouseViewAct;
    QAction* m_showxyViewAct;
    QAction* m_showxzViewAct;
    QAction* m_showyzViewAct;
};

#endif // QOSGMAINWINDOW_H
