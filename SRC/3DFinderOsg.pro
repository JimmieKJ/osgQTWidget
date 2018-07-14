#-------------------------------------------------
#
# Project created by QtCreator 2018-03-29T19:37:43
#
#-------------------------------------------------

QT       += core gui
QT       += opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3DFinderOsg
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

unix {
    # You may need to change this include directory
    INCLUDEPATH += /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/include \
                   /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/include/OpenThreads \
                   /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/include/osg \
                   /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/include/osgQt \
                    /usr/local/include/osg \
                    /usr/local/include/osgAnimation \
                    /usr/local/include/osgFX \
                    /usr/local/include/osgGA \
                    /usr/local/include/osgManipulator \
                    /usr/local/include/osgParticle \
                    /usr/local/include/osgPresentation \
                    /usr/local/include/osgQt \
                    /usr/local/include/osgShadow \
                    /usr/local/include/osgSim \
                    /usr/local/include/osgTerrain \
                    /usr/local/include/osgText \
                    /usr/local/include/osgUI \
                    /usr/local/include/osgUtil \
                    /usr/local/include/osgViewer \
                    /usr/local/include/osgVolume \
                    /usr/local/include/osgWidget \
                    ./include/eigen3 \
                    ./include/pcl-1.8 \
                    /home/aqrose23/Downloads/aq_depends/osgworks-master/include/osgwControls \
/home/aqrose23/Downloads/aq_depends/osgworks-master/include/osgwMx \
/home/aqrose23/Downloads/aq_depends/osgworks-master/include/osgwQuery \
/home/aqrose23/Downloads/aq_depends/osgworks-master/include/osgwTools \
                     /usr/local/include/bullet \
                     /home/aqrose23/Downloads/aq_depends/bullet3-master/Extras \
                     /usr/local/include/osgbCollision
    LIBS += /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libOpenThreads.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgGA.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgQt.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgTerrain.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgViewer.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgAnimation.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgManipulator.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgShadow.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgText.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgVolume.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgDB.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgParticle.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgSim.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgUI.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgWidget.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgFX.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgPresentation.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosg.so \
            /home/aqrose23/Downloads/aq_depends/OpenSceneGraph/build/lib/libosgUtil.so \
            /usr/local/lib/libpcl_io_ply.so.1.8 \
            /usr/local/lib/libpcl_io.so.1.8 \
            /usr/local/lib/libpcl_common.so.1.8 \
            /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.58.0 \
            /usr/lib/x86_64-linux-gnu/libboost_system.so.1.58.0 \
            /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.58.0 \
            /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.58.0 \
            /home/aqrose23/Downloads/aq_depends/osgworks-master/build/lib//libosgwControls.so\
            /home/aqrose23/Downloads/aq_depends/osgworks-master/build/lib//libosgwMx.so \
            /home/aqrose23/Downloads/aq_depends/osgworks-master/build/lib//libosgwQuery.so \
            /home/aqrose23/Downloads/aq_depends/osgModeling/build/src/osgModeling/libosgModeling.so \
            /home/aqrose23/Downloads/aq_depends/osgworks-master/build/lib/libosgwControls.so \
            /home/aqrose23/Downloads/aq_depends/osgworks-master/build/lib/osgdb_osgwTools.so \
            /usr/local/lib/libBulletRobotics.a \
            /usr/local/lib/libBulletDynamics.a \
            /usr/local/lib/libBulletCollision.a \
            /usr/local/lib/libConvexDecomposition.a \
            /usr/local/lib/libGIMPACTUtils.a \
            /usr/local/lib/libHACD.a \
#            /usr/local/lib/libBulletFileLoader.a \
#            /usr/local/lib/libBulletInverseDynamics.a \
#            /usr/local/lib/libBulletInverseDynamicsUtils.a \
#            /usr/local/lib/libBulletWorldImporter.a \
#            /usr/local/lib/libBulletXmlWorldImporter.a \
            /usr/local/lib/libLinearMath.a \
            /usr/local/lib/libBulletSoftBody.a \
#            /usr/local/lib/libBullet3Common.a \
#            /usr/local/lib/libBullet2FileLoader.a \
#            /usr/local/lib/libBullet3Collision.a \
#            /usr/local/lib/libBullet3Dynamics.a \
#            /usr/local/lib/libBullet3Geometry.a \
#            /usr/local/lib/libBullet3OpenCL_clew.a \
            /usr/local/lib/libosgbCollision.a \
            /usr/local/lib/libosgbDynamics.a \
            /usr/local/lib/libosgbInteraction.a
}
SOURCES += \
    main.cpp \
    osgviewwidget.cpp \
    qosgmainwindow.cpp \
    qosgtrackball.cpp \
    qosgkeyboardeventhandler.cpp \
    qosgenvmodel.cpp \
    qosgoctreebuilder.cpp \
    osgpick.cpp \
    osgskybox.cpp \
    qtosgleftsidebar.cpp \
    qosgselecteditemdraggercallback.cpp \
    paramtrackballdragger.cpp \
    paramaxisdragger.cpp \
    qosgcollsioncheck.cpp \
    qosgcommon.cpp \
    qosgupdatecallbackcheckcollsion.cpp \
    qosgcomputetrimeshvisitor.cpp \
    qosgcollectverticesvisitor.cpp \
    qosgabsolutemodeltransform.cpp

HEADERS += \
    osgviewwidget.h \
    qosgmainwindow.h \
    qosgtrackball.h \
    qosgkeyboardeventhandler.h \
    qosgenvmodel.h \
    qosgoctreebuilder.h \
    osgpick.h \
    osgskybox.h \
    qtosgleftsidebar.h \
    qosgselecteditemdraggercallback.h \
    paramtrackballdragger.h \
    paramaxisdragger.h \
    qosgcollsioncheck.h \
    qosgcommon.h \
    qosgupdatecallbackcheckcollsion.h \
    qosgcomputetrimeshvisitor.h \
    qosgcollectverticesvisitor.h \
    qosgabsolutemodeltransform.h

QMAKE_CXXFLAGS +=  -Wno-unused-parameter
QMAKE_CXXFLAGS += -Wno-deprecated-declarations

RESOURCES += \
    images.qrc

DISTFILES +=
