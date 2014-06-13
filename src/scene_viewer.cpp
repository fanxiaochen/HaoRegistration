#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>

#include "scene_viewer.h"

SceneViewer::SceneViewer()
: viewer_(new osgViewer::Viewer)
{
    
}

SceneViewer::SceneViewer(Visualizer* visualizer, EventHandler* event_handler)
: viewer_(new osgViewer::Viewer),
visualizer_(visualizer),
event_handler_(event_handler)
{
    
}

SceneViewer::~SceneViewer()
{
    
}

void SceneViewer::init()
{
    viewer_->setSceneData(visualizer_->getSceneRoot());
    viewer_->getCamera()->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    viewer_->addEventHandler(event_handler_);
    viewer_->addEventHandler(new osgViewer::StatsHandler);
    viewer_->setCameraManipulator(new osgGA::TrackballManipulator);
// Use single thread here to avoid known issues under Linux
    viewer_->setThreadingModel(osgViewer::Viewer::SingleThreaded);
}

void SceneViewer::start()
{
    viewer_->run();
}