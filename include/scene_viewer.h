#ifndef SCENE_VIEWER_H
#define SCENE_VIEWER_H

#include <osgViewer/Viewer>

#include "visualizer.h"
#include "event_handler.h"

class SceneViewer
{
public:
    SceneViewer();
    SceneViewer(Visualizer* visualizer, EventHandler* event_handler);
    ~SceneViewer();
    
    void init();
    
    void start();
    
    inline osgViewer::Viewer* getViewer(){
        return viewer_;
    }
    
private:
    osgViewer::Viewer* viewer_;
    Visualizer* visualizer_;
    EventHandler* event_handler_;
};
#endif // SCENE_VIEWER_H