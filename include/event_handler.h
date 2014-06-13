#ifndef EVENT_HANDLE_H
#define EVENT_HANDLE_H

#include <osgGA/GUIEventHandler>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>

#include "solver.h"
#include "visualizer.h"

class EventHandler: public osgGA::GUIEventHandler
{
public:
    EventHandler();
    EventHandler(Solver* solver, Visualizer* visualizer);
    virtual ~EventHandler();
    
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);  
    
private:
    Solver* solver_;
    Visualizer* visualizer_;
};
#endif // EVENT_HANDLE_H