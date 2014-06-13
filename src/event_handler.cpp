
#include "event_handler.h"

EventHandler::EventHandler()
{

}

EventHandler::EventHandler(Solver* solver, Visualizer* visualizer)
:solver_(solver), visualizer_(visualizer)
{
    
}

EventHandler::~EventHandler()
{
    
}

bool EventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    switch (ea.getEventType()) {

    case osgGA::GUIEventAdapter::KEYDOWN:

        switch (ea.getKey()) {
        case 'r':
        case 'R': {   
            PointCloud* source = solver_->getSource();
            solver_->apply();
            visualizer_->updatePointCloud(source);
            break;
        }
        }
    }

    return false;
}

