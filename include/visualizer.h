#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <osgViewer/Viewer>

#include "point_cloud.h"

class Visualizer
{
public:
    Visualizer();
    ~Visualizer();

    void init();
    
    void drawPointCloud(PointCloud *point_cloud);
    void drawGraph(PointCloud *point_cloud);

    void visualize();
 
    void closeLight();

private:
    osgViewer::Viewer *viewer_;
    osg::ref_ptr<osg::Group> scene_root_;
};

#endif //VISUALIZER_H
