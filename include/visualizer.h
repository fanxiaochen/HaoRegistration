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

    void drawSource(PointCloud *source);
    void drawTarget(PointCloud *target);
    void drawGraph(PointCloud *source);

    void visualize();

private:
    virtual void drawImpl(PointCloud *point_cloud);

private:
    osgViewer::Viewer *viewer_;
    osg::ref_ptr<osg::Group> scene_root_;
};

#endif //VISUALIZER_H
