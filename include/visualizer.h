#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <osg/Geode>

#include "point_cloud.h"

class Visualizer
{
public:
    Visualizer();
    ~Visualizer();
    
    void addPointCloud(PointCloud *point_cloud);
    void addGraph(PointCloud *point_cloud); 
    void removePointCloud(PointCloud* point_cloud);
    void removeGraph(PointCloud* point_cloud);
    void updatePointCloud(PointCloud* point_cloud);
    void updateGraph(PointCloud* point_cloud);
 
    void closeLight();
    
    inline osg::ref_ptr<osg::Group> getSceneRoot(){
        return scene_root_;
    }

private:
    osg::ref_ptr<osg::Group> scene_root_;  
    std::map<PointCloud*, osg::Geode*> cloud_map_; // be careful, memory address mapping
    std::map<PointCloud*, osg::Group*> graph_map_;
    
};

#endif //VISUALIZER_H
