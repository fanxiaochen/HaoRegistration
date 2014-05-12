#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>

#include <Eigen/Dense>
#include <lemon/list_graph.h>


class PointCloud{
public:
    typedef lemon::ListGraph DeformationGraph;
  
public:
    PointCloud();
    ~PointCloud();
    
private:
    std::vector<Eigen::Vector3d> point_cloud_; 
    DeformationGraph deformation_graph_;
    DeformationGraph::NodeMap<size_t> index_;
};

#endif //POINT_CLOUD_H