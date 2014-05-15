#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>

#include <Eigen/Dense>
#include <lemon/list_graph.h>
#include <flann/flann.hpp>

#include "graph_map.h"


class PointCloud{
public:
    typedef lemon::ListGraph DeformationGraph;
  
public:
    PointCloud();
    ~PointCloud();
    
    void binding();
    
private:
    virtual void sampling();
    virtual void connecting();
    
    void kNearestSearch(const int& k);
    void buildEdges(int* start, const int& k);
 
    
private:
    std::vector<Eigen::Vector3d> point_cloud_; 
    DeformationGraph deformation_graph_;
    GraphMap graph_map_;
  //  DeformationGraph::NodeMap<size_t> node_index_;
    flann::Matrix<int> nearest_neighbors_;
    size_t node_num_;
};

#endif //POINT_CLOUD_H