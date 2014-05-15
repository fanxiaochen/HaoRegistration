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
 
    
private:
    std::vector<Eigen::Vector3d> point_cloud_; 
    DeformationGraph deformation_graph_;
    GraphMap graph_map_;
  //  DeformationGraph::NodeMap<size_t> node_index_;
    flann::Matrix<int> nearest_neighbors_;
    size_t node_num_;
};

// to remove same edges because lemon can only support the mode of adding edges between same nodes
struct Edge{
    Edge(int source, int target):_source(source), _target(target){}
    int _source;
    int _target;
};

// compare function for user-defined std::set
// must have strict weak ordering
struct CompareEdge{
    bool operator()(const Edge& edge1, const Edge& edge2){
    
        if ((edge1._source == edge2._source && edge1._target == edge2._target)
            || (edge1._source == edge2._target && edge1._target == edge2._source))
        {
            return false;
        }
        
        if ((edge1._source == edge2._source && edge1._target < edge2._target)
            || (edge1._source < edge2._source))
        {
            return true;
        }
        
        if ((edge1._source == edge2._source && edge1._target > edge2._target)
            || (edge1._source > edge2._source))
        {
            return false;
        }
    }
};

#endif //POINT_CLOUD_H