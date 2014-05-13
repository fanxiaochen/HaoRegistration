#include "point_cloud.h"


PointCloud::PointCloud()
    : node_index_(deformation_graph_)
{

}

PointCloud::~PointCloud()
{

}


void PointCloud::binding()
{
    std::vector<size_t> index;
    for (size_t i = 0, i_end = point_cloud_.size(); i < i_end; i ++) {
        index.push_back(i);
    }
    
    // simple binding using random_shuffle, not the way in the original paper
    std::random_shuffle(index.begin(), index.end());
    for (size_t i = 0; i < node_num_; i ++) {
        DeformationGraph::Node node = deformation_graph_.addNode();
        node_index_[node] = index.at(i);
    }
}


