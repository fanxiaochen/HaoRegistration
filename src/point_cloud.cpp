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
    sampling();
    buildEdges();
}

void PointCloud::sampling()
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

void PointCloud::buildEdges()
{
    
}

void PointCloud::kNearestSearch(int k)
{
    flann::Matrix<double> data_set(new double[node_num_ * 3], node_num_, 3);
    flann::Matrix<double> query(new double[point_cloud_.size() * 3], point_cloud_.size(), 3);
    
    size_t i = 0;
    for (DeformationGraph::NodeIt it(deformation_graph_); it != lemon::INVALID; ++ it, i ++)
    {
        for (size_t j = 0; j < 3; j ++)
        {
            Eigen::Vector3d point = point_cloud_.at(node_index_[it]);
            data_set[i][j] = point(0,j);          
        }
    }
    
    flann::Matrix<int> indices(new int[query.rows * k], query.rows, k);
    flann::Matrix<double> dists(new double[query.rows * k], query.rows, k);
    
    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<float> > index(data_set, flann::KDTreeIndexParams(4));
    index.buildIndex();
    // do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, k, flann::SearchParams(128));
    
    nearest_neighbors_ = indices;
}


