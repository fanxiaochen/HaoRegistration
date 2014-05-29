
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d.h>
#include <boost/concept_check.hpp>

#include "point_cloud.h"


PointCloud::PointCloud()
{

}

PointCloud::~PointCloud()
{
    delete deformation_graph_;
    delete graph_map_;
    delete nearest_neighbors_;
    delete parameter_map_;
}

void PointCloud::binding()
{
    deformation_graph_ = new DeformationGraph();
    graph_map_ = new GraphMap(deformation_graph_);
    parameter_map_ = new ParameterMap(deformation_graph_);

    sampling();
    connecting();
    parameterize();
    buildUnknownsMap();
}

void PointCloud::load(const std::string &file)
{
    depth_map_ = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);  // Read the file
    depth_map_.convertTo(depth_map_, CV_32F); // convert the image data to float type

    width = depth_map_.cols;
    height = depth_map_.rows;
    points.resize(width * height);

    register float constant = 1.0f / 525; // kinect focal length: 525
    register int centerX = (width >> 1);
    register int centerY = (height >> 1);
    register int depth_idx = 0;
    for (int v = -centerY; v < centerY; ++v) {
        for (register int u = -centerX; u < centerX; ++u, ++depth_idx) {
            Point &pt = points[depth_idx];
            pt.z = depth_map_.at<float>(u, v) * 0.001f;
            pt.x = static_cast<float>(u) * pt.z * constant;
            pt.y = static_cast<float>(v) * pt.z * constant;
        }
    }
    sensor_origin_.setZero();
    sensor_orientation_.w() = 0.0f;
    sensor_orientation_.x() = 1.0f;
    sensor_orientation_.y() = 0.0f;
    sensor_orientation_.z() = 0.0f;

    evaluateNormal();
}

void PointCloud::evaluateNormal()
{
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    ne.setInputCloud(pcl::PointCloud<Point>::Ptr(this));
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point> ());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);
    
    for (size_t i = 0, i_end = size(); i < i_end; i ++)
    {
        at(i).normal_x = cloud_normals->at(i).normal_x;
        at(i).normal_y = cloud_normals->at(i).normal_y;
        at(i).normal_z = cloud_normals->at(i).normal_z;
    }
}

void PointCloud::evaluateMassCenter()
{
    Point center;
    for (size_t i = 0, i_end = size(); i < i_end; i ++)
    {    
        center.x += at(i).x;
        center.y += at(i).y;
        center.z += at(i).z;
    }
    
    center.x = center.x / size();
    center.y = center.y / size();
    center.z = center.z / size();
    
    mass_center_(0) = center.x;
    mass_center_(1) = center.y;
    mass_center_(2) = center.z;
}

void PointCloud::computeDependencyWeights()
{
    const int k = k_ + 1;
    kNearestSearch(k);
    
    for (size_t j = 0, j_end = size(); j < j_end; j ++)
    {
        double d_max = (*neighbor_dists_)[j][k - 1];
        const Point& point = at(j);
        double numerator[k_];
        double denominator = 0;
        for (size_t i = 0; i < k_; i ++)
        {
            const Point& node = at((*nearest_neighbors_)[j][i]);
            Eigen::Vector3d vector = EIGEN_POINT_CAST(point) - EIGEN_POINT_CAST(node);
            numerator[i] = 1 - vector.norm() / d_max;
            denominator += numerator[i];
        }
        
        for (size_t i = 0; i < k_; i ++)
        {
            dependency_weights_(j, i) = numerator[i] / denominator;
        }
    }
}

Point PointCloud::localTransform(size_t j)
{
    Eigen::Vector3d vector;
    const Point& point = at(j);
    Eigen::Vector3d eigen_point = EIGEN_POINT_CAST(point); 
    for (size_t i = 0; i < k_; i ++)
    {
        const Point& node = at((*nearest_neighbors_)[j][i]);
        Eigen::Vector3d eigen_node = EIGEN_POINT_CAST(node);
        Parameters parameters = (*parameter_map_)[(*graph_map_)[i]]; // too complicated map index...
        vector += dependency_weights_(j, i) * parameters.affi_rot_ * (eigen_point - eigen_node) + 
        eigen_node + parameters.affi_trans_;        
    }
    return POINT_EIGEN_CAST(vector);
}

Point PointCloud::globalTransform(const Point& point)
{
    Eigen::Matrix3d rotation;
    double theta = rigid_rot_(0);
    double x = rigid_rot_(1);
    double y = rigid_rot_(2);
    double z = sqrt(1 - x*x - y*y);
    rotation << cos(theta)+(1-cos(theta))*x*x, (1-cos(theta))*x*y-sin(theta)*z, (1-cos(theta))*x*z+sin(theta)*y,
                (1-cos(theta))*y*z+sin(theta)*z, cos(theta)+(1-cos(theta))*y*y, (1-cos(theta))*y*z-sin(theta)*x,
                (1-cos(theta))*z*x-sin(theta)*y, (1-cos(theta))*z*y+sin(theta)*x, cos(theta)+(1-cos(theta))*z*z;
    
    Eigen::Vector3d eigen_point = EIGEN_POINT_CAST(point);
    Eigen::Vector3d tran_point = rotation * (eigen_point - mass_center_) + mass_center_ + rigid_trans_;
    return POINT_EIGEN_CAST(tran_point);
}

void PointCloud::transform()
{
    for (size_t j = 0, j_end = size(); j < j_end; j ++)
    {
        // how about normals?
        Point point = globalTransform(localTransform(j));
        at(j) = point;
    }
}

Point PointCloud::getPointFromDepthMap(int u, int v)
{

    return Point();
}

void PointCloud::setNodeNum(size_t node_num)
{
    node_num_ = node_num;
}

void PointCloud::sampling()
{
    std::vector<size_t> index;
    for (size_t i = 0, i_end = size(); i < i_end; i ++) {
        index.push_back(i);
    }

    // simple binding using random_shuffle, not the way in the original paper
    std::random_shuffle(index.begin(), index.end());
    for (size_t i = 0; i < node_num_; i ++) {
        DeformationGraph::Node node = deformation_graph_->addNode();
        graph_map_->insert((GraphMap::MapPair(node, index.at(i))));
    }
}

void PointCloud::connecting()
{
    kNearestSearch(k_);

    std::set<Edge, CompareEdge> edges;
    for (size_t t = 0, t_end = nearest_neighbors_->rows; t < t_end; t ++) {
        for (size_t i = 0, i_end = nearest_neighbors_->cols - 1; i < i_end; i ++) {
            for (size_t j = i + 1, j_end = nearest_neighbors_->cols; j < j_end; j ++) {
                edges.insert(Edge((*nearest_neighbors_)[t][i], (*nearest_neighbors_)[t][j]));
            }
        }
    }

    for (std::set<Edge, CompareEdge>::iterator it = edges.begin(); it != edges.end(); it ++) {
        Edge edge = *it;
        DeformationGraph::Node source = (*graph_map_)[edge._source];
        DeformationGraph::Node target = (*graph_map_)[edge._target];
        deformation_graph_->addEdge(source, target);
    }
}

void PointCloud::parameterize()
{
    for (DeformationGraph::NodeIt it(*deformation_graph_); it != lemon::INVALID; ++ it) {
        parameter_map_->insert(ParameterMap::MapPair(it, Parameters()));
    }
}

void PointCloud::kNearestSearch(const int k)
{
    flann::Matrix<double> data_set(new double[node_num_ * 3], node_num_, 3);
    flann::Matrix<double> query(new double[size() * 3], size(), 3);

    // for the mapping between graph node indices and search results
    std::map<size_t, size_t> index_mapping;
    size_t i = 0;
    for (DeformationGraph::NodeIt it(*deformation_graph_); it != lemon::INVALID; ++ it, i ++) {
        Point point = at((*graph_map_)[it]);
        data_set[i][0] = point.x;
        data_set[i][1] = point.y;
        data_set[i][2] = point.z;

//         for (size_t j = 0; j < 3; j ++) {
//             Eigen::Vector3d point = at((*graph_map_)[it]);
//             data_set[i][j] = point(0, j);
//         }
        index_mapping.insert(std::pair<size_t, size_t>(i, (*graph_map_)[it]));
    }

    flann::Matrix<int> indices(new int[query.rows * k], query.rows, k);
    flann::Matrix<double> dists(new double[query.rows * k], query.rows, k);

    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<double> > index(data_set, flann::KDTreeIndexParams(4));
    index.buildIndex();
    // do a knn search, using 128 checks
    index.knnSearch(query, indices, dists, k, flann::SearchParams(128));

    // replace indices with graph node indices
    for (size_t i = 0, i_end = indices.rows; i < i_end; i ++) {
        for (size_t j = 0, j_end = indices.cols; j < j_end; j ++) {
            indices[i][j] = index_mapping[indices[i][j]];
        }
    }

    nearest_neighbors_ = new flann::Matrix<int>(indices.ptr(), indices.rows, indices.cols);
    neighbor_dists_ = new flann::Matrix<double>(dists.ptr(), dists.rows, dists.cols);
}

// remember the order of the parameters
void PointCloud::buildUnknownsMap()
{
    size_t unknown_index = 0;

    //every node has 15 parameters
    for (DeformationGraph::NodeIt it(*deformation_graph_); it != lemon::INVALID; ++ it) {

        for (size_t i = 0; i < 3; i ++) {
            for (size_t j = 0; j < 3; j ++) {
                unknowns_map_.insert(std::make_pair(unknown_index++, &((*parameter_map_)[it].affi_rot_(j, i))));
            }
        }
        for (size_t i = 0; i < 3; i ++) {
            unknowns_map_.insert(std::make_pair(unknown_index++, &((*parameter_map_)[it].affi_trans_(i))));
        }
        for (size_t j = 0; j < 3; j ++) {
            unknowns_map_.insert(std::make_pair(unknown_index++, &((*parameter_map_)[it].correspondence_(j))));
        }
    }

    // lack of rigid_rot_ unknowns

    for (size_t i = 0; i < 3; i ++) {
        unknowns_map_.insert(std::make_pair(unknown_index++, &rigid_trans_(i)));
    }

}


