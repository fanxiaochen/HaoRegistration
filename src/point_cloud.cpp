
#include <opencv2/highgui/highgui.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud.h"


PointCloud::PointCloud()
{
    node_num_ = 0;
    rigid_rot_.setZero();
    rigid_trans_.setZero();
    mass_center_.setZero();
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
}

void PointCloud::load(const std::string &file, bool flag)
{
    depth_map_ = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH);  // Read the file
    depth_map_.convertTo(depth_map_, CV_32F); // convert the image data to float type

    is_dense = false;

    // if true, the point cloud has depth map structure
    if (flag) {
        width = depth_map_.cols;
        height = depth_map_.rows;
    }

    // from depth map to point cloud, (u, v) -> (x, y ,z)
    const float z_threshold = 2.0f; // for specified dataset
    for (int u = 0; u < depth_map_.rows; ++u) {
        for (int v = 0; v < depth_map_.cols; ++v) {
            Point pt;
            pt.z = depth_map_.at<float>(u, v) * 0.001f; // mm -> m
            if (flag == false && pt.z > z_threshold) {
                continue;
            }
            // coordinate system, from upper-left to center in image plane
            // then plane to 3D
            pt = getPointFromDepthMap(u, v);
            push_back(pt);
        }
    }

    sensor_origin_.setZero();
    sensor_orientation_.w() = 0.0f;
    sensor_orientation_.x() = 0.0f;
    sensor_orientation_.y() = 0.0f;
    sensor_orientation_.z() = 1.0f;
//
//     //  PointCloud::print(this);
//
//     //  evaluateNormal();
    evaluateMassCenter();
}

Point PointCloud::getPointFromDepthMap(int u, int v)
{
    // from depth map to point cloud, (u, v) -> (x, y ,z)
    const int scale = 100;  // scale the raw data
    float constant = 575.8; // kinect focal length: 575.8

    Point pt;
    pt.z = depth_map_.at<float>(u, v) * 0.001f; // mm -> m

    // coordinate system, from upper-left to center in image plane
    // then plane to 3D
    pt.x = (v - float(depth_map_.cols) / 2) * pt.z / constant;
    pt.y = (float(depth_map_.rows) / 2 - u) * pt.z / constant;

    pt.z *= scale;
    pt.x *= scale;
    pt.y *= scale;

    return pt;
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

    for (size_t i = 0, i_end = size(); i < i_end; i ++) {
        at(i).normal_x = cloud_normals->at(i).normal_x;
        at(i).normal_y = cloud_normals->at(i).normal_y;
        at(i).normal_z = cloud_normals->at(i).normal_z;
    }
}

void PointCloud::evaluateMassCenter()
{
    Point center;
    for (size_t i = 0, i_end = size(); i < i_end; i ++) {
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

void PointCloud::smoothDependency()
{
    const int k = k_ + 1;
    kNearestSearch(k);
    dependency_weights_.resize(size(), k_);

    for (size_t j = 0, j_end = size(); j < j_end; j ++) {
        double d_max = (*neighbor_dists_)[j][k - 1]; // last one is the max?
        const Point &point = at(j);
        double numerator[k_];
        double denominator = 0;
        for (size_t i = 0; i < k_; i ++) {
            const Point &node = at((*nearest_neighbors_)[j][i]);
            Eigen::Vector3d vector = EIGEN_POINT_CAST(point) - EIGEN_POINT_CAST(node);
            numerator[i] = 1 - vector.norm() / d_max;
            denominator += numerator[i];
        }

        for (size_t i = 0; i < k_; i ++) {
            dependency_weights_(j, i) = numerator[i] / denominator;
        }
    }
}

Point PointCloud::localTransform(size_t j)
{
    Eigen::Vector3d vector;
    vector.setZero();
    const Point &point = at(j);
    Eigen::Vector3d eigen_point = EIGEN_POINT_CAST(point);
    for (size_t i = 0; i < k_; i ++) {
        int pt_idx = (*nearest_neighbors_)[j][i];
        const Point &node = at(pt_idx);
        Eigen::Vector3d eigen_node = EIGEN_POINT_CAST(node);
        Parameters parameters = (*parameter_map_)[(*graph_map_)[pt_idx]]; // too complicated map index...
        vector += dependency_weights_(j, i) * parameters.affi_rot_ * (eigen_point - eigen_node) +
                  eigen_node + parameters.affi_trans_;
    }
    return POINT_EIGEN_CAST(vector);
}


Point PointCloud::globalTransform(const Point &point)
{
    Eigen::Matrix3d rotation;
    double theta = rigid_rot_(0);
    double x = rigid_rot_(1);
    double y = rigid_rot_(2);
    double z = sqrt(1 - x * x - y * y);

    double r[9]; // rotation matrix in axis-angle form
    r[0] = cos(theta) + (double(1) - cos(theta)) * x * x;
    r[1] = (double(1) - cos(theta)) * y * x + sin(theta) * z;
    r[2] = (double(1) - cos(theta)) * z * x - sin(theta) * y;
    r[3] = (double(1) - cos(theta)) * x * y - sin(theta) * z;
    r[4] = cos(theta) + (double(1) - cos(theta)) * y * y;
    r[5] = (double(1) - cos(theta)) * z * y + sin(theta) * x;
    r[6] = (double(1) - cos(theta)) * x * z + sin(theta) * y;
    r[7] = (double(1) - cos(theta)) * y * z - sin(theta) * x;
    r[8] = cos(theta) + (double(1) - cos(theta)) * z * z;

    rotation << r[0], r[3], r[6],
             r[1], r[4], r[7],
             r[2], r[5], r[8];

    Eigen::Vector3d eigen_point = EIGEN_POINT_CAST(point);
    Eigen::Vector3d tran_point = rotation * (eigen_point - mass_center_) + mass_center_ + rigid_trans_;
    return POINT_EIGEN_CAST(tran_point);
}

void PointCloud::transform()
{
    for (size_t j = 0, j_end = size(); j < j_end; j ++) {
        // how about normals?
        transform(j);
    }
}

void PointCloud::transform(size_t index)
{
    // normals, colors...
    Point point = globalTransform(localTransform(index));
    at(index).x = point.x;
    at(index).y = point.y;
    at(index).z = point.z;
}

void PointCloud::update()
{
    for (size_t j = 0, j_end = size(); j < j_end; j ++) {
        // how about normals?
        Point point = localTransform(j);
        at(j).x = point.x;
        at(j).y = point.y;
        at(j).z = point.z;
    }
    return;
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
        Point &point = at((*graph_map_)[it]);
        data_set[i][0] = point.x;
        data_set[i][1] = point.y;
        data_set[i][2] = point.z;

//         for (size_t j = 0; j < 3; j ++) {
//             Eigen::Vector3d point = at((*graph_map_)[it]);
//             data_set[i][j] = point(0, j);
//         }
        index_mapping.insert(std::pair<size_t, size_t>(i, (*graph_map_)[it]));
    }

    for (size_t j = 0, j_end = size(); j < j_end; j ++) {
        Point &point = at(j);
        query[j][0] = point.x;
        query[j][1] = point.y;
        query[j][2] = point.z;
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

void PointCloud::setColor(size_t r, size_t g, size_t b)
{
    for (size_t i = 0, i_end = size(); i < i_end; i ++) {
        Point &point = at(i);
        point.r = r;
        point.g = g;
        point.b = b;
        point.a = 255;
    }
    return;
}

void PointCloud::getCorrespondenceByKnn(pcl::PointCloud<Point>::Ptr target_knn, PointCloud *target)
{
    int rows = target->getDepthMap().rows;
    int cols = target->getDepthMap().cols;

    // Attention here: when deleting the smart pointer, the target will also be released!
    // that's why I copied the target
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(target_knn);

    // K nearest neighbor search

    int K = 1;

    for (PointCloud::DeformationGraph::NodeIt it(*deformation_graph_); it != lemon::INVALID; ++ it) {
        Parameters &paras = (*parameter_map_)[it];
        Point &point = at((*graph_map_)[it]);

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            Point &corres = target_knn->at(pointIdxNKNSearch[0]);

            // from (x, y ,z) to (u, v)
            float constant = 575.8; // kinect focal length: 575.8
            double u, v;
            v = (corres.x * constant) / corres.z + cols /  2;
            u = rows / 2 - (corres.y * constant) / corres.z;

            paras.correspondence_[0] = u;
            paras.correspondence_[1] = v;
        }
    }

    return;
}



