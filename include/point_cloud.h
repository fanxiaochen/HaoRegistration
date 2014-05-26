#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <lemon/list_graph.h>
#include <flann/flann.hpp>

#include "graph_map.h"
#include "parameter_map.h"

typedef pcl::PointXYZRGBNormal Point;

class PointCloud: public pcl::PointCloud<Point>
{
public:
    typedef lemon::ListGraph DeformationGraph;

public:
    PointCloud();
    virtual ~PointCloud();

    void binding();

    void setNodeNum(size_t node_num);
    inline size_t getNodeNum() {
        return node_num_;
    }

private:
    virtual void sampling();
    virtual void connecting();
    virtual void parameterize();

    void kNearestSearch(const int &k);


private:
    size_t node_num_;

    DeformationGraph *deformation_graph_;
    GraphMap *graph_map_;
    ParameterMap *parameter_map_;

    flann::Matrix<int> *nearest_neighbors_;

    Eigen::Matrix3d rigid_rot_;
    Eigen::Vector3d rigid_trans_;
};

// to remove same edges because lemon can only support the mode of adding edges between same nodes
struct Edge {
    Edge(int source, int target): _source(source), _target(target) {}
    int _source;
    int _target;
};

// compare function for user-defined std::set
// must have strict weak ordering
struct CompareEdge {
    bool operator()(const Edge &edge1, const Edge &edge2) {

        if ((edge1._source == edge2._source && edge1._target == edge2._target)
                || (edge1._source == edge2._target && edge1._target == edge2._source)) {
            return false;
        }

        if ((edge1._source == edge2._source && edge1._target < edge2._target)
                || (edge1._source < edge2._source)) {
            return true;
        }

        if ((edge1._source == edge2._source && edge1._target > edge2._target)
                || (edge1._source > edge2._source)) {
            return false;
        }
    }
};

#endif //POINT_CLOUD_H
