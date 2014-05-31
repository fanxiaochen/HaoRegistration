#ifndef TYPES_H
#define TYPES_H
#include <pcl/point_types.h>
#include <Eigen/Dense>


typedef pcl::PointXYZRGBNormal Point;

static Eigen::Vector3d EIGEN_POINT_CAST(const Point &point)
{
    Eigen::Vector3d vector;
    vector(0) = point.x;
    vector(1) = point.y;
    vector(2) = point.z;
    return vector;
}

static Point POINT_EIGEN_CAST(const Eigen::Vector3d &vector)
{
    Point point;
    point.x = vector(0);
    point.y = vector(1);
    point.z = vector(2);
    return point;
}
#endif //TYPES_H
