#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <vector>

#include <Eigen/Dense>

class PointCloud{
public:
    PointCloud();
    ~PointCloud();
    
private:
    std::vector<Eigen::Vector3d> point_cloud_; 
};

#endif //POINT_CLOUD_H