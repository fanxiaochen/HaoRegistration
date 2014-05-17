#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Eigen/Dense>
#include <lemon/list_graph.h>

class Parameters
{
public:
    Parameters();
    ~Parameters();
    
private:
    Eigen::Matrix3d affi_rot_;
    Eigen::Vector3d affi_trans_;
    Eigen::Vector3d correspondence_;
    Eigen::Vector3d weights_;
};
#endif //PARAMETERS_H
