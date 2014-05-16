#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <flann/flann.hpp>
#include <lemon/list_graph.h>

class Parameters{
public:
    Parameters();
    ~Parameters();
private:
    lemon::ListGraph::Node node_;
    
    flann::Matrix<double> affi_rot_;
    flann::Matrix<double> affi_trans_;
};
#endif //PARAMETERS_H