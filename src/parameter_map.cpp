#include "parameter_map.h"

Parameters::Parameters()
{
    affi_rot_.setZero();
    affi_trans_.setZero();
    correspondence_.setZero();
    weight_ = 0;
}

void Parameters::mapToUnknowns(Eigen::VectorXd &unknowns, size_t index)
{
    for (size_t i = 0; i < 3; i ++) {
        for (size_t j = 0; j < 3; j ++) {
            unknowns[index ++] = affi_rot_(j, i);
        }
    }
    for (size_t i = 0; i < 3; i ++) {
        unknowns[index ++] = affi_trans_(0, i);
    }
    for (size_t j = 0; j < 2; j ++) {
        unknowns[index ++] = correspondence_(0, j);
    }
    unknowns[index ++] = weight_;
}

void Parameters::mapToTransforms(const Eigen::VectorXd &unknowns, size_t index)
{
    for (size_t i = 0; i < 3; i ++) {
        for (size_t j = 0; j < 3; j ++) {
            affi_rot_(j, i) = unknowns[index ++];
        }
    }
    for (size_t i = 0; i < 3; i ++) {
        affi_trans_(0, i) = unknowns[index ++];
    }
    for (size_t j = 0; j < 2; j ++) {
        correspondence_(0, j) = unknowns[index ++];
    }
    weight_ = unknowns[index ++];
}



ParameterMap::ParameterMap(lemon::ListGraph *graph)
    : _graph(graph),
      _parameter_map(*graph)
{

}

ParameterMap::~ParameterMap()
{

}

void ParameterMap::insert(MapPair map_pair)
{
    _parameter_map[map_pair._key] = map_pair._value;
}

ParameterMap::Value ParameterMap::operator[](const Key &k)
{
    return _parameter_map[k];
}

ParameterMap::Value ParameterMap::operator[](const Iterator &it)
{
    return _parameter_map[it];
}
