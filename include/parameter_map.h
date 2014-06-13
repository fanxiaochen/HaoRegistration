#ifndef PARAMETER_MAP_H
#define PARAMETER_MAP_H

#include <map>
#include <Eigen/Dense>
#include <lemon/list_graph.h>

struct Parameters {
public:
    Parameters() {
        affi_rot_.setZero();
        affi_trans_.setZero();
        correspondence_.setZero();
    }
    
public:
    void print(){
        std::cout << "Affine Rotation: " << std::endl;
        std::cout << affi_rot_ << std::endl;
        
        std::cout << "Affine Translation: " << std::endl;
        std::cout << affi_trans_ << std::endl;
        
        std::cout << "Correspondence: " << std::endl;
        std::cout << correspondence_ << std::endl;  
        
        std::cout << std::endl;
    }

public:
    Eigen::Matrix3d affi_rot_;
    Eigen::Vector3d affi_trans_;
    Eigen::Vector3d correspondence_;  //put the weight into the third double in correspondence
};

class ParameterMap
{
public:
    class MapPair
    {
    public:
        MapPair();

        MapPair(lemon::ListGraph::Node key, Parameters value) {
            _key = key;
            _value = value;
        }

    public:
        lemon::ListGraph::Node _key;
        Parameters _value;
    };

public:
    typedef lemon::ListGraph::NodeIt Iterator;
    typedef lemon::ListGraph::Node Key;
    typedef Parameters Value;

public:
    ParameterMap(lemon::ListGraph *graph);
    ~ParameterMap();

    void insert(MapPair map_pair);

    Value &operator[](const Iterator &it);
    Value &operator[](const Key &k);

private:
    lemon::ListGraph *_graph;

    lemon::ListGraph::NodeMap<Value> _parameter_map;
};

#endif //PARAMETER_MAP_H
