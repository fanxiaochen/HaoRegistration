#ifndef PARAMETER_MAP_H
#define PARAMETER_MAP_H

#include <map>

#include "parameters.h"

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

    Value operator[](const Iterator &it);
    Value operator[](const Key &k);

private:
    lemon::ListGraph *_graph;

    lemon::ListGraph::NodeMap<Value> _parameter_map;
};

#endif //PARAMETER_MAP_H
