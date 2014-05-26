#include "parameter_map.h"

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
