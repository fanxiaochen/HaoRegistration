#include "graph_map.h"

GraphMap::GraphMap(lemon::ListGraph* graph)
:_graph(graph),
 _key_map(*graph)
{
    
}

GraphMap::~GraphMap()
{
    
}

void GraphMap::insert(MapPair map_pair)
{
    _key_map[map_pair._key] = map_pair._value;
    _value_map.insert(std::pair<const GraphMap::Value, GraphMap::Key>(map_pair._value, map_pair._key));
}

GraphMap::Value GraphMap::operator[](const Key& k)
{
    return _key_map[k];
}

GraphMap::Key GraphMap::operator[](const Value& v)
{
    return _value_map[v];
}

GraphMap::Value GraphMap::operator[](const Iterator& it)
{
    return _key_map[it];
}

