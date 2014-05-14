#include "graph_map.h"

GraphMap::GraphMap()
{
    
}

GraphMap::~GraphMap()
{
    
}

void GraphMap::insert(MapPair map_pair)
{
    _key_map.insert(std::pair<const GraphMap::Key, GraphMap::Value>(map_pair._key, map_pair._value));
    _value_map.insert(std::pair<const GraphMap::Value, GraphMap::Key>(map_pair._value, map_pair._key));
}

GraphMap::Value GraphMap::operator()(const Key& k, std::string key)
{
    if (key.compare("key") == 0){
        return _key_map[k];
    }
    else{
        return -1;
    }
}

GraphMap::Key GraphMap::operator()(const Value& v, std::string value)
{
    if (value.compare("value") == 0){
        return _value_map[v];
    }
    else{
        return GraphMap::Key();  //an empty object
    }
}

