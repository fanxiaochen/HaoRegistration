#ifndef GRAPH_MAP_H
#define GRAPH_MAP_H
#include <map>
#include <string>

#include <lemon/list_graph.h>


typedef struct {
    lemon::ListGraph::Node _key;
    int _value;
}MapPair;

// using as one to one mapping
class GraphMap{
public:
    typedef lemon::ListGraph::Node Key;
    typedef int Value;
    
public:
    GraphMap();
    ~GraphMap();
    
    void insert(MapPair map_pair);
    
    Value operator()(const Key& k, std::string key);
    Key operator()(const Value& v, std::string value);
    
private:
    std::map<Key, Value> _key_map;
    std::map<Value, Key> _value_map;   
};


#endif //GRAPH_MAP_H