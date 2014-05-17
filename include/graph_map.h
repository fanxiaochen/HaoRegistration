#ifndef GRAPH_MAP_H
#define GRAPH_MAP_H
#include <map>
#include <string>

#include <lemon/list_graph.h>

// using as one to one mapping
class GraphMap
{
public:
    class MapPair
    {
    public:
        MapPair();

        MapPair(lemon::ListGraph::Node key, int value) {
            _key = key;
            _value = value;
        }

    public:
        lemon::ListGraph::Node _key;
        int _value;
    };

public:
    typedef lemon::ListGraph::NodeIt Iterator;
    typedef lemon::ListGraph::Node Key;
    typedef int Value;

public:
    GraphMap(lemon::ListGraph *graph);
    ~GraphMap();

    void insert(MapPair map_pair);

    Value operator[](const Iterator &it);
    Value operator[](const Key &k);
    Key operator[](const Value &v);

private:
    lemon::ListGraph *_graph;

    lemon::ListGraph::NodeMap<Value> _key_map;
    std::map<Value, Key> _value_map;
};


#endif //GRAPH_MAP_H
