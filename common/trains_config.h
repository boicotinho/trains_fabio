#pragma once
//#include <boost/graph/adjacency_list.hpp>
#include <string>
#include <vector>
#include <set>
#include <stdint.h>

using distance_t = uint32_t;
using weight_t   = uint32_t;
/*
/// Choosing \b Edge/Vertex-types
// https://cs.brown.edu/~jwicks/boost/libs/graph/doc/using_adjacency_list.html#sec:choosing-graph-type
using Graph = boost::adjacency_list
                < boost::vecS           // OutEdgeListS (listS?)
                , boost::vecS           // VertexListS
                , boost::undirectedS    // DirectedS
                //, boost::no_property  // VertexProperty
                , boost::property<      // VertexProperty
                    boost::vertex_index_t, uint32_t
                    >
                , boost::property<      // EdgeProperty
                    boost::edge_weight_t, distance_t
                    >
                >;
*/
struct RenderOpts
{
    bool show_trains      {true};
    bool show_deliveries  {true};
    bool show_route_names {false};
    bool show_empty       {false};
    bool show_details     {true}; // train capacity, letter weight and destination
};

struct TrainsConfig
{
public:
    TrainsConfig() = default;
    explicit TrainsConfig(const std::string& fpath) {load_from_file(fpath);}
    void load_from_file(const std::string& fpath);
    void save_to_file(const std::string& fpath);
    void save_graphviz_dot_file(const std::string& fpath, RenderOpts = {});
    void clear();

    using Station = std::string; // Vertex

    struct Route // Edge
    {
        std::string route_name;
        size_t      stations[2];
        distance_t  distance;
        size_t src() const;
        size_t dst() const;
        bool operator == (const Route& oo) const;
        bool operator < (const Route& oo) const;
    };

    struct Delivery
    {
        std::string package_name;
        size_t      station_curr;
        size_t      station_dest;
        weight_t    weight;
    };

    struct Train
    {
        std::string train_name;
        size_t      station_curr;
        weight_t    capacity;
    };

    std::vector<Station>    stations;
    std::set<Route>         routes;
    std::vector<Delivery>   deliveries;
    std::vector<Train>      trains;
};
