#include "trains_config.h"
#include "stl_filter.h"
#include <boost/graph/adjacency_list.hpp>
#include <fstream>
#include <stdexcept>
#include <cerrno>
#include <system_error>
#include <map>
#include <set>
#include <vector>
#include <iostream>
#include <locale>
#include <algorithm>
#include <limits>

/*
// example input
3 // number of stations
A // station name
B // station name
C // station name
2 // number of routes
E1,A,B,3 // route from A to B that takes 3 units of time
E2,B,C,1 // route from B to C that takes 1 unit of time
1 // number of deliveries to be performed
P1,A,C,5 // package P1 with weight 5 located currently at station A that must be delivered to station C
1 // number of trains
Q1,B,6 // train T1 with capacity 6 located at station B
*/

class CommaDelimCType : public std::ctype<char>
{
    mask my_table[256];
public:
    CommaDelimCType(size_t refs = 0) : std::ctype<char>(my_table, false, refs)
    {
        std::copy(classic_table(), classic_table() + table_size, my_table);
        my_table[','] = (mask)space;
    }
};

size_t TrainsConfig::Route::src() const
{
    return std::min(stations[0], stations[1]);
}

size_t TrainsConfig::Route::dst() const
{
    return std::max(stations[0], stations[1]);
}

bool TrainsConfig::Route::operator == (const Route& oo) const
{
    return src() == oo.src() && dst() == oo.dst();
}

bool TrainsConfig::Route::operator < (const Route& oo) const
{
    if(src() < oo.src()) return true;
    if(src() > oo.src()) return false;
    return dst() < oo.dst();
}

void TrainsConfig::clear()
{
    stations.clear();
    deliveries.clear();
    trains.clear();
}

void TrainsConfig::load_from_file(const std::string& a_fpath)
{
    std::ifstream ff(a_fpath);
    if(!ff.good())
        throw std::system_error(errno, std::generic_category(),
            "Could not open file " + a_fpath);

    std::locale loc(std::locale::classic(), new CommaDelimCType);
    ff.imbue(loc);

    auto CheckEof = [&]()
    {
        if(ff.eof())
            throw std::runtime_error("Input file looks truncated: " + a_fpath);
    };

    clear();

    std::map<std::string, size_t> station_to_index;

    // Stations
    {
        uint32_t num_stations = 0;
        CheckEof(); ff >> num_stations;
        for(uint32_t ss = 0; ss < num_stations; ++ss)
        {
            std::string station_name;
            CheckEof(); ff >> station_name;
            const auto ip = station_to_index.insert({station_name, stations.size()});
            if(!ip.second)
                throw std::runtime_error("Duplicate station: " + station_name);
            stations.push_back(station_name);
        }
    }

    // Routes
    {
        uint32_t num_routes = 0;
        CheckEof(); ff >> num_routes;
        // E1,A,B,3 // route from A to B that takes 3 units of time
        for(uint32_t ss = 0; ss < num_routes; ++ss)
        {
            Route route;
            CheckEof(); ff >> route.route_name;
            std::string st_a;
            std::string st_b;
            CheckEof(); ff >> st_a;
            CheckEof(); ff >> st_b;
            auto ix1 = station_to_index.find(st_a);
            auto ix2 = station_to_index.find(st_b);
            if(ix1 == station_to_index.end() || ix2 == station_to_index.end() || ix1 == ix2)
                throw std::runtime_error("Bad route: " + route.route_name);
            const size_t st1 = ix1->second;
            const size_t st2 = ix2->second;
            route.stations[0] = std::min(st1, st2);
            route.stations[1] = std::max(st1, st2);
            CheckEof(); ff >> route.distance;
            auto ins_res = routes.insert(route);
            if(!ins_res.second)
                std::cerr << "Warning: parallel route dropped: "
                          << route.route_name
                          << " and "
                          << ins_res.first->route_name
                          << "\n";
        }
    }

    // Deliveries
    {
        uint32_t num_deliveries = 0;
        CheckEof(); ff >> num_deliveries;
        // P1,A,C,5 // package P1 with weight 5 located currently at station A that must be delivered to station C
        for(uint32_t ss = 0; ss < num_deliveries; ++ss)
        {
            Delivery dlv;
            CheckEof(); ff >> dlv.package_name;
            std::string st_curr;
            std::string st_dest;
            CheckEof(); ff >> st_curr;
            CheckEof(); ff >> st_dest;
            auto ix1 = station_to_index.find(st_curr);
            auto ix2 = station_to_index.find(st_dest);
            if(ix1 == station_to_index.end() || ix2 == station_to_index.end() || ix1 == ix2)
                throw std::runtime_error("Bad delivery: " + dlv.package_name);
            dlv.station_curr = ix1->second;
            dlv.station_dest = ix2->second;
            CheckEof(); ff >> dlv.weight;
            deliveries.push_back(dlv);
        }
    }

    // Trains
    {
        uint32_t num_trains = 0;
        CheckEof(); ff >> num_trains;
        // Q1,B,6 // train T1 with capacity 6 located at station B
        for(uint32_t ss = 0; ss < num_trains; ++ss)
        {
            Train train;
            CheckEof(); ff >> train.train_name;
            std::string st_curr;
            CheckEof(); ff >> st_curr;
            auto ix1 = station_to_index.find(st_curr);
            if(ix1 == station_to_index.end())
                throw std::runtime_error("Bad train: " + train.train_name);
            train.station_curr = ix1->second;
            CheckEof(); ff >> train.capacity;
            trains.push_back(train);
        }
    }
}

/*
// example input
3 // number of stations
A // station name
B // station name
C // station name
2 // number of routes
E1,A,B,3 // route from A to B that takes 3 units of time
E2,B,C,1 // route from B to C that takes 1 unit of time
1 // number of deliveries to be performed
P1,A,C,5 // package P1 with weight 5 located currently at station A that must be delivered to station C
1 // number of trains
Q1,B,6 // train T1 with capacity 6 located at station B
*/

void TrainsConfig::save_to_file(const std::string& a_fpath)
{
    std::ofstream ff(a_fpath);

    ff << stations.size() << "\n";
    for(const auto& station: stations)
        ff << station << "\n";

    ff << routes.size() << "\n";
    for(const auto& rr: routes)
        ff  << rr.route_name << ","
            << stations[rr.src()] << ","
            << stations[rr.dst()] << ","
            << rr.distance << "\n";

    ff << deliveries.size() << "\n";
    for(const auto& dd: deliveries)
        ff  << dd.package_name << ","
            << stations[dd.station_curr]  << ","
            << stations[dd.station_dest]  << ","
            << dd.weight << "\n";

    ff << trains.size() << "\n";
    for(const auto& tt: trains)
        ff  << tt.train_name << ","
            << stations[tt.station_curr] << ","
            << tt.capacity << "\n";
}

// Example of .gv file:
// graph D {
//   rankdir=LR
//   size="4,3"
//   ratio="fill"
//   edge[style="bold"]
//   node[shape="circle"]
//   A[label="A\n[P1,P2,P3]"];
//   B[label="B\n[P4,P5]"];
//   C[label="C\n[]\Q7[P8,P9]"];
//   A -- C[label="1\nQ1[P1,P2]\nQ2[P3]", color="black"]
//   B -- B[label="2", color="grey"]
//   B -- D[label="1", color="#FF76FF"]
//   B -- E[label="2", color="grey"]
//   C -- B[label="7", color="grey"]
//   C -- D[label="3", color="black"]
//   D -- E[label="1", color="black"]
//   E -- A[label="1", color="grey"]
//   E -- B[label="1", color="black"]
// }
void TrainsConfig::save_graphviz_dot_file(const std::string& fpath, RenderOpts opts)
{
    std::ofstream ff(fpath);
    // https://en.wikipedia.org/wiki/DOT_(graph_description_language)
    // https://www.graphviz.org/doc/info/shapes.html#html
    ff  << R"(graph trains_config { )" << '\n';
    ff  << R"(  node[shape="rect"] )" << '\n';
    //ff  << R"(  rankdir=LR )" << '\n';
    //ff  << R"(  size="4,3" )" << '\n';
    //ff  << R"(  ratio="fill" )" << '\n';
    //ff  << R"(  edge[style="bold"] )" << '\n';

    // Vertices = stations
    // e.g. A[label="A\n[P1,P2,P3]"];
    for(size_t ss = 0; ss < stations.size(); ++ss)
    {
        auto const& station_id = ss;
        auto const& station_label = stations[ss];
        ff << "  " << station_id << "[label=\"" << station_label;

        // cargo at this station
        if(opts.show_deliveries)
        {
            const auto deliveries_here = stl_filter(deliveries,
                [=](const Delivery& del) {return del.station_curr == ss;});
            if(opts.show_empty || deliveries_here.size())
            {
                ff << "\n"; // {
                for(size_t dd = 0; dd < deliveries_here.size(); ++dd)
                {
                    auto const& delivery = deliveries_here[dd];
                    ff << (dd ? ", " : "") << delivery.package_name;
                    const auto& dest = stations[delivery.station_dest];
                    if(opts.show_details)
                        ff << "(>" << dest << "," << delivery.weight << ")";
                }
                ff << ""; // }
            }
        }

        // trains at this station
        if(opts.show_trains)
        {
            const auto trains_here = stl_filter(trains,
                [=](const Train& trn) {return trn.station_curr == ss;});
            if(opts.show_empty || trains_here.size())
            {
                ff << "\n"; // {
                for(size_t tt = 0; tt < trains_here.size(); ++tt)
                {
                    auto const& train = trains_here[tt];
                    ff << (tt?", ":"");
                    ff << train.train_name;
                    if(opts.show_details)
                        ff << "(" << train.capacity << ")";
                }
                ff << ""; // }
            }
        }
        ff << "\"];\n";
    }

    // Edges = routes
    // C -- D[label="3", color="black"]
    for(const auto& route : routes)
    {
        const size_t s0_id = route.stations[0];
        const size_t s1_id = route.stations[1];
        ff << "  " << s0_id << " -- " << s1_id;
        ff << " [label=\"";
        ff << route.distance;
        if(opts.show_route_names)
            ff << " (" << route.route_name << ")";
        ff << "\"]\n";
    }
    ff  << "}\n";
}
