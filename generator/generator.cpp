#include "common/trains_config.h"
#include "rand_int_generator.h"
#include <map>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include <x86intrin.h>

// Random distribution specs for TRAINS problem generator
struct GenArgs
{
    ssize_t     seed            {0}; // if zero then use current time stamp

    RandArgs    num_stations    {10, 5, 2, 20};

    RandArgs    num_routes      {3.0, 2.0, 1, 4}; // per station
    RandArgs    route_len       {5.0, 3.0, 1, 10};

    RandArgs    num_trains      { 3, 3, 1, 6};
    RandArgs    train_capacity  {10, 2, 1, 20};

    RandArgs    num_packages    {10, 3, 1, 10};
    RandArgs    package_weight  { 3, 5, 1, 20};
};

// TRAINS problem generator.
TrainsConfig generate_cfg(const GenArgs& args = {})
{
    TrainsConfig cfg;
    std::default_random_engine reng {args.seed ? args.seed : __rdtsc()};

    // Create stations/vertices S0...SN-1
    {
        const size_t num_stations = std::max<size_t>(2, RandIntGenerator(reng, args.num_stations)());
        for(size_t src_station = 0; src_station < num_stations; ++src_station)
        {
            cfg.stations.emplace_back("S" + std::to_string(src_station));
        }
    }

    RandIntGenerator rand_station (reng, RandArgs{-1, -1, 0, (ssize_t)cfg.stations.size()-1});

    // Trains
    size_t max_train_capacity = 0;
    {
        const size_t num_trains = RandIntGenerator(reng, args.num_trains)();
        RandIntGenerator rand_train_capacity (reng, args.train_capacity);
        for(size_t tt = 0; tt < num_trains; ++tt)
        {
            cfg.trains.emplace_back();
            auto& train        = cfg.trains.back();
            train.train_name   = "Q" + std::to_string(tt);
            train.station_curr = rand_station();
            train.capacity     = rand_train_capacity();
            max_train_capacity = std::max<size_t>(max_train_capacity, train.capacity);
        }
    }

    // Deliveries/Packages
    {
        const size_t num_packages = RandIntGenerator(reng, args.num_packages)();
        RandArgs pkg_w = args.package_weight;
        pkg_w.max = std::min<ssize_t>(pkg_w.max, max_train_capacity);
        RandIntGenerator rand_package_weight(reng, pkg_w);
        for(size_t pp = 0; pp < num_packages; ++pp)
        {
            TrainsConfig::Delivery pkg;
            pkg.package_name = "P" + std::to_string(pp);
            pkg.station_curr = rand_station();
            do {
                pkg.station_dest = rand_station();
            } while(pkg.station_dest == pkg.station_curr);
            pkg.weight = rand_package_weight();
            cfg.deliveries.push_back(pkg);
        }
    }

    // Create routes/edges
    {
        RandIntGenerator rand_num_routes (reng, args.num_routes);
        RandIntGenerator rand_route_len  (reng, args.route_len);
        // First determine how many edges/routes each station should (still) target.
        // Later, as we route to/from a station, we decrease the counter for the station.
        std::vector<ssize_t> tgt_routes_at_station(cfg.stations.size());
        for(ssize_t& tgt: tgt_routes_at_station)
            tgt = rand_num_routes();

        // Used for naming routes
        size_t routename_counter = 0;
        auto insert_new_route = [&](size_t station_a, size_t station_b)
        {
            if(station_a == station_b)
                throw std::logic_error("station_a == station_b");
            TrainsConfig::Route route;
            // Try insert unamed route; equality / sorting is determined on stations alone
            route.stations[0] = station_a;
            route.stations[1] = station_b;
            auto res = cfg.routes.insert(route);
            if(!res.second)
                return false; // route already exists, with src/dst reversed
            auto& rr = const_cast<TrainsConfig::Route&>(*res.first);
            rr.route_name = "E" + std::to_string(routename_counter++);
            rr.distance   = rand_route_len();
            tgt_routes_at_station[station_a]--;
            tgt_routes_at_station[station_b]--;
            return true;
        };

        // Second, spend up 2 routes from each station connect all nodes forming a circle.
        // This ensures we don't have a split graph.
        for(size_t src_station = 0; src_station < cfg.stations.size(); ++src_station)
        {
            size_t const dst_station = (src_station + 1) % cfg.stations.size();
            insert_new_route(src_station, dst_station);
        }

        // Now insert additional routes
        for(size_t src_station = 0; src_station < cfg.stations.size(); ++src_station)
        {
            for( size_t neighbour = 1
                ; neighbour < cfg.stations.size() && tgt_routes_at_station[src_station] > 0
                ; neighbour++
                )
            {
                size_t const dst_station = (src_station + neighbour) % cfg.stations.size();
                if(tgt_routes_at_station[dst_station] < 1)
                    continue; // dst station is already at target num routes, pick another.
                insert_new_route(src_station, dst_station);
            }
        }
    }

    return cfg;
}

int main(int argc, const char* argv[])
{
    if(argc < 2 || argc > 3)
    {
        fprintf(stderr, "Syntax: \n  generator <destination_file> [seed=0]\n");
        return EXIT_FAILURE;
    }

    GenArgs args;

    const std::string fpath = argv[1];
    if(argc > 2)
        args.seed = std::atoi(argv[2]);

    TrainsConfig cfg = generate_cfg(args);
    cfg.save_to_file(fpath);
    cfg.save_graphviz_dot_file(fpath+".gv");

    return EXIT_SUCCESS;
}
