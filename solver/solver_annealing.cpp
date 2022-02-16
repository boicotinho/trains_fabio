#include "common/trains_config.h"
#include "common/t_uint.h"
#include "common/array_sz.h"
#include <boost/intrusive/set.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool.hpp>
#include <limits>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <bitset>
#include <cassert>

namespace intr = boost::intrusive;

constexpr size_t cx_max(size_t a, size_t b) {return a > b ? a : b;}

// Using tight constant sizes so we can use std::array, 
// reducing number of heap allocations (was consting a lot)
constexpr int PMAX          = 4; //10; // number of bits repr. number of packages (1024)
constexpr int QMAX          = 3; // 7; // number of bits repr. number of trains   (127)
constexpr int SMAX          = 4; // 7; // number of bits repr. number of stations (127)
constexpr int RMAX          = 3; // 3; // number of bits repr. number of routes per station (8)
constexpr int EMAX          = 4; // 8; // number of bits repr. maximum number of routes globally (256)
constexpr int WMAX          = 4; // 8; // number of bits repr. weight of single edge/route  (256)
constexpr int MAX_PATH_BITS = 15; //15; // maximum path weight (sum of edge weights) for maximum time eval (32768)
/*
// TODO: replace MAX_VAL with NUM_BITS, for bit fields
template< int PMAX // number of bits number of packages
        , int QMAX // number of bits number of trains
        , int SMAX // number of bits number of stations
        , int RMAX // number of bits number of routes per station
        , int WMAX // number of bits weight of single edge/route
        , int MAX_PATH_BITS // maximum path weight (sum of edge weights) for maximum time eval
        > */
struct SASolver
{
    constexpr static size_t MAX_PACKAGES            = 1ul << PMAX; // 1024
    constexpr static size_t MAX_TRAINS              = 1ul << QMAX; // 128
    constexpr static size_t MAX_STATIONS            = 1ul << SMAX; // 128
//  constexpr static size_t MAX_ROUTES_PER_STATION  = 1ul << RMAX;
//  constexpr static size_t MAX_EDGE_WEIGHT         = 1ul << WMAX;
//  constexpr static size_t MAX_ROUTES_GLOBALLY     = 1ul << MAX_PATH_BITS;
    using package_id = uintn<PMAX>;
    using train_id   = uintn<QMAX>;
    using station_id = uintn<SMAX>;
    using lroute_ix  = uintn<RMAX>; // local route index, within station only
    using groute_id  = uintn<EMAX>; // global route id (for fetching weight info)
    using weight_t   = uintn<WMAX>;
    using distance_t = uintn<MAX_PATH_BITS>;
    using strat_rank_t = uint8_t;

    enum : groute_id  { NO_GROUTE = std::numeric_limits<groute_id>::max() };
    enum : package_id { NO_PACKAGE = std::numeric_limits<package_id>::max() };
    enum : station_id { NO_STATION = std::numeric_limits<station_id>::max() };

    using timepoint_t = uintn<MAX_PATH_BITS>;
    class Evaluation // 2 bytes. the lower the better
    {
        using eval_t = uintn<MAX_PATH_BITS+1>;
        constexpr static eval_t worst() {return std::numeric_limits<eval_t>::max() >> 1;}
        eval_t m_is_solution  : 1;
        eval_t m_eval         : MAX_PATH_BITS;
    public:
        constexpr Evaluation() : m_is_solution(0), m_eval(worst()) {}
        explicit Evaluation(size_t vv) : m_is_solution(0), m_eval(vv) {assert(vv <= worst());}
        bool operator< (const Evaluation& oo) const
            {
            assert(m_is_solution == oo.m_is_solution);
            return m_eval < oo.m_eval;
            }
        bool operator == (const Evaluation& oo) const
            {
            assert(m_is_solution == oo.m_is_solution);
            return m_eval == oo.m_eval;
            }
        Evaluation& operator += (size_t const vv)
            {
            assert(m_eval + vv <= worst());
            m_eval += vv;
            return *this;
            }
        void set_as_solution() {m_is_solution = 1;}
        bool is_solution() const {return m_is_solution;}
        bool was_evaluated() const {return m_eval != worst();}
    };

    // Coordinate contains fractional progress between source and destination station,
    // because we want to be able to evaluate for any given time step.
    struct Coordinate // size = 2 bytes
    {
        //station_id  station_src;
        station_id  station_dst {NO_STATION};
        weight_t    eta         {0}; // how many time units until arrived at destination
    };

    class TrainOrStationId // size = 1 byte
    {
        enum { BITS = cx_max(SMAX,QMAX) };
        using val_t = uintn< BITS >;
        union {
            struct {
                val_t trn : 1;
                val_t val : BITS;
            };
            val_t combined;
        };
    public:
        constexpr TrainOrStationId() : trn(0), val(std::numeric_limits<val_t>::max()>>1) {}
        constexpr TrainOrStationId(val_t vv, bool train) : trn(train), val(vv) {}
        constexpr bool is_train() const {return trn;}
        constexpr bool is_station() const {return !is_train();}
        train_id as_train() const {assert(is_train()); return val;}
        station_id as_station() const {assert(is_station()); return val;}
        void assign_train(train_id id) {trn = 1; val = id;}
        void assign_station(station_id id) {trn = 0; val = id;}
        constexpr bool operator == (const TrainOrStationId& oo) const {return combined == oo.combined;}
        struct Hasher
            { 
            std::size_t operator()(const TrainOrStationId& key) const
                {return ::std::hash<val_t>()(key.combined);}
            };    
    };


    // For a state, we want to know how far all packages are from delivery, so that we can score two unsolved states.
    // Optionally, we also want to create some gravity force pulling empty trains towards stationed cargo -
    //      and for that we need to know where each empty train is.
    // Ideally, we don't want the scoring of a State to be too expensive.
    // Ideally, we also don't want a State to take too much space because we'd like to keep many alive.
    struct State // size = 127*1 + 127*2 = 381 bytes
    {
        std::array<Coordinate, MAX_TRAINS> train_2_coord; // [train_id] -> Coordinate
        using PackageList = array_sz< package_id, MAX_PACKAGES >;
        std::unordered_map< TrainOrStationId
                          , PackageList
                          , TrainOrStationId::Hasher
                          > train_2_packagelist;
    };

    struct Strategy // 2 bytes
    {
        enum class eMoveStrat : uint8_t // 1 byte
        {
            LeadingPackage,   // select a leading package, then chose a route that improves score for it, then load all packages which also improves the score
            WaitForNextTrain, // maybe only trains which have started moving towards a letter station from adjacent station
            COUNT
        };
        constexpr static strat_rank_t RANK_END = 4;
        eMoveStrat   strat_move {}; // 1 byte
        strat_rank_t strat_rank {}; // 1 byte
        Strategy& operator++() 
            {
            if(++strat_rank >= RANK_END)
                return *this;
            strat_rank = 0;
            strat_move = static_cast<eMoveStrat>(static_cast<int>(strat_move)+1); 
            return *this;
            }
    };

    // An action is a valid permutation that can be performed on a state. (SOBOL-like for threading?)
    struct Action // size = 2 + 128 * (1 or 2) = 130 or 258 bytes
    {
        Strategy                          move_strat {};
        std::array< groute_id,MAX_TRAINS> route_selected_by_train; // [train_id] -> global route/vertex
        Action() {route_selected_by_train.fill(NO_GROUTE);}
    };


    struct PathNode;
    using PathCompare = intr::compare< std::less<PathNode> >;
    using IntrSetBase = intr::set_base_hook< intr::optimize_size<true> >;
    using NodeIntrSet = intr::set< PathNode, PathCompare >;
    using GlobSetHook = intr::set_member_hook< intr::optimize_size<true> >; // for global set of solutions
    struct PathNode : public IntrSetBase // size = 1520 (IntrSetBase 24 bytes)
    {
        // Tree linking
        bool operator < (const PathNode& oo) const {return score < oo.score;} // children sorted in NodeIntrSet[]
        const PathNode*     parent_ptr {nullptr};
        GlobSetHook         intr_global_solution_hook; // 24 bytes
        mutable NodeIntrSet children;      // 32 bytes
        mutable Strategy    children_next_strategy;  // 2 bytes. strategy to be used for selecting next child

        timepoint_t base_tp {0};    // 2 bytes. timestamp when state was evaluated, TODO: what if multiple trains in the same time frame?
        Evaluation  score {};       // 2 bytes. evaluation score at time base_tp
        State       state;          // 381 bytes
        //Action    parent_action;  // 13/258 bytes. permutation/action on parent state that lead to tis node's state. TODO: can be removed? used only for checking if permutation was already explored before
    };

    using GlobSetMember = intr::member_hook
                            < PathNode
                            , GlobSetHook
                            , &PathNode::intr_global_solution_hook
                            >;
    using GlobalSet     = intr::set
                            < PathNode
                            , PathCompare
                            , GlobSetMember
                            >;

    PathNode  m_root_node;
    GlobalSet m_global_solution_set; // sorted set of all PathNode of which state is a complete solution.
    boost::object_pool<PathNode> m_node_pool {1000000, 0};

    // Pre-calculated distance between any 2 stations.
    // Could be made lazily-evaluated using Dijkstra on-demand rather than Johnson all-pair
    class DistanceMatrix
    {
        std::unique_ptr<distance_t[]> m_storage {};
        size_t                        m_num_vertices {};
    public:
        void reset_square(size_t num_vertices)
        {
            m_num_vertices = num_vertices;
            m_storage.reset(new distance_t[m_num_vertices * m_num_vertices]);
        }
        // API for johnson_all_pairs_shortest_paths()
        struct Row
        {
            const distance_t* row_bgn;
            distance_t& operator[](size_t xx) {return const_cast<distance_t*>(row_bgn)[xx];}
            const distance_t& operator[](size_t xx) const {return row_bgn[xx];}
        };
        Row operator[](size_t yy) {return Row{m_storage.get() + yy * m_num_vertices};}
        Row operator[](size_t yy) const {return Row{m_storage.get() + yy * m_num_vertices};}
    };

    DistanceMatrix m_distance_matrix;

    using Graph =  boost::adjacency_list
                        < boost::vecS
                        , boost::vecS
                        , boost::undirectedS
                        , boost::no_property
                        , boost::property< boost::edge_weight_t, distance_t >
                        >;

    Graph m_graph;

    struct Package
    {
        weight_t    weight;
        station_id  destination;
        Package(size_t ww, size_t dd) 
            : weight(safe_cast<weight_t>(ww))
            , destination(safe_cast<station_id>(dd))
            { }
    };
    struct Train
    {
        size_t      capacity;
    };
    std::vector<Package>    m_packages;
    std::vector<Train>      m_trains;

    // Looks for just 1 solution path starting from given root,
    // given current strategy
    //=============================================================================================
    const PathNode* depth_search_solution(const PathNode& root)
    {
        assert(root.score.was_evaluated());
        assert(!root.score.is_solution());

        // Create a new child off root
        PathNode* const new_child = m_node_pool.construct();
        new_child->parent_ptr = &root;
        root.children.insert(*new_child);

        new_child->state = root.state;

        // Value to be eventually added to base_tp, representing the future time 
        // the new child node will be found at.
        weight_t time_to_advance = std::numeric_limits<weight_t>::max();

        // 1) check ALL trains that are parked at stations (eta=0). These need actions.
        array_sz<train_id, MAX_TRAINS> trains_neededing_action;
        for( train_id tt = 0; tt < m_trains.size(); ++tt)
        {
            const Coordinate& train_coord = new_child->state.train_2_coord[tt];
            if(train_coord.eta)
            {
                time_to_advance = std::min(time_to_advance, train_coord.eta);
                continue;
            }
            // Train is at station and need actions.
            // Unload cargo but don't chose action yet because there might be other  
            // trains at the same station at the same time.
            trains_neededing_action.push_back(tt);
            const station_id ss = train_coord.station_dst;

            State::PackageList& packages_src_train = 
                new_child->state.train_2_packagelist[ TrainOrStationId(tt,true) ];
            
            State::PackageList& packages_dst_station = 
                new_child->state.train_2_packagelist[ TrainOrStationId(ss,false) ];

            for( package_id pp : packages_src_train)
                packages_dst_station.push_back(pp);

            packages_src_train.clear(); 
        }

        // Assumed precondition: root.state contains parked (not moving) train(s).
        assert(trains_neededing_action.size());

        // 2) for each train needing action, 
        for (train_id tt : trains_neededing_action)
        {
            Coordinate& train_coord = new_child->state.train_2_coord[tt];
            const station_id ss = train_coord.station_dst;

            State::PackageList& packages_src_station = 
                new_child->state.train_2_packagelist[ TrainOrStationId(ss,false) ];

            State::PackageList& packages_dst_train = 
                new_child->state.train_2_packagelist[ TrainOrStationId(tt,true) ];

            // Station might run out of packages if parking multiple trains
            // assert(packages_src_station.size() > 0);
            // assert(packages_dst_train.size() == 0);
            
            size_t train_capacity_left = m_trains[tt].capacity;

            //  b) select [some] first leading package from station, 
            //     (respecting train capacity) depending on strategy:
            //          quasi-random?
            //          largest?
            //          furthest away from destination?
            //          closest?
            //  c) choose route leading to a station which is closer to that package's destination
            station_id chosen_dest;
            {
                package_id chosen_pid = NO_PACKAGE;
                for(ssize_t ii = packages_src_station.size()-1; ii >= 0; --ii)
                {
                    package_id const pp = packages_src_station[ii];
                    const Package& pkg = m_packages[pp];
                    if(pkg.weight <= train_capacity_left)
                    {
                        chosen_pid = pp;
                        break;
                    }
                }
                if(NO_PACKAGE == chosen_pid)
                {
                    if(packages_src_station.size() == 0)
                        new_child->state.train_2_packagelist.erase(TrainOrStationId(ss,false));
                    // all of the station's packages are too large for the train.
                    // move the train somewhere (TODO: Strategy::WaitForNextTrain)
                    // perhaps keep a list of isolated, stationed packages and then
                    // select train to move into that direction?
                    // Can't skip train here because we still need to advance time for all.
                    chosen_pid = rand() % m_packages.size(); 
                }
                
                // Given where the leading package wants to go, select the edge/route
                // which ends up in a station closer to the package.
                const station_id leading_pkg_dest = m_packages[chosen_pid].destination;

                distance_t best_distance = std::numeric_limits<distance_t>::max();
                boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                for ( boost::tie(ei, ei_end) = boost::out_edges(ss, m_graph)
                    ; ei != ei_end
                    ; ++ei )
                {
                    const station_id source = boost::source( *ei, m_graph ); // source == ss
                    const station_id target = boost::target( *ei, m_graph ); 
                    const weight_t   weight = boost::get(boost::edge_weight_t(), m_graph, *ei);
                    //const size_t     eindex = boost::get(boost::edge_index_t(), m_graph, *ei); // for Action, but curretly disabled

                    // check if target station is better
                    const auto target_dist = m_distance_matrix [target] [leading_pkg_dest];
                    if(best_distance > target_dist)
                    {
                        best_distance           = target_dist;
                        chosen_dest             = target;
                        // Set train moving
                        train_coord.station_dst = target;
                        train_coord.eta         = weight;
                        //groute_id chosen_edge = eindex; 
                        // new_child->parent_action;
                    }
                }
            }

            //  d) take all cargo that does like the chosen destination.
            //  e) [quasi-random] consider taking cargo that doesn't like the destination so much.
            for(ssize_t ii = packages_src_station.size()-1; ii >= 0 && train_capacity_left; --ii)
            {
                package_id const pp = packages_src_station[ii];
                const Package& pkg = m_packages[pp];
                if(pkg.weight > train_capacity_left)
                    continue; // skip too big of a package

                auto const dist_curr_ss = m_distance_matrix [pkg.destination] [ss];
                auto const dist_next_ss = m_distance_matrix [pkg.destination] [chosen_dest];
                bool const moves_closer = dist_next_ss < dist_curr_ss;

                if(!moves_closer)
                    continue; // skip if package would be moving further away from its destination
                
                // Package accepted into the train. Move it from station into the train.
                train_capacity_left -= pkg.weight;
                packages_dst_train.push_back(pp);
                packages_src_station.swap_erase_at(ii);
            }

            time_to_advance = std::min(time_to_advance, train_coord.eta);
        }
        
        // switch(root.children_next_strategy.strat_move)
        // ++root.children_last_strategy;

        // 3) advance time for new child's state until some train(s) arrive at some station.
        new_child->base_tp = root.base_tp + time_to_advance;
        for(Coordinate& tcoord : new_child->state.train_2_coord)
            tcoord.eta -= time_to_advance;

        new_child->score = evaluate(new_child->state, new_child);

        if(new_child->score.is_solution())
        {
            m_global_solution_set.insert(*new_child);
            return nullptr;
        }

        // depth-first search
        // depth_search_solution(*new_child); // avoid stack overflow on deep recursion
        return new_child;
    }

    // Sum of distance for each package to reach their destination,
    // using pre-computed all-pairs shortest paths, johnson_all_pairs_shortest_paths()
    // dijkstra_shortest_paths(): The time complexity is O(V log V).
    // johnson_all_pairs_shortest_paths(): The time complexity is O(E V log V).
    // https://www.boost.org/doc/libs/1_77_0/libs/graph/doc/johnson_all_pairs_shortest.html
    //=============================================================================================
    Evaluation evaluate( const State& state
                       , const PathNode* const node
                       , const bool use_gravity = true
                       ) const
    {
        Evaluation eval(0);

        std::bitset<MAX_TRAINS> busy_trains {0};
        array_sz<station_id, MAX_STATIONS> loaded_stations; // stations with packages but no train

        for(auto kvp: state.train_2_packagelist)
        {
            Coordinate pkg_curr_coord;
            const TrainOrStationId& loc = kvp.first;
            if(loc.is_train())
            {
                train_id const tt = loc.as_train();
                pkg_curr_coord = state.train_2_coord[tt];
                busy_trains[tt] = true;
            }
            else if(use_gravity) // package is outside train, at loaded station
            {
                station_id const ss = loc.as_station();
                pkg_curr_coord.station_dst = ss;
                pkg_curr_coord.eta = 0;
                loaded_stations.push_back(ss);
            }

            // all packages in train/station
            for(package_id pp : kvp.second)
            {
                // Where does the package have to go?
                const station_id pkg_final_dest = m_packages[pp].destination;

                // How far is the package from its destination?
                const distance_t pkg_delivery_distance = 
                    pkg_curr_coord.eta + // eta will be positive if the train hasn't arrive at the station yet.
                    m_distance_matrix
                        [pkg_curr_coord.station_dst] // pkg at station? train half-way to a station?
                        [pkg_final_dest]; // where the package must go

                eval += pkg_delivery_distance;
            }
        }

        // Solution is detected if the sum of pkg_delivery_distance is zero.
        // This means all packages are exactly at their final destinations.
        if(eval == Evaluation())
        {
            eval.set_as_solution();
            // The numeric score of a solved evaluation is the path cost to root
            for(const PathNode* nn = node; nn; nn = nn->parent_ptr)
                eval += nn->base_tp;
            return eval;
        }
        
        // Add some gravity force that pulls empty trains towards loaded stations.
        if(use_gravity && loaded_stations.size())
        {
            for(train_id tt = 0; tt < m_trains.size(); ++tt)
            {
                if(busy_trains[tt])
                    continue;
                // Empty train tt found.
                // We want to know how far each train is from the closest loaded station
                const Coordinate& train_coord = state.train_2_coord[tt];
                distance_t distance_to_nearest_pkg = std::numeric_limits<distance_t>::max();
                for(station_id ss : loaded_stations)
                {
                    const distance_t dist = train_coord.eta +  
                        m_distance_matrix [train_coord.station_dst] [ss];
                    if( distance_to_nearest_pkg > dist )
                        distance_to_nearest_pkg = dist;
                    if( 0 == distance_to_nearest_pkg)
                        break;
                }
                eval += distance_to_nearest_pkg;
            }
        }

        return eval;
    }

    //=============================================================================================
    void load_config(const TrainsConfig& a_cfg)
    {
        m_graph = graph_from_config(a_cfg);

        // Compute all-pairs shortest path -> m_distance_matrix[][];
        // data/01.txt: m_distance_matrix[i][j]
        //           0    1    2    3    4    5    6    7    8
        //  0 ->     0    9    5    9   14   15   13   13   10
        //  1 ->     9    0    5    9   14   15   13   13   18
        //  2 ->     5    5    0    4    9   10    8    8   13
        //  3 ->     9    9    4    0    5    6    4    4    9
        //  4 ->    14   14    9    5    0    1    3    1    6
        //  5 ->    15   15   10    6    1    0    4    2    7
        //  6 ->    13   13    8    4    3    4    0    4    9
        //  7 ->    13   13    8    4    1    2    4    0    5
        //  8 ->    10   18   13    9    6    7    9    5    0
        const size_t VV = m_graph.vertex_set().size();
        const size_t EE = m_graph.m_edges.size();
        m_distance_matrix.reset_square(VV);
        johnson_all_pairs_shortest_paths(m_graph, m_distance_matrix);

        // Setup packages and trains.
        // Setup root node to initial state of the problem.

        m_trains.clear();
        for(train_id tt = 0; tt < a_cfg.trains.size(); ++tt)
        {
            const auto& cfg_train = a_cfg.trains[tt];
            m_trains.push_back({cfg_train.capacity});
            Coordinate& r_train_coord = m_root_node.state.train_2_coord[tt];
            r_train_coord.station_dst = cfg_train.station_curr;
            r_train_coord.eta = 0;
        }
        
        m_packages.clear();
        for(package_id pp = 0; pp < a_cfg.deliveries.size(); ++pp)
        {
            const auto& cfg_pkg = a_cfg.deliveries[pp];
            const station_id ss_src = cfg_pkg.station_curr;
            const station_id ss_dst = cfg_pkg.station_dest;
            m_packages.push_back({(weight_t)cfg_pkg.weight,ss_dst});

            const auto station_key = TrainOrStationId(ss_src,false);
            m_root_node.state.train_2_packagelist[ station_key ].push_back(pp);
        }

        m_root_node.score = evaluate(m_root_node.state, &m_root_node);
    }

    //=============================================================================================
    static Graph graph_from_config(const TrainsConfig& a_cfg)
    {
        using namespace boost;
        Graph graph;

        // Construct graph edges
        {
            using Edge = std::pair< station_id, station_id >;
            std::vector<Edge> edges;
            edges.reserve(a_cfg.routes.size());
            for(const TrainsConfig::Route& route: a_cfg.routes)
                edges.push_back({(station_id)route.src(), (station_id)route.dst()});
            const size_t VV = a_cfg.stations.size();
            graph = Graph(edges.begin(), edges.end(), VV);
        }

        // Set graph weights
        {
            property_map< Graph, edge_weight_t >::type weight_map = get(edge_weight, graph);
            graph_traits< Graph >::edge_iterator ee, e_end;
            auto it_routes = a_cfg.routes.begin();
            for (boost::tie(ee, e_end) = edges(graph); ee != e_end; ++ee)
                weight_map[*ee] = (it_routes++)->distance;
        }
        return graph;
    }

    //=============================================================================================
    void solve(const TrainsConfig& a_cfg)
    {
        load_config(a_cfg);
        if(m_root_node.score.is_solution())
            m_global_solution_set.insert(m_root_node);
        {
            const PathNode* child = &m_root_node;
            size_t ii = 0;
            while(child)
            {
                std::cerr << ii ++ << '\n';
                child = depth_search_solution(*child);
            }
        }
    }

};


bool solver_annealing(const TrainsConfig& a_cfg)
{
    /*
    SASolver< 16    // bits, packages
            , 8     // bits, trains
            , 8     // bits, stations
            , 3     // bits, routes per station
            , 8     // bits, weight of single route
            , 15    // bits, maximum path weight/time (1 bit reserved)
            > solver;
    */
    SASolver solver;
    solver.solve(a_cfg);
    return true;
}
