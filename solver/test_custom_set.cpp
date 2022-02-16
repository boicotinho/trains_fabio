//#define BOOST_POOL_NO_MT
#include <boost/pool/pool_alloc.hpp>
#include <boost/unordered/unordered_map.hpp>
#include <set>
#include <boost/intrusive/set.hpp>
#include <iostream>

namespace intr = boost::intrusive;
using IntrSetHook = intr::set_member_hook<>;
using IntrSetBase = intr::set_base_hook< intr::optimize_size<true> >;
struct PathNode;
using BaseSet  = intr::set< PathNode, intr::compare<std::greater<PathNode> > >;

struct PathNode : public IntrSetBase
{
    PathNode*   parent {nullptr};
    IntrSetHook member_hook_for_global_set;
    BaseSet     childrenxx;
    int  score {};

    explicit PathNode(int ss) : score(ss) {}

    bool operator > (const PathNode& oo) const {return score > oo.score;}
};

using MemberOption = intr::member_hook<PathNode, intr::set_member_hook<>, &PathNode::member_hook_for_global_set> ;
using MemberSet    = intr::set< PathNode, intr::compare<std::greater<PathNode> >, MemberOption>;


void test_intrusive_set()
{
    std::vector<PathNode> node_storage;
    for(int ii = 0; ii < 10; ++ii)
    {
        auto rr = rand() % 100;
        std::cout << rr << '\n';
        node_storage.emplace_back(rr);
    }

    PathNode aa(11);
    PathNode bb(22);
    PathNode cc(33);
    bb.childrenxx.insert(cc);

    MemberSet memberset;

    BaseSet baseset;
    for(PathNode& node: node_storage)
    {
        baseset.insert(node);
        if(35 ==node.score ) // || 86 == node.score) can't add to second list
        {
            node.childrenxx.insert(aa);
            node.childrenxx.insert(bb);

            memberset.insert(aa);
            memberset.insert(bb);
        }
    }
    std::cout << "==========" << '\n';
    for(const auto& ee : memberset)
        std::cout << ee.score << '\n';

    std::cout << "==========" << '\n';
    for(auto it = baseset.begin(); it != baseset.end(); ++it)
    {
        int uu = it->score;
        std::cout << uu << '\n';
        for(const auto& ee : it->childrenxx)
        {
            std::cout << "  " << ee.score << '\n';
            for(const auto& gg : ee.childrenxx)
                std::cout << "    " << gg.score << '\n';
        }
    }

    //std::cout << "==========" << '\n';
    //for(auto ee : baseset)
    //{
    //    int uu = ee.score;
    //    std::cout << uu << '\n';
    //}
    int asd = 132;
}



using RawMap = boost::unordered_map<std::string, std::string>;
using Elements = boost::unordered_map<
        std::string,
        std::string,
        RawMap::hasher,
        RawMap::key_equal,
        boost::fast_pool_allocator
            < RawMap::value_type
            , boost::default_user_allocator_new_delete // default_user_allocator_new_delete
            //, boost::details::pool::null_mutex
            //, 64
            //, 1ull*1024*1024*1024
            >
    >;

using Key = int;
using Allocator = boost::fast_pool_allocator
                    < Key
                    , boost::default_user_allocator_new_delete
                    , boost::details::pool::null_mutex
                    >;
using PooledSet = std::set<Key, std::less<Key>, Allocator>;


int test_custom_set()
{
    test_intrusive_set();
    {
        PooledSet pset;
        auto p0 = &*pset.insert(3).first; // 32(count) * 40(elem_size) + 8 + 8 = 1280 + 16 = 1296
        auto p1 = &*pset.insert(4).first; // 40 bytes offset from p0, yuck!
        auto p2 = &*pset.insert(2).first; // 80 bytes offset from p0, double yuck!
        int u = 32;
    }

    {
        Elements hashtable;

        hashtable.insert({
                { "one",   "Eins"  },
                { "two",   "Zwei"  },
                { "three", "Drei"  },
                { "four",  "Vier"  },
                { "five",  "Fuenf" },
                });

        for (auto& e : hashtable) {
            std::cout << "Hash entry: " << e.first << " -> " << e.second << "\n";
        }

        std::cout << "hashtable[\"three\"] -> " << hashtable.find("three")->second << "\n";
    }

    // OPTIONALLY: free up system allocations in fixed size pools
    // Two sizes, are implementation specific. My 64 system has the following:
    boost::singleton_pool<boost::fast_pool_allocator_tag, 8>::release_memory();  // the bucket pointer allocation
    boost::singleton_pool<boost::fast_pool_allocator_tag, 32>::release_memory(); // the ptr_node<std::pair<std::string const, std::string> >
    return 0;
}
