#pragma once
#include <memory>
#include <limits>
#include <random>


// Arguments defining the distribution for a random number.
struct RandArgs
{
    using Val = ssize_t;
    double  mean    {0};
    double  stddev  {0};
    Val     min     {std::numeric_limits<Val>::min()};
    Val     max     {std::numeric_limits<Val>::max()};

    RandArgs() = default;
    RandArgs( double  mn
            , double  sd
            , Val     lo = std::numeric_limits<Val>::min()
            , Val     hi = std::numeric_limits<Val>::max()
            )
        : mean(mn), stddev(sd), min(lo), max(hi) {}
    bool is_uniform_distribution() const {return stddev < 0;}
};

// Random integer generator, given distribution arguments.
// Uses type erasure to support both uniform and normal distributions.
class RandIntGenerator
{
    using Val = RandArgs::Val;
    struct GenErasure { virtual Val draw() = 0; };
    std::unique_ptr<GenErasure> m_gen;
public:
    Val operator()() {return draw();}
    Val draw()       {return m_gen->draw();}

    template<class Engine>
    RandIntGenerator(Engine& a_engine, const RandArgs& a_args);
};

//=============================================================================
template<class Engine>
inline RandIntGenerator::RandIntGenerator( Engine&         a_engine
                                         , const RandArgs& a_args )
{
    if(a_args.is_uniform_distribution())
    {
        class UniformDist : public GenErasure
        {
            Engine& m_engine;
            std::uniform_int_distribution<Val> m_dist_uni;
        public:
            UniformDist(Engine& eng, const RandArgs& args)
                : m_engine(eng)
                , m_dist_uni(args.min, args.max)
                { }
            virtual Val draw() override
                { return m_dist_uni(m_engine); }
        };
        m_gen.reset(new UniformDist(a_engine, a_args));
    }
    else
    {
        class NormalDist : public GenErasure
        {
            Engine& m_engine;
            std::normal_distribution<double> m_dist_nrm;
            Val m_min;
            Val m_max;
        public:
            NormalDist(Engine& eng, const RandArgs& args)
                : m_engine(eng)
                , m_dist_nrm(args.mean, args.stddev)
                , m_min(args.min)
                , m_max(args.max)
                { }
            virtual Val draw() override
                {
                auto const v0 = m_dist_nrm(m_engine);
                Val const vv = std::round(v0);
                if(vv < m_min) return m_min;
                if(vv > m_max) return m_max;
                return vv;
                }
        };
        m_gen.reset(new NormalDist(a_engine, a_args));
    }
}
