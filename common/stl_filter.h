#pragma once
#include <algorithm>

template<class TT, class FN>
inline TT stl_filter(const TT& src_container, FN filter_func)
{
    TT res;
    std::copy_if( src_container.begin()
                , src_container.end()
                , std::back_inserter(res)
                , filter_func
                );
    return res;
}
