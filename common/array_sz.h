#pragma once

#include "t_uint.h"
#include <array>
#include <cassert>

template<class T, size_t MAX_N>
class array_sz
{
public:
    using size_type = uintv<MAX_N>;

    enum {INVOKES_DESTRUCTOR = false};

    static size_t capacity() {return MAX_N;}
    size_t size() const {return m_size;}
    void resize(size_t sz) {assert(sz < MAX_N); m_size = sz;} // doesn't invoke destructor
    void clear() { resize(0);}

    void fill(const T& val)   { std::fill(m_arr.begin(), m_arr.end(), val); }

    void push_back(const T& v) 
        {
        assert(m_size < MAX_N);
        m_arr[m_size++]=v;
        }

    T& emplace_back() // doesn't actually invoke constructor for now
        {
        assert(m_size < MAX_N);
        return m_arr[m_size++];
        }

    T& back()
        {
        assert(m_size > 0);
        return m_arr[m_size-1];
        }

    void swap_erase_at(size_t const ix)
        {
        assert(m_size > 0);
        assert(ix < m_size);
        std::swap(back(), m_arr[ix]);
        --m_size;
        }

    void pop_back() {swap_erase_at(m_size-1);}

    T& operator[](size_t ix)             {assert(ix < m_size); return m_arr[ix];}
    const T& operator[](size_t ix) const {assert(ix < m_size); return m_arr[ix];}

    using array_t        = std::array<T, MAX_N>;
    using iterator       = typename array_t::iterator;
    using const_iterator = typename array_t::const_iterator;

    iterator begin()              {return m_arr.begin();}
    iterator end()                {return begin()+m_size;}
    const_iterator begin() const  {return m_arr.cbegin();}
    const_iterator end()   const  {return m_arr.cbegin()+m_size;}
    const_iterator cbegin() const {return m_arr.cbegin();}
    const_iterator cend()   const {return m_arr.cbegin()+m_size;}

private:
    size_type   m_size {0};
    array_t     m_arr;
};
