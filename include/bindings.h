#ifndef __BINDINGS_H__
#define __BINDINGS_H__

/* std */
#include <vector>
#include <deque>


// STL list-like containers
void IndexError();

template<class T>
struct stl_list
{
    typedef typename T::value_type V;
    static V& get(T const& x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) return x[i];
        IndexError();
    }
    static void set(T const& x, int i, V const& v)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x[i]=v;
        else IndexError();
    }
    static void del(T const& x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x.erase(i);
        else IndexError();
    }
    static void add(T const& x, V const& v)
    {
        x.push_back(v);
    }
};

template<class T>
struct stl_deque
{
    typedef typename T::value_type V;
    static V& get(T & x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) return x[i];
        IndexError();
    }
    static void set(T & x, int i, V const& v)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x[i]=v;
        else IndexError();
    }
    static void del(T & x, int i)
    {
        if( i<0 ) i+=x.size();
        if( i>=0 && i<x.size() ) x.erase(x.begin() + i);
        else IndexError();
    }
    static void add(T & x, V const& v)
    {
        x.push_back(v);
    }
};

#endif
