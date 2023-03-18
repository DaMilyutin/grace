#pragma once
#pragma once
#include "rules.h"
#include <ylems/elements.h>

namespace grace
{
    namespace rules
    {
        template<typename T, typename D, typename I>
                                         auto iota(T t, D d, I i)        { return ylems::elements::iota<terminal>(t, d, i); }
        template<typename T, typename D> auto iota(T t, D d)             { return ylems::elements::iota<terminal>(t, d); }

        template<typename T, typename I> auto linspace(T b, T e, I i)    { return ylems::elements::linspace<terminal>(b, e, i); }

        template<typename... Ys>         auto join(Ys&&... ys)           { return ylems::elements::join<terminal>(FWD(ys)...); }

        template<typename Y1, typename Y2>
        using Joined = ylems::elements::JoinYield<terminal, Y1, Y2>;

        template<typename T>             auto range(T b, T e, T step)    { return ylems::elements::range<terminal>(b, e, step); }
        template<typename T>             auto range(T b, T e)            { return ylems::elements::range<terminal>(b, e); }

        template<typename    Y>          auto yield(Y&& y)               { return ylems::elements::yield<terminal>(FWD(y)); }
        template<typename... T>          auto zip(T&&... f)              { return ylems::elements::zip<terminal>(FWD(f)...); }
        template<typename    Y>          auto as_range(Y&& y)            { return ylems::elements::as_range<terminal>(FWD(y)); }
        template<typename B, typename E> auto as_range(B b, E e)         { return ylems::elements::as_range<terminal>(b, e); }
        template<size_t N, typename Y>   auto cycle(Y&& y)               { return ylems::elements::cycle<terminal, N>(FWD(y)); }


        template<typename F>             auto filter(F&& f)              { return ylems::elements::filter<terminal>(FWD(f)); }
        template<typename I>             auto take(I i)                  { return ylems::elements::take<terminal>(i); }
        template<typename I>             auto drop(I i)                  { return ylems::elements::drop<terminal>(i); }

        template<typename F>             auto transform(F&& f)           { return ylems::elements::transform<terminal>(FWD(f)); }
        template<typename F>             auto transform_or(F&& f)        { return ylems::elements::transform_or<terminal>(FWD(f)); }
        template<typename F, typename G> auto transform_or(F&& f, G&& g) { return ylems::elements::transform_or<terminal>(FWD(f), FWD(g)); }

        template<typename S>             auto memoize(S&& s)             { return ylems::elements::memoize<terminal>(FWD(s)); }
        template<typename T, size_t N>   auto memoize()                  { return ylems::elements::memoize<terminal, T, N>(); }

        template<typename S>             auto push_back(S&& s)           { return ylems::elements::push_back<terminal>(FWD(s)); }
    }

    using rules::iota;
    using rules::linspace;
    using rules::range;
    using rules::cycle;

    using rules::yield;
    using rules::zip;
    using rules::join;
    using rules::as_range;

    using rules::filter;
    using rules::take;
    using rules::drop;

    using rules::transform;
    using rules::transform_or;

    using rules::memoize;
    using rules::push_back;
}