#pragma once

#include <grace/algebra/algebra.h>
#include <iterator>

namespace grace
{
    namespace rules
    {
        template<typename E>
        struct Reversed: Yield<Reversed<E>>
        {
            template<typename T1>
            Reversed(T1&& p)
                : under(FWD(p))
            {}
            E under;
        };

        template<typename P>
        Reversed<P> reversed(Yield<P>&& p)
        {
            return {FWD(p)._get_()};
        }

        template<typename P>
        Reversed<P const&> reversed(Yield<P> const& p)
        {
            return p._get_();
        }

        template<typename R, typename P>
        R& operator<<(Sink<R>& ras, Reversed<P> const& points)
        {
            R& the_ras = ras._get_();
            P const& the_points = points.under;
            auto b = std::rbegin(the_points);
            auto e = std::rend(the_points);
            for(; b != e; ++b)
                the_ras << *b;
            return the_ras;
        }
    }
}
