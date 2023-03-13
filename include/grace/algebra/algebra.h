#pragma once
#include <grace/algebra/rules.h>

namespace grace
{
    namespace rules
    {
        // Sink
        template<typename S, typename E>
        S& operator<<(Sink<S>& s, Yield<E> const& e)
        {
            meld_tag<terminal>(e, s);
            return s._get_();
        }
    }
}