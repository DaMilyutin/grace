#pragma once

#include <grace/algebra/algebra.h>
#include <grace/algebra/adapted.h>
#include <grace/algebra/control.h>

namespace grace
{
    namespace elements
    {
        template<typename C>
        struct PushBack: rules::Sink<PushBack<C>>
        {
            PushBack(C& c): container(c) {}

            bool consume(Point_r const& p)
            {
                container.push_back(p);
                return true;
            }

            C& container;
        };

        template<typename C>
        auto push_back(C& c) { return PushBack<C>(c); }

    }
}