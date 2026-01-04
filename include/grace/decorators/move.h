#pragma once

#include <grace/algebra/algebra.h>
#include <grace/algebra/adapted.h>
#include <grace/mediators/Path.h>

#include <grace/elements/Segment.h>

namespace grace
{
    namespace decorators
    {
        struct Move : grace::rules::Transform<Move>
        {
            Move(Vector_r v) : offset(v) {}

            template<typename S>
            bool feed(S& the_sink, Point_r const& x) const
            {
                return the_sink.consume(x + offset);
            }

            template<typename S>
            bool feed(S& the_sink, std::vector<Point_r> x) const
            {
                for(auto& v : x) 
                    v += offset;
                return the_sink.consume(x);
            }

            template<typename S, size_t N>
            bool feed(S& the_sink, ylems::elements::CycleBuffer<Point_r, N> x) const
            {
                for (int i = 0; i < x.size(); ++i)
                    x.back(i) += offset;
                return the_sink.consume(x);
            }

            Point_r operator()(Point_r x) const { return x + offset; }

            Vector_r offset = {};
        };
    }
}