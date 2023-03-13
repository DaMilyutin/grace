#pragma once

#include <grace/algebra/algebra.h>
#include <grace/algebra/adapted.h>
#include <grace/mediators/Path.h>

#include <grace/elements/Segment.h>

namespace grace
{
    namespace decorators
    {
        struct OrthoShift: rules::TransformOr<OrthoShift>
        {
            OrthoShift(real_t off)
                : offset(off)
            {}

            real_t offset = 0.0f;

            template<typename S, size_t N>
            bool feed(S& the_sink, ylems::elements::CycleBuffer<Point_r, N> const& buffer) const
            {
                if(buffer.size() < 2)
                    return true;
                real_t const scale = offset/distance(buffer.back(1), buffer.back(0));
                Vector_r const o{(buffer.back(0).y - buffer.back(1).y)*scale, -(buffer.back(0).x - buffer.back(1).x)*scale};
                return the_sink.consume(buffer.back(1) + o)
                    && the_sink.consume(buffer.back(0) + o);
            }
        };
    }
}