#pragma once

#include <grace/algebra/algebra.h>
#include <grace/algebra/adapted.h>
#include <grace/algebra/control.h>

#include <grace/elements/Segment.h>
#include <optional>

namespace grace
{
    namespace elements
    {
        struct Segments: rules::TransformOr<Segments>
        {
            std::optional<elements::Segment> operator()(Point_r const& p) const
            {
                buffer.push_back(p);
                if(buffer.size() < 2)
                    return std::nullopt;
                return elements::Segment{buffer.back(1), buffer.back(0)};
            }

            template<typename S>
            bool feed(S& the_sink, Point_r const& p) const
            {
                buffer.push_back(p);
                if(buffer.size() < 2)
                    return true;
                return the_sink.consume(elements::Segment{buffer.back(1), buffer.back(0)});
            }

            ylems::elements::CycleBuffer<Point_r, 2> mutable buffer;
            bool                                     mutable start = false;
        };
    }
}