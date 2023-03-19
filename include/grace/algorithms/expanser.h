#pragma once
#include <grace/algebra/rules.h>
#include <grace/decorators/extrudes.h>
#include <grace/decorators/joins.h>

#include <optional>
#include <vector>

namespace grace
{
    namespace elements
    {
        class Expanser: public rules::Link<Expanser>
        {
        public:
            Expanser(extrudes::Ortho const& o)
                : extrude_(o)
            {}

            template<typename S>
            bool feed(S& sink, std::vector<Point_r> const& p)
            {
                if(p.size() < 2)
                    return true;
                std::vector<real_t> dists;
                dists.resize(p.size()-1, 0.f);
                sink << grace::rules::start;
                for(size_t i = 1; i < p.size(); ++i)
                {
                    dists[i-1] = distance(p[i], p[i-1]);
                    real_t const scale = 0.5f*extrude_.width/dists[i-1];
                    Vector_r const o{(p[i].y - p[i-1].y)*scale, -(p[i].x - p[i-1].x)*scale};
                    if(!sink.consume(p[i-1] + o) || !sink.consume(p[i] + o))
                        return false;
                }
                for(size_t i = p.size()-1; i > 0; --i)
                {
                    real_t const scale = 0.5f*extrude_.width/dists[i-1];
                    Vector_r const o{(p[i-1].y - p[i].y)*scale, -(p[i-1].x - p[i].x)*scale};
                    if(!sink.consume(p[i] + o) || !sink.consume(p[i-1] + o))
                        return false;
                }
                sink << grace::rules::close;
                return true;
            }

        private:
            extrudes::Ortho extrude_;
        };

    }


}