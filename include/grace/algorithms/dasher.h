#pragma once
#include <grace/algebra/rules.h>
#include <grace/algorithms/segmentation.h>
#include <grace/decorators/dash.h>

#include <optional>
#include <vector>

namespace grace
{
    namespace elements
    {
        class Dasher: public rules::Link<Dasher>
        {
        public:
            Dasher(decorators::Dash const& d)
                : dash_(d)
            {
                skipper_.reset(dash_.start);
            }

            template<typename Y> void begin(Y const& ) const {}
            template<typename Y> void end(Y const&)    const {}


            template<typename S>
            bool feed(S& sink, Point_r const& p)
            {
                while(true)
                {
                    if((phase_&1)==0)
                    {
                        if(keeper_.consume(p))
                            break;
                        Point_r const last = keeper_.path.back();
                        keeper_.path.back() = towards(keeper_.path[keeper_.path.size()-2], last, keeper_.length_limit);
                        if(!sink.consume(keeper_.path))
                            return false;
                        next_phase();
                        skipper_.reset(dash_.pattern[phase_/2].gap_length);
                        skipper_.consume(keeper_.path.back());
                    }
                    if((phase_&1)==1)
                    {
                        if(skipper_.consume(p))
                            break;
                        next_phase();
                        keeper_.reset(dash_.pattern[phase_/2].dash_length);
                        keeper_.consume(towards(skipper_.points.back(1), skipper_.points.back(0), skipper_.length_limit));
                    }
                }
                return true;
            }

        private:
            void next_phase() { phase_ = (phase_ + 1)%(dash_.pattern.size()*2); }

            decorators::Dash dash_;
            int              phase_    = -1;
            Keeper<void>     keeper_;
            Skipper          skipper_;
        };
    }


    namespace decorators
    {
        template<typename Y>
        auto operator/(rules::Yield<Y>&& y, Dash const& d)
        {
            return FWD(y)._get_()/elements::Dasher(d);
        }
    }

}