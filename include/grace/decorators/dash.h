#pragma once
#include <grace/algebra/rules.h>
#include <vector>

namespace grace
{
    namespace decorators
    {
        class Dash: rules::Decorator<Dash>
        {
        public:
            Dash& reset(real_t offset = 0.f)
            {
                pattern.clear();
                start = offset;
                return *this;
            }

            Dash& add(real_t dash_length, real_t gap_length)
            {
                dash_gap dg = {dash_length, gap_length};
                pattern.push_back(dg);
                return *this;
            }

            struct dash_gap
            {
                real_t dash_length;
                real_t gap_length;
            };
            real_t                start = 0.f;
            std::vector<dash_gap> pattern;
        };
    }
}