#pragma once
#include <grace/algebra/rules.h>
#include <grace/types/scalars.h>
#include <grace/types/Point.h>

#include <functional>

namespace grace
{
    namespace extrudes
    {
        struct Ortho: rules::Decorator<extrudes::Ortho>
        {
            Ortho() = default;
            Ortho(agge::real_t w): width(w) {}

            agge::real_t width = 1.0f;
        };

        struct OrthoWidthFun: rules::Decorator<extrudes::OrthoWidthFun>
        {
            OrthoWidthFun(std::function<double(double)> w): width(w) {}
            std::function<double(double)> width;
        };

        struct Directed: rules::Decorator<extrudes::Directed>
        {
            Directed() = default;
            Directed(agge::real_t w): width(w) {}

            agge::real_t   width = 1.0f;
            agge::Vector_r direction {0., 1};
        };

    }
}