#pragma once
#include <grace/algebra/rules.h>
#include <agge/types/scalars.h>

namespace grace
{
    namespace joins
    {
        struct Bevel: rules::Decorator<joins::Bevel>
        {};

        struct Round: rules::Decorator<joins::Round>
        {};

        struct Miter: rules::Decorator<joins::Miter>
        {};

        struct Edgeless: rules::Decorator<joins::Miter>
        {
            agge::real_t smaller = 3.;
        };
    }
}