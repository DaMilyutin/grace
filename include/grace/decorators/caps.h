#pragma once
#include <grace/algebra/rules.h>

namespace grace
{
    namespace caps
    {
        struct Butt: rules::Decorator<caps::Butt>
        {};

        struct Round: rules::Decorator<caps::Round>
        {};
    }
}