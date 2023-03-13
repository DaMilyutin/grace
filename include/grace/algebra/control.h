#pragma once

#include <grace/algebra/rules.h>
#include <grace/algebra/join.h>
#include <optional>

namespace grace
{
    namespace rules
    {
        struct start_token {};
        struct close_token {};

        inline constexpr start_token start;
        inline constexpr close_token close;
    }
}
