#pragma once

namespace grace
{
    namespace types
    {
        typedef float          real_t;

        typedef unsigned int   count_t;
        typedef unsigned char  uint8_t;
        typedef unsigned short uint16_t;
    }
    using namespace types;

    namespace constants
    {
        constexpr real_t pi           = real_t(3.141592653589793);
        constexpr real_t tau          = 2*pi;
        constexpr real_t pi_2         = pi/2;
        constexpr real_t pi_4         = pi/4;
        constexpr real_t sqrt_3_4     = 0.86602540378f;
        constexpr real_t inv_sqrt_3_4 = 1.15470053838f;
    }

    using namespace constants;
};