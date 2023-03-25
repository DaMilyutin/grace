#pragma once
#include <grace/algebra/rules.h>
#include <grace/algebra/rules.h>
#include <grace/types/Point.h>

#include <grace/elements/Arc.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

namespace grace::decorators::joins
{
    using join_func_t = std::vector<Point_r> const& (std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2);

    inline Point_r miter_point(Point_r const& cen, real_t hw, real_t dir1, real_t dir2)
    {
        real_t const half = 0.5f*(dir2 - dir1);
        real_t const dir = dir1 + half - pi_2;
        return cen + Vector_r::polar(hw/cos(half), dir);
    }

    inline std::vector<Point_r> const& round(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2)
    {
        buf.clear();
        if(dir2 > dir1)
            push_back(buf) << elements::Arc(cen, hw, dir1-pi_2, dir2 - pi_2);
        else
            buf.push_back(miter_point(cen, hw, dir1, dir2));
        return buf;
    }

    inline std::vector<Point_r> const& bevel(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2)
    {
        buf.clear();
        if(dir2 > dir1)
        {
            buf.push_back(cen + Vector_r::polar(hw, dir1-pi_2));
            buf.push_back(cen + Vector_r::polar(hw, dir2-pi_2));
        }
        else
            buf.push_back(miter_point(cen, hw, dir1, dir2));
        return buf;
    }

    inline std::vector<Point_r> const& miter(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2)
    {
        buf.clear();
        buf.push_back(miter_point(cen, hw, dir1, dir2));
        return buf;
    }

    struct Polygonal
    {
        int const sides = 2;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2)
        {
            buf.clear();
            if(dir2 > dir1)
                push_back(buf) << elements::Arc(sides, cen, hw, dir1-pi_2, dir2 - pi_2);
            else
                buf.push_back(miter_point(cen, hw, dir1, dir2));
            return buf;
        }
    };
}