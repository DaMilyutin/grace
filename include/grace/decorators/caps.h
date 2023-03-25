#pragma once
#include <grace/algebra/rules.h>
#include <grace/algebra/rules.h>
#include <grace/types/Point.h>

#include <grace/elements/Arc.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

namespace grace::decorators::caps
{
    using cap_func_t = std::vector<Point_r> const& (std::vector<Point_r>& buf, Point_r const& endp, real_t hw, real_t dir);

    inline std::vector<Point_r> const& butt(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
    {
        buf.clear();
        Vector_r const v = Vector_r::polar(hw, dir + pi_2);
        buf.push_back(e - v);
        buf.push_back(e + v);
        return buf;
    }

    inline std::vector<Point_r> const& projecting(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
    {
        buf.clear();
        Vector_r const v = Vector_r::polar(hw, dir + pi_2);
        Point_r  const p = e + Vector_r{v.y, -v.x};
        buf.push_back(p - v);
        buf.push_back(p + v);
        return buf;
    }

    inline std::vector<Point_r> const& round(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
    {
        buf.clear();
        push_back(buf) << elements::Arc(e, hw, dir - pi_2, dir + pi_2);
        return buf;
    }

    struct Polygonal
    {
        int const sides = 2;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            push_back(buf) << elements::Arc(sides, e, hw, dir - pi_2, dir + pi_2);
            return buf;
        }
    };

    struct Knob
    {
        int    const sides = 5;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            real_t spread = pi*(1.f - 1.f/(sides+1));
            real_t r = hw/sin(spread);
            push_back(buf) << elements::Arc(sides, e, r, dir - spread, dir + spread);
            return buf;
        }
    };

    struct ArrowHead
    {
        ArrowHead(float length = 3.f,
            float width = 3.f,
            float sharpness = 2.f,
            float extent = 0.f)
            : length(length), width(width), sharpness(sharpness), extent(extent)
        {}

        float length = 3.f;
        float width = 3.f;
        float sharpness = 2.f;
        float extent = 0.f;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            Vector_r const e2 = Vector_r::polar(hw, dir + pi_2);
            Vector_r const e1 = {e2.y, -e2.x};
            Point_r const p = e + extent*e1;
            buf.push_back(p - e2);
            buf.push_back(p - width*e2 - sharpness*e1);
            buf.push_back(p  + length*e1);
            buf.push_back(p + width*e2 - sharpness*e1);
            buf.push_back(p + e2);
            return buf;
        }
    };

    struct LArrowHead
    {
        LArrowHead(float length = 3.f,
            float width = 2.f,
            float sharpness = 0.5f,
            float extent = 0.f)
            : length(length), width(width), sharpness(sharpness), extent(extent)
        {}

        float length = 2.f;
        float width = 2.f;
        float sharpness = 0.5f;
        float extent = 0.f;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            Vector_r const e2 = Vector_r::polar(hw, dir + pi_2);
            Vector_r const e1 = {e2.y, -e2.x};
            Point_r const p = e + extent*e1;
            buf.push_back(p - e2);
            buf.push_back(p - width*e2 - sharpness*e1);
            buf.push_back(p + e2 + length*e1);
            return buf;
        }
    };

    struct RArrowHead
    {
        RArrowHead(float length = 3.f,
            float width = 2.f,
            float sharpness = 0.5f,
            float extent = 0.f)
            : length(length), width(width), sharpness(sharpness), extent(extent)
        {}

        float length = 2.f;
        float width = 2.f;
        float sharpness = 0.5f;
        float extent = 0.f;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            Vector_r const e2 = Vector_r::polar(hw, dir + pi_2);
            Vector_r const e1 = {e2.y, -e2.x};
            Point_r const p = e + extent*e1;
            buf.push_back(p - e2 + length*e1);
            buf.push_back(p + width*e2 - sharpness*e1);
            buf.push_back(p + e2);
            return buf;
        }
    };

    struct Skew
    {
        Skew(float left = 0.f,
            float right = 0.f)
            : left(left), right(right)
        {}

        float left = 0.f;
        float right = 0.f;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            Vector_r const e2 = Vector_r::polar(hw, dir + pi_2);
            Vector_r const e1 = {e2.y, -e2.x};
            buf.push_back(e - e2 + left*e1);
            buf.push_back(e + e2 + right*e1);
            return buf;
        }
    };

    struct FlatArrowTail
    {
        FlatArrowTail(float length = 2.f,
            float extent = 0.f)
            : length(length), extent(extent)
        {}

        float length = 2.f;
        float extent = 0.f;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            Vector_r const e2 = Vector_r::polar(hw, dir + pi_2);
            Vector_r const e1 = {e2.y, -e2.x};
            Point_r const p = e + extent*e1;
            buf.push_back(p - e2 + length*e1);
            buf.push_back(p);
            buf.push_back(p + e2 + length*e1);
            return buf;
        }
    };

    struct ArrowTail
    {
        ArrowTail(float length = 0.f,
            float width = 1.f,
            float sharpness = 1.f,
            float extent = 0.f)
            : length(length), width(width), sharpness(sharpness), extent(extent)
        {}

        float length = 0.f;
        float width = 1.f;
        float sharpness = 1.f;
        float extent = 0.f;

        inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
        {
            buf.clear();
            Vector_r const e2 = Vector_r::polar(hw, dir + pi_2);
            Vector_r const e1 = {e2.y, -e2.x};
            Point_r const p = e + extent*e1;
            buf.push_back(p - e2);
            buf.push_back(p - width*e2 + sharpness*e1);
            buf.push_back(p + length*e1);
            buf.push_back(p + width*e2 + sharpness*e1);
            buf.push_back(p + e2);
            return buf;
        }
    };
}