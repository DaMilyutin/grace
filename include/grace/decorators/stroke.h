#pragma once
#include <grace/algebra/rules.h>
#include <grace/types/Point.h>

#include <grace/elements/Arc.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <functional>
#include <vector>

namespace grace
{
    namespace decorators
    {
        namespace joins
        {
            using join_func_t = std::vector<Point_r> const& (std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter);

            inline std::vector<Point_r> const& round(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                buf.clear();
                if(dir2 > dir1)
                    push_back(buf) << elements::Arc(cen, hw, dir1-M_PI_2, dir2 - M_PI_2);
                else
                    buf.push_back(miter);
                return buf;
            }

            inline std::vector<Point_r> const& bevel(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                buf.clear();
                if(dir2 > dir1)
                {
                    buf.push_back(cen + Vector_r::polar(hw, dir1-M_PI_2));
                    buf.push_back(cen + Vector_r::polar(hw, dir2-M_PI_2));
                }
                else
                    buf.push_back(miter);
                return buf;
            }

            inline std::vector<Point_r> const& miter(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& m)
            {
                buf.clear();
                buf.push_back(m);
                return buf;
            }

            struct Polygonal
            {
                int const sides = 2;

                inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& m)
                {
                    buf.clear();
                    if(dir2 > dir1)
                        push_back(buf) << elements::Arc(sides, cen, hw, dir1-M_PI_2, dir2 - M_PI_2);
                    else
                        buf.push_back(m);
                    return buf;
                }
            };
        }

        namespace caps
        {
            using cap_func_t = std::vector<Point_r> const& (std::vector<Point_r>& buf, Point_r const& endp, real_t hw, real_t dir);

            inline std::vector<Point_r> const& butt(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
            {
                buf.clear();
                Vector_r const v = Vector_r::polar(hw, dir + M_PI_2);
                buf.push_back(e - v);
                buf.push_back(e + v);
                return buf;
            }

            inline std::vector<Point_r> const& projecting(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
            {
                buf.clear();
                Vector_r const v = Vector_r::polar(hw, dir + M_PI_2);
                Point_r  const p = e + Vector_r{v.y, -v.x};
                buf.push_back(p - v);
                buf.push_back(p + v);
                return buf;
            }

            inline std::vector<Point_r> const& round(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
            {
                buf.clear();
                push_back(buf) << elements::Arc(e, hw, dir - M_PI_2, dir + M_PI_2);
                return buf;
            }

            struct Polygonal
            {
                int const sides = 2;

                inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
                {
                    buf.clear();
                    push_back(buf) << elements::Arc(sides, e, hw, dir - M_PI_2, dir + M_PI_2);
                    return buf;
                }
            };

            struct Knob
            {
                int    const sides = 5;

                inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
                {
                    buf.clear();
                    real_t spread = M_PI*(1. - 1./(sides+1));
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

                float length    = 3.f;
                float width     = 3.f;
                float sharpness = 2.f;
                float extent    = 0.f;

                inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
                {
                    buf.clear();
                    Vector_r const e2 = Vector_r::polar(hw, dir + M_PI_2);
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
                    float width = 3.f,
                    float sharpness = 1.f,
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
                    Vector_r const e2 = Vector_r::polar(hw, dir + M_PI_2);
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
                RArrowHead(float length = 3.,
                    float width = 3.,
                    float sharpness = 1.f,
                    float extent = 0.f)
                    : length(length), width(width), sharpness(sharpness), extent(extent)
                {}

                float length = 3.f;
                float width = 3.f;
                float sharpness = 1.f;
                float extent = 0.f;

                inline std::vector<Point_r> const& operator()(std::vector<Point_r>& buf, Point_r const& e, real_t hw, real_t dir)
                {
                    buf.clear();
                    Vector_r const e2 = Vector_r::polar(hw, dir + M_PI_2);
                    Vector_r const e1 = {e2.y, -e2.x};
                    Point_r const p = e + extent*e1;
                    buf.push_back(p - e2 + length*e1);
                    buf.push_back(p + width*e2 - sharpness*e1);
                    buf.push_back(p + e2);
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
                    Vector_r const e2 = Vector_r::polar(hw, dir + M_PI_2);
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

        class Stroke
        {
        public:
            Stroke() = default;
            Stroke(real_t w): halfWidth_(w/2) {}

            Stroke& width(real_t w)
            {
                halfWidth_ = w*0.5f;
                return *this;
            }

            template<typename F>
            Stroke& cap(F&& f)
            {
                cap_ = FWD(f);
                return *this;
            }

            template<typename F>
            Stroke& join(F&& f)
            {
                join_ = FWD(f);
                return *this;
            }

            real_t                            halfWidth_ = 1.0f;
            std::function<caps::cap_func_t>   cap_ = caps::Polygonal{4};
            std::function<joins::join_func_t> join_ = joins::Polygonal{3};
        };

        class FancyStroke
        {
        public:
            FancyStroke() = default;
            FancyStroke(real_t width): halfWidth_(width/2) {}

            FancyStroke& width(real_t w)
            {
                halfWidth_ = w*0.5f;
                return *this;
            }

            template<typename F>
            FancyStroke& cap(F&& f)
            {
                head_ = FWD(f);
                tail_ = FWD(f);
                return *this;
            }

            template<typename F>
            FancyStroke& head(F&& f)
            {
                head_ = FWD(f);
                return *this;
            }

            template<typename F>
            FancyStroke& tail(F&& f)
            {
                tail_ = FWD(f);
                return *this;
            }

            template<typename F>
            FancyStroke& join(F&& f)
            {
                left_  = FWD(f);
                right_ = FWD(f);
                return *this;
            }

            template<typename F>
            FancyStroke& left(F&& f)
            {
                left_ = FWD(f);
                return *this;
            }

            template<typename F>
            FancyStroke& right(F&& f)
            {
                right_ = FWD(f);
                return *this;
            }

            real_t                            halfWidth_ = 1.0f;
            std::function<caps::cap_func_t>   head_      = caps::Polygonal{4};
            std::function<caps::cap_func_t>   tail_      = caps::Polygonal{4};
            std::function<joins::join_func_t> left_      = joins::Polygonal{3};
            std::function<joins::join_func_t> right_     = joins::Polygonal{3};
        };
    }
}