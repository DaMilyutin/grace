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
        }


        class Stroke: rules::Decorator<Stroke>
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
    }
}