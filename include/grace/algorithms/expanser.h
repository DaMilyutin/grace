#pragma once
#include <grace/algebra/rules.h>
#include <grace/decorators/extrudes.h>
#include <grace/decorators/joins.h>

#include <grace/elements/arc.h>

#include <optional>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

namespace grace
{
    namespace elements
    {
        struct Joint: rules::Yield<Joint>
        {
            Joint(Point_r piv, real_t w, real_t d1, real_t d2, Point_r alt)
                : pivot(piv), wid(w), dir1(d1), dir2(d2), alt(alt)
            {}

            Point_r pivot;
            real_t  wid;
            real_t  dir1;
            real_t  dir2;
            Point_r alt;
        };

        template<typename S>
        S& operator<<(rules::Sink<S>& s, Joint const& j)
        {
            S& the_sink = s._get_();
            if(j.dir2 > j.dir1)
                return s << Arc(j.pivot, j.wid, j.dir1-M_PI_2, j.dir2 - M_PI_2);
            s << j.alt;
        }


        auto round_cap(Point_r const& e, real_t w, real_t dir)
        {
            return Arc(e, 0.5f*w, dir - M_PI_2, dir + M_PI_2);
        }

        class Expanser: public rules::Link<Expanser>
        {
            using Annot = grace::annotations::United_DD;

            struct Buffer
            {
                void initialize(Point_r p1, Point_r p2)
                {
                    point.push_back(p1);
                    point.push_back(p2);
                    push_annot();
                }

                void push_back(Point_r p)
                {
                    point.push_back(p);
                    push_annot();
                }

                void push_annot()
                {
                    assert(point.size()>=2);
                    auto const& p1 = point.back(1);
                    auto const& p2 = point.back(0);
                    auto dirV = p2 - p1;
                    mid.push_back(p1 + 0.5f*dirV);
                    annot.push_back(grace::annotations::United_DD(dirV));
                }

                ylems::elements::CycleBuffer<Annot, 2>    annot;
                ylems::elements::CycleBuffer<Point_r, 2>  mid;
                ylems::elements::CycleBuffer<Point_r, 3>  point;
            };

            static real_t adjust_angle(real_t a2, real_t a1)
            {
                if(a2 - a1 > 2*agge::pi)
                    a2 -= 2*agge::pi;
                if(a2 - a1 < -2*agge::pi)
                    a2 += 2*agge::pi;
                return a2;
            }

            void extend(Point_r const& s, Point_r& m, Point_r& e, real_t& d, real_t const l)
            {
                if(d >= l || d == 0.f)
                    return;
                auto const diff = (e-s)*(l/d);
                e = s + diff;
                m = s + 0.5f*diff;
                d = l;
            }

            real_t get_lim(real_t a) const
            {
                return fabs(0.5f*extrude_.width*tan(a));
            }

            template<typename S>
            bool feed_start(S& sink, Buffer& b)
            {
                auto const a2 = b.annot.back(0).direction;
                auto const a1 = adjust_angle(b.annot.back(1).direction, a2);
                auto const half = 0.5f*(a2 - a1);
                auto const lim = get_lim(half);
                extend(b.point.back(1), b.mid.back(1), b.point.back(2), b.annot.back(1).distance, lim);

                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.mid.back(0);
                Vector_r const co = Vector_r::polar(extrude_.width/(2.f*cosf(half)), a1 + half - agge::pi*0.5f);
                Vector_r const mo1 = Vector_r::polar(extrude_.width*0.5f, a1 - agge::pi*0.5f);
                Vector_r const mo2 = Vector_r::polar(extrude_.width*0.5f, a2 - agge::pi*0.5f);
                sink << rules::start;
                sink << round_cap(m1, extrude_.width, a1 + M_PI)
                     << m1 + mo1 << Joint(c, extrude_.width/2, a1, a2, c + co) << m2 + mo2
                     << m2 - mo2 << Joint(c, extrude_.width/2, a2 + M_PI, a1+ M_PI, c - co) << m1 - mo1;
                sink << rules::close;
                return true;
            }

            template<typename S>
            bool feed_end(S& sink, Buffer& b)
            {
                auto const a2 = b.annot.back(0).direction;
                auto const a1 = adjust_angle(b.annot.back(1).direction, a2);
                auto const half = 0.5f*(a2 - a1);
                auto const lim = get_lim(half);
                extend(b.point.back(1), b.mid.back(0), b.point.back(0), b.annot.back(0).distance, lim);
                auto const& m1 = b.mid.back(1);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                Vector_r const co = Vector_r::polar(extrude_.width/(2.f*cosf(half)), a1 + half - agge::pi*0.5f);
                Vector_r const mo1 = Vector_r::polar(extrude_.width*0.5f, a1 - agge::pi*0.5f);
                Vector_r const mo2 = Vector_r::polar(extrude_.width*0.5f, a2 - agge::pi*0.5f);
                sink << rules::start;
                sink << m1 + mo1 << Joint(c, extrude_.width/2, a1, a2, c + co)  << m2 + mo2
                     << round_cap(m2, extrude_.width, a2)
                     << m2 - mo2 << Joint(c, extrude_.width/2, a2+M_PI, a1+M_PI, c - co) << m1 - mo1;
                sink << rules::close;
                return true;
            }

            template<typename S>
            bool feed_full(S& sink, Buffer& b)
            {
                auto const a2 = b.annot.back(0).direction;
                auto const a1 = adjust_angle(b.annot.back(1).direction, a2);
                auto const half = 0.5f*(a2 - a1);
                auto const lim = get_lim(half);
                extend(b.point.back(1), b.mid.back(1), b.point.back(2), b.annot.back(1).distance, lim);
                extend(b.point.back(1), b.mid.back(0), b.point.back(0), b.annot.back(0).distance, lim);
                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                Vector_r const co = Vector_r::polar(extrude_.width/(2.f*cosf(half)), a1 + half - agge::pi*0.5f);
                Vector_r const mo1 = Vector_r::polar(extrude_.width*0.5f, a1 - agge::pi*0.5f);
                Vector_r const mo2 = Vector_r::polar(extrude_.width*0.5f, a2 - agge::pi*0.5f);
                sink << rules::start;
                sink << round_cap(m1, extrude_.width, a1 + M_PI)
                     << m1 + mo1 << Joint(c, extrude_.width/2, a1, a2, c + co)  << m2 + mo2
                     << round_cap(m2, extrude_.width, a2)
                     << m2 - mo2 << Joint(c, extrude_.width/2, a2+M_PI, a1+M_PI, c - co)  << m1 - mo1;
                sink << rules::close;
                return true;
            }

            template<typename S>
            bool feed_short(S& sink, Buffer& b)
            {
                auto const a = b.annot.back(0).direction;
                auto const& m1 = b.point.back(1);
                auto const& m2 = b.point.back(0);
                Vector_r const o = Vector_r::polar(extrude_.width*0.5f, a - agge::pi*0.5f);
                sink << rules::start;
                sink << round_cap(m1, extrude_.width, a + M_PI)
                     << m1 + o << m2 + o
                     << round_cap(m2, extrude_.width, a)
                     << m2 - o << m1 - o;
                sink << rules::close;
                return true;
            }


            template<typename S>
            bool feed_joint(S& sink, Buffer const& b) const
            {
                auto const a2 = b.annot.back(0).direction;
                auto const a1 = adjust_angle(b.annot.back(1).direction, a2);
                auto const half = 0.5f*(a2 - a1);
                auto const& m1 = b.mid.back(1);
                auto const& c = b.point.back(1);
                auto const& m2 = b.mid.back(0);
                Vector_r const co = Vector_r::polar(extrude_.width/(2.f*cosf(half)), a1 + half - agge::pi*0.5f);
                Vector_r const mo1 = Vector_r::polar(extrude_.width*0.5f, a1 - agge::pi*0.5f);
                Vector_r const mo2 = Vector_r::polar(extrude_.width*0.5f, a2 - agge::pi*0.5f);
                sink << rules::start;
                sink << m1 + mo1 << c + co  << m2 + mo2
                     << m2 - mo2 << c - co  << m1 - mo1;
                sink << rules::close;
                return true;
            }

        public:
            Expanser(extrudes::Ortho const& o)
                : extrude_(o)
            {}

            template<typename S>
            bool feed(S& sink, std::vector<Point_r> const& p)
            {
                if(p.size() < 2)
                    return true;

                bool const closed = dot(p.front() - p.back()) < 1.e-20f;
                Buffer b;
                if(p.size() == 2)
                {
                    if(closed)
                        return true;
                    b.initialize(p[0], p[1]);
                    return feed_short(sink, b);;
                }


                size_t i = 1;
                if(closed)
                {
                    b.initialize(p[p.size()-2], p[0]);
                    b.push_back(p[1]);
                    feed_joint(sink, b);
                    while(++i < p.size())
                    {
                        b.push_back(p[i]);
                        feed_joint(sink, b);
                    }
                }
                else
                {//
                    b.initialize(p[0], p[1]);
                    b.push_back(p[++i]);
                    if(p.size() == 3)
                        return feed_full(sink, b);


                    feed_start(sink, b);
                    //feed_joint(sink, b);
                    auto const count = p.size() - 1;
                    while(++i < count)
                    {
                        b.push_back(p[i]);
                        feed_joint(sink, b);
                    }
                    //feed_joint(sink, b);
                    feed_end(sink, b);
                }
                return true;
            }

        private:
            extrudes::Ortho extrude_;
        };

    }


}