#pragma once
#include <grace/algebra/rules.h>
#include <grace/decorators/stroke.h>

namespace grace
{
    namespace elements
    {
        class Stroker: public rules::Link<Stroker>
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

            static real_t diff_angle(real_t a2, real_t a1)
            {
                real_t d = a2 - a1;
                if(d > 2*M_PI)
                    d -= 2*M_PI;
                if(d < -2*M_PI)
                    d += 2*M_PI;
                return d;
            }

            void extend(Point_r const& s, Annot const& ann_s, Point_r& m, Point_r& e, Annot& ann_e, real_t const l)
            {
                if(ann_e.distance >= l || ann_e.distance == 0.f)
                    return;
                if(ann_e.distance < 0.5f*l)
                {
                    ann_e.distance = 0.f;
                    ann_e.direction = ann_s.direction;
                    m = e = s;
                }
                else
                {
                    auto const diff = (e-s)*(l/ann_e.distance);
                    e = s + diff;
                    m = s + 0.5f*diff;
                    ann_e.distance = l;
                }
            }

            real_t get_lim(real_t a) const
            {
                if(a >  2*M_PI) a -= 2*M_PI;
                if(a < -2*M_PI) a += 2*M_PI;
                return fabs(opt_.halfWidth_*tan(a));
            }

            template<typename S>
            bool feed_start(S& sink, Buffer& b)
            {
                auto const lim = get_lim(0.5f*(b.annot.back(0).direction - b.annot.back(1).direction));
                extend(b.point.back(1), b.annot.back(0), b.mid.back(1), b.point.back(2), b.annot.back(1), lim);

                auto const a2 = b.annot.back(0).direction;
                auto const a1 = b.annot.back(1).direction;
                auto const half = 0.5f*diff_angle(a2, a1);
                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.mid.back(0);
                Vector_r const co = Vector_r::polar(opt_.halfWidth_/cosf(half), a1 + half - M_PI_2);
                Vector_r const mo2 = Vector_r::polar(opt_.halfWidth_, a2 - M_PI_2);
                sink << rules::start;
                sink << make_cap(m1, opt_.halfWidth_, a1 + M_PI)
                     << make_join(c, opt_.halfWidth_, a1, a2, c + co)
                     << m2 + mo2 << m2 - mo2
                     << make_join(c, opt_.halfWidth_, a2 + M_PI, a1 + M_PI, c - co);
                sink << rules::close;
                return true;
            }

            template<typename S>
            bool feed_end(S& sink, Buffer& b)
            {
                auto const lim = get_lim(0.5f*(b.annot.back(0).direction - b.annot.back(1).direction));
                extend(b.point.back(1), b.annot.back(1), b.mid.back(0), b.point.back(0), b.annot.back(0), lim);

                auto const a2 = b.annot.back(0).direction;
                auto const a1 = b.annot.back(1).direction;
                auto const half = 0.5f*diff_angle(a2, a1);
                auto const& m1 = b.mid.back(1);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                Vector_r const co = Vector_r::polar(opt_.halfWidth_/cosf(half), a1 + half - M_PI_2);
                Vector_r const mo1 = Vector_r::polar(opt_.halfWidth_, a1 - M_PI_2);
                sink << rules::start;
                sink << m1 - mo1 << m1 + mo1 << make_join(c, opt_.halfWidth_, a1, a2, c + co)
                     << make_cap(m2, opt_.halfWidth_, a2) << make_join(c, opt_.halfWidth_, a2+M_PI, a1+M_PI, c - co);
                sink << rules::close;
                return true;
            }

            template<typename S>
            bool feed_full(S& sink, Buffer& b)
            {
                auto const lim = get_lim(0.5f*(b.annot.back(0).direction - b.annot.back(1).direction));
                if(b.annot.back(1).distance < b.annot.back(0).distance)
                    extend(b.point.back(1), b.annot.back(0), b.mid.back(1), b.point.back(2), b.annot.back(1), lim);
                else
                    extend(b.point.back(1), b.annot.back(1), b.mid.back(0), b.point.back(0), b.annot.back(0), lim);
                auto const a2 = b.annot.back(0).direction;
                auto const a1 = b.annot.back(1).direction;
                auto const half = 0.5f*diff_angle(a2, a1);
                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                Vector_r const co = Vector_r::polar(opt_.halfWidth_/cosf(half), a1 + half - M_PI_2);
                sink << rules::start;
                sink << make_cap(m1, opt_.halfWidth_, a1 + M_PI) << make_join(c, opt_.halfWidth_, a1, a2, c + co)
                     << make_cap(m2, opt_.halfWidth_, a2) << make_join(c, opt_.halfWidth_, a2+M_PI, a1+M_PI, c - co);
                sink << rules::close;
                return true;
            }

            template<typename S>
            bool feed_short(S& sink, Buffer& b)
            {
                auto const a = b.annot.back(0).direction;
                auto const& m1 = b.point.back(1);
                auto const& m2 = b.point.back(0);
                sink << rules::start;
                sink << make_cap(m1, opt_.halfWidth_, a + M_PI)
                     << make_cap(m2, opt_.halfWidth_, a);
                sink << rules::close;
                return true;
            }


            template<typename S>
            bool feed_joint(S& sink, Buffer const& b)
            {
                auto const a2 = b.annot.back(0).direction;
                auto const a1 = b.annot.back(1).direction;
                auto const half = 0.5f*(a2 - a1);
                auto const& m1 = b.mid.back(1);
                auto const& c = b.point.back(1);
                auto const& m2 = b.mid.back(0);
                Vector_r const co = Vector_r::polar(opt_.halfWidth_/cosf(half), a1 + half - M_PI_2);
                Vector_r const mo1 = Vector_r::polar(opt_.halfWidth_, a1 - M_PI_2);
                Vector_r const mo2 = Vector_r::polar(opt_.halfWidth_, a2 - M_PI_2);
                sink << rules::start;
                sink << m1 + mo1 << make_join(c, opt_.halfWidth_, a1, a2, c + co)  << m2 + mo2
                     << m2 - mo2 << make_join(c, opt_.halfWidth_, a2+M_PI, a1+M_PI, c - co)  << m1 - mo1;
                sink << rules::close;
                return true;
            }

        public:
            Stroker() = default;

            Stroker(real_t width)
                : opt_(width*0.5f)
            {}

            Stroker(decorators::Stroke const& rhs)
                : opt_(rhs)
            {}

            Stroker(decorators::Stroke&& rhs)
                : opt_(rhs)
            {}

            template<typename S>
            bool feed(S& sink, std::vector<Point_r> const& p)
            {
                static int dbg = 0;
                ++dbg;
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
                    auto const count = p.size() - 1;
                    while(++i < count)
                    {
                        b.push_back(p[i]);
                        feed_joint(sink, b);
                    }
                    b.push_back(p[i]);
                    feed_end(sink, b);
                }
                return true;
            }


            // for case we want to reuse buffers
            Stroker& width(real_t w)
            {
                opt_.halfWidth_ = w/2;
                return *this;
            }

            template<typename F>
            Stroker& cap(F&& f)
            {
                opt_.cap_ = FWD(f);
                return *this;
            }

            template<typename F>
            Stroker& join(F&& f)
            {
                opt_.join_ = FWD(f);
                return *this;
            }

        private:
            std::vector<Point_r> const& make_cap(Point_r const& endp, real_t hw, real_t dir)
            {
                return opt_.cap_(buffer_, endp, hw, dir);
            }

            std::vector<Point_r> const& make_join(Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                return opt_.join_(buffer_, cen, hw, dir1, dir2, miter);
            }

            decorators::Stroke         opt_;
            std::vector<Point_r>       buffer_;
        };
    }

    namespace decorators
    {
        template<typename Y>
        auto operator/(rules::Yield<Y>&& y, Stroke const& s)
        {
            return FWD(y)._get_()/elements::Stroker(s);
        }

        template<typename Y>
        auto operator/(rules::Yield<Y>&& y, Stroke&& s)
        {
            return FWD(y)._get_()/elements::Stroker(std::move(s));
        }
    }


}