#pragma once
#include <grace/algebra/rules.h>
#include <grace/decorators/shifted.h>

namespace grace
{
    namespace elements
    {
        template<typename S>
        struct ShiftWrapper;

        template<>
        struct ShiftWrapper<decorators::Shift>
        {
            std::vector<Point_r> const& left(Point_r const& cen, real_t hw, real_t dir1, real_t dir2)
            {
                return shift.join_(buffer, cen, hw, dir1, dir2);
            }

            decorators::Shift    shift;
            std::vector<Point_r> buffer;
        };

        template<typename SW>
        class Shifter: public rules::Link<Shifter<SW>>
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
                    annot.push_back(grace::annotations::United_DD(dirV));
                }

                ylems::elements::CycleBuffer<Annot, 2>    annot;
                ylems::elements::CycleBuffer<Point_r, 3>  point;
            };

            static real_t diff_angle(real_t a2, real_t a1)
            {
                real_t d = a2 - a1;
                if(d > 2*pi)
                    d -= 2*pi;
                if(d < -2*pi)
                    d += 2*pi;
                return d;
            }

            static real_t adjust_angle(real_t a, real_t a0)
            {
                real_t const d = a - a0;
                if(d > pi)
                    a -= 2*pi;
                else if(d < -pi)
                    a += 2*pi;
                return a;
            }

            template<typename S>
            bool feed_joint(S& sink, Buffer const& b)
            {
                auto const a1 = b.annot.back(1).direction;
                auto const a2 = adjust_angle(b.annot.back(0).direction, a1);
                auto const& c = b.point.back(1);
                sink << make.left(c, offset(), a1, a2);
                return true;
            }

            template<typename S>
            bool feed_full(S& sink, Buffer& b)
            {
                auto const a1 = b.annot.back(1).direction;
                auto const a2 = adjust_angle(b.annot.back(0).direction, a1);
                Vector_r const mo1 = Vector_r::polar(offset(), a1 - pi_2);
                Vector_r const mo2 = Vector_r::polar(offset(), a2 - pi_2);
                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                sink << m1 - mo1 << make.left(c, offset(), a1, a2) << m2 - mo2;
                return true;
            }

            template<typename S>
            bool feed_short(S& sink, Buffer& b)
            {
                auto const a = b.annot.back(0).direction;
                auto const& m1 = b.point.back(1);
                auto const& m2 = b.point.back(0);
                Vector_r const mo = Vector_r::polar(offset(), a - pi_2);
                sink << m1 - mo << m2 - mo;
                return true;
            }

            template<typename S>
            bool feed_start(S& sink, Buffer& b)
            {
                auto const a1 = b.annot.back(1).direction;
                auto const a2 = adjust_angle(b.annot.back(0).direction, a1);
                Vector_r const mo1 = Vector_r::polar(offset(), a1 - pi_2);
                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                sink << m1 - mo1 << make.left(c, offset(), a1, a2);
                return true;
            }

            template<typename S>
            bool feed_end(S& sink, Buffer& b)
            {
                auto const a1 = b.annot.back(1).direction;
                auto const a2 = adjust_angle(b.annot.back(0).direction, a1);
                Vector_r const mo2 = Vector_r::polar(offset(), a2 - pi_2);
                auto const& m1 = b.point.back(2);
                auto const& c = b.point.back(1);
                auto const& m2 = b.point.back(0);
                sink << make.left(c, offset(), a1, a2) << m2 - mo2;
                return true;
            }

        public:
            Shifter(SW const& rhs)
                : make(rhs)
            {}

            Shifter(SW&& rhs)
                : make(rhs)
            {}

            template<typename S>
            bool feed(S& sink, std::vector<Point_r> const& p)
            {
                return transfuse(p, sink);
            }

            template<typename S>
            bool feed(S&, Point_r const&)
            {
                return false;
            }

            template<typename S>
            bool tranfuse(std::vector<Point_r> const& p, S& sink)
            {
                static int dbg = 0;
                ++dbg;
                if(p.size() < 2)
                    return true;

                bool const closed = dot(p.front() - p.back()) < 1.e-2f;
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
                    b.initialize(p[p.size()-2], p[p.size()-1]);
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
        private:
            SW  make;
            real_t offset() const {
                return make.shift.offset_;
            }
        };
    }

    namespace decorators
    {
        template<typename T>
        auto operator/(T&& term, Shift const& s)
        {
            return FWD(term)/elements::Shifter(elements::ShiftWrapper<Shift>{s});
        }

        template<typename T>
        auto operator/(T&& term, Shift&& s)
        {
            return FWD(term)/elements::Shifter(elements::ShiftWrapper<Shift>{std::move(s)});
        }


        template<typename T>
        auto operator/(Shift const& s, T&& term)
        {
            return elements::Shifter(elements::ShiftWrapper<Shift>{s})/FWD(term);
        }

        template<typename T>
        auto operator/(Shift&& s, T&& term)
        {
            return elements::Shifter(elements::ShiftWrapper<Shift>{std::move(s)})/FWD(term);
        }

    }
}

namespace ylems::rules
{
    template<typename Y, typename SW, typename Sink>
    bool transfuse(Y&& y, grace::rules::LinkSink<grace::elements::Shifter<SW>, Sink>&& ls)
    {
        auto&& l = FWD(ls).link;
        auto&& s = FWD(ls).sink;
        FWD(l).transfuse(FWD(y), FWD(s));
    }
}