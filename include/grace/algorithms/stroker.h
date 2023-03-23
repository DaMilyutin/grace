#pragma once
#include <grace/algebra/rules.h>
#include <grace/decorators/stroke.h>

namespace grace
{
    namespace elements
    {
        template<typename S>
        struct StrokeWrapper;

        template<>
        struct StrokeWrapper<decorators::Stroke>
        {
            std::vector<Point_r> const& head(Point_r const& endp, real_t hw, real_t dir)
            {
                return stroke.cap_(buffer, endp, hw, dir);
            }

            std::vector<Point_r> const& tail(Point_r const& endp, real_t hw, real_t dir)
            {
                return stroke.cap_(buffer, endp, hw, dir);
            }

            std::vector<Point_r> const& left(Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                return stroke.join_(buffer, cen, hw, dir1, dir2, miter);
            }

            std::vector<Point_r> const& right(Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                return stroke.join_(buffer, cen, hw, dir1, dir2, miter);
            }

            decorators::Stroke   stroke;
            std::vector<Point_r> buffer;
        };

        template<>
        struct StrokeWrapper<decorators::FancyStroke>
        {
            std::vector<Point_r> const& head(Point_r const& endp, real_t hw, real_t dir)
            {
                return stroke.head_(buffer, endp, hw, dir);
            }

            std::vector<Point_r> const& tail(Point_r const& endp, real_t hw, real_t dir)
            {
                return stroke.tail_(buffer, endp, hw, dir);
            }

            std::vector<Point_r> const& left(Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                return stroke.left_(buffer, cen, hw, dir1, dir2, miter);
            }

            std::vector<Point_r> const& right(Point_r const& cen, real_t hw, real_t dir1, real_t dir2, Point_r const& miter)
            {
                return stroke.right_(buffer, cen, hw, dir1, dir2, miter);
            }

            decorators::FancyStroke   stroke;
            std::vector<Point_r>      buffer;
        };

        template<typename SW>
        class Stroker: public rules::Link<Stroker<SW>>
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
                return fabs(halfWidth()*tan(a));
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
                Vector_r const co = Vector_r::polar(halfWidth()/cosf(half), a1 + half - M_PI_2);
                Vector_r const mo2 = Vector_r::polar(halfWidth(), a2 - M_PI_2);
                sink << rules::start;
                sink << make.tail(m1, halfWidth(), a1 + M_PI)
                     << make.left(c, halfWidth(), a1, a2, c + co)
                     << m2 + mo2 << m2 - mo2
                     << make.right(c, halfWidth(), a2 + M_PI, a1 + M_PI, c - co);
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
                Vector_r const co = Vector_r::polar(halfWidth()/cosf(half), a1 + half - M_PI_2);
                Vector_r const mo1 = Vector_r::polar(halfWidth(), a1 - M_PI_2);
                sink << rules::start;
                sink << m1 - mo1 << m1 + mo1 << make.left(c, halfWidth(), a1, a2, c + co)
                     << make.head(m2, halfWidth(), a2) << make.right(c, halfWidth(), a2+M_PI, a1+M_PI, c - co);
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
                Vector_r const co = Vector_r::polar(halfWidth()/cosf(half), a1 + half - M_PI_2);
                sink << rules::start;
                sink << make.tail(m1, halfWidth(), a1 + M_PI) << make.left(c, halfWidth(), a1, a2, c + co)
                     << make.head(m2, halfWidth(), a2) << make.right(c, halfWidth(), a2+M_PI, a1+M_PI, c - co);
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
                sink << make.tail(m1, halfWidth(), a + M_PI)
                     << make.head(m2, halfWidth(), a);
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
                Vector_r const co = Vector_r::polar(halfWidth()/cosf(half), a1 + half - M_PI_2);
                Vector_r const mo1 = Vector_r::polar(halfWidth(), a1 - M_PI_2);
                Vector_r const mo2 = Vector_r::polar(halfWidth(), a2 - M_PI_2);
                sink << rules::start;
                sink << m1 + mo1 << make.left(c, halfWidth(), a1, a2, c + co)  << m2 + mo2
                     << m2 - mo2 << make.right(c, halfWidth(), a2+M_PI, a1+M_PI, c - co)  << m1 - mo1;
                sink << rules::close;
                return true;
            }

        public:
            Stroker(SW const& rhs)
                : make(rhs)
            {}

            Stroker(SW&& rhs)
                : make(rhs)
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
        private:
            SW  make;
            real_t halfWidth() const {
                return make.stroke.halfWidth_;
            }
        };
    }

    namespace decorators
    {
        template<typename T>
        auto operator/(T&& term, Stroke const& s)
        {
            return FWD(term)/elements::Stroker(elements::StrokeWrapper<Stroke>{s});
        }

        template<typename T>
        auto operator/(T&& term, Stroke&& s)
        {
            return FWD(term)/elements::Stroker(elements::StrokeWrapper<Stroke>{std::move(s)});
        }

        template<typename T>
        auto operator/(T&& term, FancyStroke const& s)
        {
            return FWD(term)/elements::Stroker(elements::StrokeWrapper<FancyStroke>{s});
        }

        template<typename T>
        auto operator/(T&& term, FancyStroke&& s)
        {
            return FWD(term)/elements::Stroker(elements::StrokeWrapper<FancyStroke>{std::move(s)});
        }


        template<typename T>
        auto operator/(Stroke const& s, T&& term)
        {
            return elements::Stroker(elements::StrokeWrapper<Stroke>{s})/FWD(term);
        }

        template<typename T>
        auto operator/(Stroke&& s, T&& term)
        {
            return elements::Stroker(elements::StrokeWrapper<Stroke>{std::move(s)})/FWD(term);
        }

        template<typename T>
        auto operator/(FancyStroke const& s, T&& term)
        {
            return elements::Stroker(elements::StrokeWrapper<FancyStroke>{s})/FWD(term);
        }

        template<typename T>
        auto operator/(FancyStroke&& s, T&& term)
        {
            return elements::Stroker(elements::StrokeWrapper<FancyStroke>{std::move(s)})/FWD(term);
        }
    }


}