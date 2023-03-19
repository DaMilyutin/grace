#pragma once
#include <grace/algebra/rules.h>
#include <grace/types/Point.h>

namespace grace
{
    namespace annotations
    {
        struct Distance
        {
            Distance() = default;
            Distance(real_t d): distance(d) {}
            Distance(Vector_r const& v): distance(norm(v)) {}

            Distance(Vector_r const& v, Distance const&): distance(norm(v)) {} // for generic

            real_t distance = 0.0f;
        };

        struct Direction
        {
            Direction() = default;
            Direction(real_t d): direction(d) {}
            Direction(Vector_r const& v): direction(arg(v)) {}

            Direction(Vector_r const& v, Direction const&): direction(arg(v)) {} // for generic

            real_t direction = 0.0f;
        };

        struct CurveLength
        {
            CurveLength() = default;

            CurveLength(real_t d): length(d) {}
            CurveLength(real_t d, real_t prev = 0.f): length(d + prev) {}
            CurveLength(Vector_r const& v, CurveLength const& prev = CurveLength()): length(norm(v) + prev.length) {}
            CurveLength(Distance const& d, CurveLength const& prev = CurveLength()): length(d.distance + prev.length) {}

            real_t length = 0.0f;
        };

        template<typename... Ts>
        struct United;

        template<>
        struct United<Distance, CurveLength>: Distance, CurveLength
        {
            United() = default;
            United(United&&) = default;
            United(United const&) = default;
            United& operator=(United&&) = default;
            United& operator=(United const&) = default;

            United(real_t d, real_t prev = 0.f): Distance(d), CurveLength(d, prev) {}

            United(Vector_r const& v, CurveLength const& prev = CurveLength()): Distance(norm(v)), CurveLength(Distance::distance, prev) {}
        };

        template<>
        struct United<Distance, Direction>: Distance, Direction
        {
            United() = default;
            United(United&&) = default;
            United(United const&) = default;
            United& operator=(United&&) = default;
            United& operator=(United const&) = default;

            United(real_t dist, real_t dir): Distance(dist), Direction(dir) {}
            United(Vector_r const& v): United(norm(v), arg(v)) {}
        };

        template<>
        struct United<Distance, Direction, CurveLength>: Distance, Direction, CurveLength
        {
            United() = default;
            United(United&&) = default;
            United(United const&) = default;
            United& operator=(United&&) = default;
            United& operator=(United const&) = default;


            United(real_t d, real_t a, real_t prev = 0.f): Distance(d), Direction(a), CurveLength(d, prev) {}
            United(Vector_r const& v, CurveLength const& prev = CurveLength()): United(norm(v), arg(v), prev.length) {}
        };

        using United_DL = United<Distance, CurveLength>;
        using United_DD = United<Distance, Direction>;
        using United_DDL = United<Distance, Direction, CurveLength>;
    }
}