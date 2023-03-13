#pragma once
#include <grace/algebra/rules.h>
#include <vector>

namespace grace
{
    using PathPoints = std::vector<Point_r>;

    namespace annotations
    {
        struct Distance
        {
            Distance() = default;
            Distance(real_t d): distance(d) {}
            Distance(Vector_r const& v): distance(norm(v)) {}
            Distance(Vector_r const& v, Distance const&): distance(norm(v)) {}

            real_t distance = 0.0f;
        };

        struct Direction
        {
            Direction() = default;
            Direction(real_t d): direction(d) {}
            Direction(Vector_r const& v): direction(atan2(v.y, v.x)) {}
            Direction(Vector_r const& v, Direction const&): direction(atan2(v.y, v.x)) {}

            real_t direction = 0.0f;
        };

        struct Path
        {
            Path() = default;

            Path(real_t p): path(p) {}
            Path(Vector_r const& v, Path prev = Path()): path(norm(v) + prev.path) {}
            Path(Distance d, Path prev = Path()): path(d.distance + prev.path) {}

            real_t path = 0.0f;
        };

        template<typename... Ts>
        struct United: Ts...
        {
            static United from(Vector_r const& v, real_t prev = 0.0f) { return ; }
        };

        template<typename S, typename A>
        struct Annotated
        {
            S segment;
            A annotation;
        };

        template<typename E>
        class PathAnnotation
        {
        public:
            PathAnnotation() = default;
            PathAnnotation(PathAnnotation const&) = default;
            PathAnnotation(PathAnnotation&&) = default;

            PathAnnotation(PathPoints const& p)
            {
                data.resize(p.size());
                if(data.size() == 0)
                    return;
                data[0] = E{};
                for(agge::count_t i = 1; i < p.size(); ++i)
                    data[i] = E(p[i] - p[i-1]);
            }

            void clear() { data.clear(); }
            void push_back(E e) { data.push_back(e); }
            void push_back(Vector_r v) { data.push_back(E::from(v)); }

            std::vector<E> data;
        };
    }
}