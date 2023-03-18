#pragma once

#include <grace/algebra/algebra.h>
#include <grace/algebra/adapted.h>
#include <grace/algebra/control.h>

#include <grace/algorithms/annotations.h>
#include <optional>
#include <vector>

namespace grace
{
    namespace elements
    {
        template<typename A = annotations::United_DDL>
        struct Keeper: rules::Sink<Keeper<A>>
        {
            bool consume(Point_r const& p)
            {
                path.push_back(p);
                if(path.size() < 2)
                    return true;
                if(annot.empty())
                    annot.emplace_back(path.back() - path[path.size()-2]);
                else
                    annot.emplace_back(path.back() - path[path.size()-2], annot.back());
                if(annot.back().distance < length_limit)
                    return length_limit -= annot.back().distance, true;
                return false;
            }

            void clear()
            {
                path.clear();
                annot.clear();
            }

            std::vector<Point_r>    path;
            std::vector<A>          annot;
            real_t                  length_limit;
        };

        template<>
        struct Keeper<void>: rules::Sink<Keeper<void>>
        {
            bool consume(Point_r const& p)
            {
                path.push_back(p);
                if(path.size() < 2)
                    return true;
                real_t const distance = norm(path.back() - path[path.size()-2]);
                if(distance < length_limit)
                    return length_limit -= distance, true;
                return false;
            }

            void clear()
            {
                path.clear();
            }

            std::vector<Point_r>    path;
            real_t                  length_limit;
        };

        struct Skipper: rules::Sink<Skipper>
        {
            bool start(Point_r const& p1, Point_r const& p)
            {
                prev = p1;
                last = p;
                real_t const dist = norm(p - prev);
                if(dist < length_limit)
                    return length_limit -= dist, true;
                return false;
            }

            bool consume(Point_r const& p)
            {
                prev = last;
                last = p;
                real_t const dist = norm(p - prev);
                if(dist < length_limit)
                    return length_limit -= dist, true;
                return false;
            }

            Point_r   prev;
            Point_r   last;
            real_t    length_limit;
        };
    }
}