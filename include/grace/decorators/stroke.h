#pragma once
#include <grace/decorators/caps.h>
#include <grace/decorators/joins.h>

#include <functional>

namespace grace
{
    namespace decorators
    {
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
            std::function<caps::cap_func_t>   cap_  = caps::butt;
            std::function<joins::join_func_t> join_ = joins::miter;
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
            std::function<caps::cap_func_t>   head_      = caps::butt;
            std::function<caps::cap_func_t>   tail_      = caps::butt;
            std::function<joins::join_func_t> left_      = joins::miter;
            std::function<joins::join_func_t> right_     = joins::miter;
        };
    }
}