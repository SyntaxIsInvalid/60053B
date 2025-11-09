#pragma once

#include "pneumatic.hpp"

namespace abclib::hardware
{
    class DummyPneumatic : public Pneumatic
    {
    public:
        // Use a dummy port that won't be used
        DummyPneumatic() : Pneumatic('A') {}

        // Override all methods to do nothing
        void extend() override {}
        void retract() override {}
        void toggle() override {}
        bool get_state() const override { return false; }
    };

} // namespace abclib::hardware