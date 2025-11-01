#pragma once

#include "api.h"

namespace abclib::hardware
{
    class Pneumatic
    {
    private:
        pros::ADIDigitalOut piston;
        bool is_extended;

    public:
        Pneumatic(char port);

        void extend();
        void retract();
        void toggle();
        bool get_state() const;
    };

} // namespace abclib::hardware