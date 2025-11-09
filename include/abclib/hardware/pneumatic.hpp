#pragma once

#include "api.h"

namespace abclib::hardware
{
    class Pneumatic
    {
    private:
        pros::adi::DigitalOut piston;
        bool is_extended;

    public:
        Pneumatic(char port);

        virtual void extend();
        virtual void retract();
        virtual void toggle();
        virtual bool get_state() const;
        virtual ~Pneumatic() = default;
    };

} // namespace abclib::hardware