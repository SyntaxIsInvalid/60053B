#include "abclib/hardware/pneumatic.hpp"

namespace abclib::hardware
{
    Pneumatic::Pneumatic(char port)
        : piston(port), is_extended(false)
    {
    }

    void Pneumatic::extend()
    {
        piston.set_value(true);
        is_extended = true;
    }

    void Pneumatic::retract()
    {
        piston.set_value(false);
        is_extended = false;
    }

    void Pneumatic::toggle()
    {
        if (is_extended)
        {
            retract();
        }
        else
        {
            extend();
        }
    }

    bool Pneumatic::get_state() const
    {
        return is_extended;
    }

} // namespace abclib::hardware