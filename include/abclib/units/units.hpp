#pragma once
#include <ratio>
#include <cmath>
#include <type_traits>

namespace abclib::units
{
    // ============= CONVERSION CONSTANTS =============
    namespace constants
    {
        constexpr double PI = 3.141592653589793238462643383279502884;

        // Angular conversions
        constexpr double DEG_TO_RAD = PI / 180.0;
        constexpr double RAD_TO_DEG = 180.0 / PI;
        constexpr double REV_TO_RAD = 2.0 * PI;
        constexpr double RAD_TO_REV = 1.0 / (2.0 * PI);
        constexpr double RPM_TO_RAD_PER_SEC = 2.0 * PI / 60.0;
        constexpr double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * PI);

        // Length conversions (to meters)
        constexpr double INCH_TO_METER = 0.0254;
        constexpr double FOOT_TO_METER = 0.3048;
        constexpr double YARD_TO_METER = 0.9144;
        constexpr double CM_TO_METER = 0.01;
        constexpr double MM_TO_METER = 0.001;

        // Mass conversions (to kg)
        constexpr double GRAM_TO_KG = 0.001;
        constexpr double TONNE_TO_KG = 1000.0;
        constexpr double POUND_TO_KG = 0.45359237;
        constexpr double OUNCE_TO_KG = 0.028349523125;

        // Time conversions (to seconds)
        constexpr double MS_TO_SEC = 0.001;
        constexpr double MIN_TO_SEC = 60.0;
        constexpr double HOUR_TO_SEC = 3600.0;
        constexpr double DAY_TO_SEC = 86400.0;

        // Velocity conversions (to m/s)
        constexpr double IPS_TO_MPS = INCH_TO_METER;

        // Acceleration
        constexpr double G_TO_MPS2 = 9.80665;

        // Temperature
        constexpr double CELSIUS_TO_KELVIN_OFFSET = 273.15;
        constexpr double FAHRENHEIT_OFFSET = 32.0;
        constexpr double FAHRENHEIT_SCALE = 5.0 / 9.0;
        constexpr double KELVIN_TO_FAHRENHEIT_SCALE = 9.0 / 5.0;

        constexpr double KILO_UNIT = 1000.0; // For kilo- prefix
        constexpr double MILLI_UNIT = 0.001; // For milli- prefix

    }

    // ============= FORWARD DECLARATIONS =============
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    class Quantity;

    template <typename QuantityType>
    class absolute;

    // ============= CORE QUANTITY TEMPLATE =============
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    class Quantity
    {
    private:
        double value_; // Always stored in SI units

    public:
        constexpr Quantity() : value_(0.0) {}
        constexpr explicit Quantity(double val) : value_(val) {}

        // Accessors
        [[nodiscard]] constexpr double value() const { return value_; }

        // Addition/Subtraction (same dimensions only)
        constexpr Quantity operator+(const Quantity &rhs) const
        {
            return Quantity(value_ + rhs.value_);
        }

        constexpr Quantity operator-(const Quantity &rhs) const
        {
            return Quantity(value_ - rhs.value_);
        }

        constexpr Quantity operator-() const
        {
            return Quantity(-value_);
        }

        constexpr Quantity &operator+=(const Quantity &rhs)
        {
            value_ += rhs.value_;
            return *this;
        }

        constexpr Quantity &operator-=(const Quantity &rhs)
        {
            value_ -= rhs.value_;
            return *this;
        }

        // Comparison operators
        constexpr bool operator==(const Quantity &rhs) const
        {
            return value_ == rhs.value_;
        }

        constexpr bool operator!=(const Quantity &rhs) const
        {
            return value_ != rhs.value_;
        }

        constexpr bool operator<(const Quantity &rhs) const
        {
            return value_ < rhs.value_;
        }

        constexpr bool operator>(const Quantity &rhs) const
        {
            return value_ > rhs.value_;
        }

        constexpr bool operator<=(const Quantity &rhs) const
        {
            return value_ <= rhs.value_;
        }

        constexpr bool operator>=(const Quantity &rhs) const
        {
            return value_ >= rhs.value_;
        }
    };

    // ============= MULTIPLICATION (adds dimensions) =============
    template <typename M1, typename L1, typename T1, typename I1, typename Temp1, typename A1,
              typename M2, typename L2, typename T2, typename I2, typename Temp2, typename A2>
    constexpr auto operator*(
        const Quantity<M1, L1, T1, I1, Temp1, A1> &lhs,
        const Quantity<M2, L2, T2, I2, Temp2, A2> &rhs)
    {
        using ResultType = Quantity<
            std::ratio_add<M1, M2>,
            std::ratio_add<L1, L2>,
            std::ratio_add<T1, T2>,
            std::ratio_add<I1, I2>,
            std::ratio_add<Temp1, Temp2>,
            std::ratio_add<A1, A2>>;
        return ResultType(lhs.value() * rhs.value());
    }

    // Scalar multiplication
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr Quantity<M, L, T, I, Temp, A> operator*(
        double scalar,
        const Quantity<M, L, T, I, Temp, A> &q)
    {
        return Quantity<M, L, T, I, Temp, A>(scalar * q.value());
    }

    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr Quantity<M, L, T, I, Temp, A> operator*(
        const Quantity<M, L, T, I, Temp, A> &q,
        double scalar)
    {
        return Quantity<M, L, T, I, Temp, A>(q.value() * scalar);
    }

    // ============= DIVISION (subtracts dimensions) =============
    template <typename M1, typename L1, typename T1, typename I1, typename Temp1, typename A1,
              typename M2, typename L2, typename T2, typename I2, typename Temp2, typename A2>
    constexpr auto operator/(
        const Quantity<M1, L1, T1, I1, Temp1, A1> &lhs,
        const Quantity<M2, L2, T2, I2, Temp2, A2> &rhs)
    {
        using ResultType = Quantity<
            std::ratio_subtract<M1, M2>,
            std::ratio_subtract<L1, L2>,
            std::ratio_subtract<T1, T2>,
            std::ratio_subtract<I1, I2>,
            std::ratio_subtract<Temp1, Temp2>,
            std::ratio_subtract<A1, A2>>;
        return ResultType(lhs.value() / rhs.value());
    }

    // Scalar division
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr Quantity<M, L, T, I, Temp, A> operator/(
        const Quantity<M, L, T, I, Temp, A> &q,
        double scalar)
    {
        return Quantity<M, L, T, I, Temp, A>(q.value() / scalar);
    }

    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr auto operator/(
        double scalar,
        const Quantity<M, L, T, I, Temp, A> &q)
    {
        using ResultType = Quantity<
            std::ratio_subtract<std::ratio<0>, M>,
            std::ratio_subtract<std::ratio<0>, L>,
            std::ratio_subtract<std::ratio<0>, T>,
            std::ratio_subtract<std::ratio<0>, I>,
            std::ratio_subtract<std::ratio<0>, Temp>,
            std::ratio_subtract<std::ratio<0>, A>>;
        return ResultType(scalar / q.value());
    }

    // ============= TYPESAFE MATH FUNCTIONS =============

    // Square root (divides dimensions by 2)
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr auto Qsqrt(const Quantity<M, L, T, I, Temp, A> &q)
    {
        using ResultType = Quantity<
            std::ratio_divide<M, std::ratio<2>>,
            std::ratio_divide<L, std::ratio<2>>,
            std::ratio_divide<T, std::ratio<2>>,
            std::ratio_divide<I, std::ratio<2>>,
            std::ratio_divide<Temp, std::ratio<2>>,
            std::ratio_divide<A, std::ratio<2>>>;
        return ResultType(std::sqrt(q.value()));
    }

    // Square (multiplies dimensions by 2)
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr auto Qsq(const Quantity<M, L, T, I, Temp, A> &q)
    {
        return q * q;
    }

    // Absolute value (preserves dimensions)
    template <typename M, typename L, typename T, typename I, typename Temp, typename A>
    constexpr Quantity<M, L, T, I, Temp, A> Qabs(const Quantity<M, L, T, I, Temp, A> &q)
    {
        return Quantity<M, L, T, I, Temp, A>(std::fabs(q.value()));
    }

    // ============= BASE TYPE ALIASES =============
    // Format: Quantity<M, L, T, I, Temp, A>

    using Number = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<0>,
                            std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using Mass_Base = Quantity<std::ratio<1>, std::ratio<0>, std::ratio<0>,
                               std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using Length_Base = Quantity<std::ratio<0>, std::ratio<1>, std::ratio<0>,
                                 std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using Time_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>,
                               std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using Angle_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<0>,
                                std::ratio<0>, std::ratio<0>, std::ratio<1>>;

    using Velocity_Base = Quantity<std::ratio<0>, std::ratio<1>, std::ratio<-1>,
                                   std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using AngularVelocity_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<-1>,
                                          std::ratio<0>, std::ratio<0>, std::ratio<1>>;

    using Acceleration_Base = Quantity<std::ratio<0>, std::ratio<1>, std::ratio<-2>,
                                       std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using AngularAcceleration_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<-2>,
                                              std::ratio<0>, std::ratio<0>, std::ratio<1>>;

    using Jerk_Base = Quantity<std::ratio<0>, std::ratio<1>, std::ratio<-3>,
                               std::ratio<0>, std::ratio<0>, std::ratio<0>>;

    using AngularJerk_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<-3>,
                                      std::ratio<0>, std::ratio<0>, std::ratio<1>>;

    using Current_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<0>,
                                  std::ratio<1>, std::ratio<0>, std::ratio<0>>;

    using Voltage_Base = Quantity<std::ratio<1>, std::ratio<2>, std::ratio<-3>,
                                  std::ratio<-1>, std::ratio<0>, std::ratio<0>>;

    using Resistance_Base = Quantity<std::ratio<1>, std::ratio<2>, std::ratio<-3>,
                                     std::ratio<-2>, std::ratio<0>, std::ratio<0>>;

    using Charge_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<1>,
                                 std::ratio<1>, std::ratio<0>, std::ratio<0>>;

    using TemperatureDifference_Base = Quantity<std::ratio<0>, std::ratio<0>, std::ratio<0>,
                                                std::ratio<0>, std::ratio<1>, std::ratio<0>>;

    // ============= STRUCTS WITH FACTORY METHODS =============

    struct Mass : Mass_Base
    {
        using Mass_Base::Mass_Base;
        using Mass_Base::operator=;
        constexpr Mass(const Mass_Base &base) : Mass_Base(base) {}

        static constexpr Mass from_kg(double kg) { return Mass(kg); }
        static constexpr Mass from_grams(double g) { return Mass(g * constants::GRAM_TO_KG); }
        static constexpr Mass from_tonnes(double t) { return Mass(t * constants::TONNE_TO_KG); }
        static constexpr Mass from_pounds(double lb) { return Mass(lb * constants::POUND_TO_KG); }
        static constexpr Mass from_ounces(double oz) { return Mass(oz * constants::OUNCE_TO_KG); }

        constexpr double to_kg() const { return value(); }
        constexpr double to_grams() const { return value() / constants::GRAM_TO_KG; }
        constexpr double to_tonnes() const { return value() / constants::TONNE_TO_KG; }
        constexpr double to_pounds() const { return value() / constants::POUND_TO_KG; }
        constexpr double to_ounces() const { return value() / constants::OUNCE_TO_KG; }
    };

    struct Length : Length_Base
    {
        using Length_Base::Length_Base;
        using Length_Base::operator=;
        constexpr Length(const Length_Base &base) : Length_Base(base) {}

        static constexpr Length from_meters(double m) { return Length(m); }
        static constexpr Length from_inches(double in) { return Length(in * constants::INCH_TO_METER); }
        static constexpr Length from_feet(double ft) { return Length(ft * constants::FOOT_TO_METER); }
        static constexpr Length from_cm(double cm) { return Length(cm * constants::CM_TO_METER); }
        static constexpr Length from_mm(double mm) { return Length(mm * constants::MM_TO_METER); }
        static constexpr Length from_yards(double yd) { return Length(yd * constants::YARD_TO_METER); }

        constexpr double to_meters() const { return value(); }
        constexpr double to_inches() const { return value() / constants::INCH_TO_METER; }
        constexpr double to_feet() const { return value() / constants::FOOT_TO_METER; }
        constexpr double to_cm() const { return value() / constants::CM_TO_METER; }
        constexpr double to_mm() const { return value() / constants::MM_TO_METER; }
        constexpr double to_yards() const { return value() / constants::YARD_TO_METER; }
    };

    struct Time : Time_Base
    {
        using Time_Base::Time_Base;
        using Time_Base::operator=;
        constexpr Time(const Time_Base &base) : Time_Base(base) {}

        static constexpr Time from_seconds(double s) { return Time(s); }
        static constexpr Time from_milliseconds(double ms) { return Time(ms * constants::MS_TO_SEC); }
        static constexpr Time from_minutes(double min) { return Time(min * constants::MIN_TO_SEC); }
        static constexpr Time from_hours(double h) { return Time(h * constants::HOUR_TO_SEC); }
        static constexpr Time from_days(double d) { return Time(d * constants::DAY_TO_SEC); }

        constexpr double to_seconds() const { return value(); }
        constexpr double to_milliseconds() const { return value() / constants::MS_TO_SEC; }
        constexpr double to_minutes() const { return value() / constants::MIN_TO_SEC; }
        constexpr double to_hours() const { return value() / constants::HOUR_TO_SEC; }
        constexpr double to_days() const { return value() / constants::DAY_TO_SEC; }
    };

    struct Angle : Angle_Base
    {
        using Angle_Base::Angle_Base;
        using Angle_Base::operator=;
        constexpr Angle(const Angle_Base &base) : Angle_Base(base) {}

        static constexpr Angle from_radians(double rad) { return Angle(rad); }
        static constexpr Angle from_degrees(double deg) { return Angle(deg * constants::DEG_TO_RAD); }
        static constexpr Angle from_revolutions(double rev) { return Angle(rev * constants::REV_TO_RAD); }

        constexpr double to_radians() const { return value(); }
        constexpr double to_degrees() const { return value() * constants::RAD_TO_DEG; }
        constexpr double to_revolutions() const { return value() * constants::RAD_TO_REV; }
    };

    struct Velocity : Velocity_Base
    {
        using Velocity_Base::Velocity_Base;
        using Velocity_Base::operator=;
        constexpr Velocity(const Velocity_Base &base) : Velocity_Base(base) {}

        static constexpr Velocity from_mps(double mps) { return Velocity(mps); }
        static constexpr Velocity from_ips(double ips) { return Velocity(ips * constants::IPS_TO_MPS); }

        constexpr double to_mps() const { return value(); }
        constexpr double to_ips() const { return value() / constants::IPS_TO_MPS; }
    };

    struct AngularVelocity : AngularVelocity_Base
    {
        using AngularVelocity_Base::AngularVelocity_Base;
        using AngularVelocity_Base::operator=;
        constexpr AngularVelocity(const AngularVelocity_Base &base) : AngularVelocity_Base(base) {}

        static constexpr AngularVelocity from_rad_per_sec(double rad_s) { return AngularVelocity(rad_s); }
        static constexpr AngularVelocity from_deg_per_sec(double deg_s)
        {
            return AngularVelocity(deg_s * constants::DEG_TO_RAD);
        }
        static constexpr AngularVelocity from_rpm(double rpm)
        {
            return AngularVelocity(rpm * constants::RPM_TO_RAD_PER_SEC);
        }

        constexpr double to_rad_per_sec() const { return value(); }
        constexpr double to_deg_per_sec() const { return value() * constants::RAD_TO_DEG; }
        constexpr double to_rpm() const { return value() * constants::RAD_PER_SEC_TO_RPM; }
    };

    struct Acceleration : Acceleration_Base
    {
        using Acceleration_Base::Acceleration_Base;
        using Acceleration_Base::operator=;
        constexpr Acceleration(const Acceleration_Base &base) : Acceleration_Base(base) {}

        static constexpr Acceleration from_mps2(double mps2) { return Acceleration(mps2); }
        static constexpr Acceleration from_G(double g) { return Acceleration(g * constants::G_TO_MPS2); }

        constexpr double to_mps2() const { return value(); }
        constexpr double to_G() const { return value() / constants::G_TO_MPS2; }
    };

    struct AngularAcceleration : AngularAcceleration_Base
    {
        using AngularAcceleration_Base::AngularAcceleration_Base;
        using AngularAcceleration_Base::operator=;
        constexpr AngularAcceleration(const AngularAcceleration_Base &base) : AngularAcceleration_Base(base) {}

        static constexpr AngularAcceleration from_rad_per_sec2(double rad_s2)
        {
            return AngularAcceleration(rad_s2);
        }
        static constexpr AngularAcceleration from_deg_per_sec2(double deg_s2)
        {
            return AngularAcceleration(deg_s2 * constants::DEG_TO_RAD);
        }

        constexpr double to_rad_per_sec2() const { return value(); }
        constexpr double to_deg_per_sec2() const { return value() * constants::RAD_TO_DEG; }
    };

    struct Jerk : Jerk_Base
    {
        using Jerk_Base::Jerk_Base;
        using Jerk_Base::operator=;
        constexpr Jerk(const Jerk_Base &base) : Jerk_Base(base) {}

        static constexpr Jerk from_mps3(double mps3) { return Jerk(mps3); }

        constexpr double to_mps3() const { return value(); }
    };

    struct AngularJerk : AngularJerk_Base
    {
        using AngularJerk_Base::AngularJerk_Base;
        using AngularJerk_Base::operator=;
        constexpr AngularJerk(const AngularJerk_Base &base) : AngularJerk_Base(base) {}

        static constexpr AngularJerk from_rad_per_sec3(double rad_s3)
        {
            return AngularJerk(rad_s3);
        }

        static constexpr AngularJerk from_deg_per_sec3(double deg_s3)
        {
            return AngularJerk(deg_s3 * constants::DEG_TO_RAD);
        }

        constexpr double to_rad_per_sec3() const { return value(); }

        constexpr double to_deg_per_sec3() const { return value() * constants::RAD_TO_DEG; }
    };

    struct Current : Current_Base
    {
        using Current_Base::Current_Base;
        using Current_Base::operator=;
        constexpr Current(const Current_Base &base) : Current_Base(base) {}

        static constexpr Current from_amps(double a) { return Current(a); }
        static constexpr Current from_milliamps(double ma) { return Current(ma * constants::MILLI_UNIT); }

        constexpr double to_amps() const { return value(); }
        constexpr double to_milliamps() const { return value() / constants::MILLI_UNIT; }
    };

    struct Voltage : Voltage_Base
    {
        using Voltage_Base::Voltage_Base;
        using Voltage_Base::operator=;
        constexpr Voltage(const Voltage_Base &base) : Voltage_Base(base) {}

        static constexpr Voltage from_volts(double v) { return Voltage(v); }
        static constexpr Voltage from_millivolts(double mv) { return Voltage(mv * constants::MILLI_UNIT); }

        constexpr double to_volts() const { return value(); }
        constexpr double to_millivolts() const { return value() / constants::MILLI_UNIT; }
    };

    struct Resistance : Resistance_Base
    {
        using Resistance_Base::Resistance_Base;
        using Resistance_Base::operator=;
        constexpr Resistance(const Resistance_Base &base) : Resistance_Base(base) {}

        static constexpr Resistance from_ohms(double ohm) { return Resistance(ohm); }
        static constexpr Resistance from_milliohms(double mohm) { return Resistance(mohm * constants::MILLI_UNIT); }
        static constexpr Resistance from_kiloohms(double kohm) { return Resistance(kohm * constants::KILO_UNIT); }

        constexpr double to_ohms() const { return value(); }
        constexpr double to_milliohms() const { return value() / constants::MILLI_UNIT; }
        constexpr double to_kiloohms() const { return value() / constants::KILO_UNIT; }
    };

    struct Charge : Charge_Base
    {
        using Charge_Base::Charge_Base;
        using Charge_Base::operator=;
        constexpr Charge(const Charge_Base &base) : Charge_Base(base) {}

        static constexpr Charge from_coulombs(double c) { return Charge(c); }
        static constexpr Charge from_milliamp_hours(double mah)
        {
            return Charge(mah * 3.6); // mAh to Coulombs: mAh * 3600 / 1000 = mAh * 3.6
        }
        static constexpr Charge from_amp_hours(double ah)
        {
            return Charge(ah * constants::HOUR_TO_SEC); // Ah to Coulombs: Ah * 3600
        }

        constexpr double to_coulombs() const { return value(); }
        constexpr double to_milliamp_hours() const { return value() / 3.6; }
        constexpr double to_amp_hours() const { return value() / constants::HOUR_TO_SEC; }
    };

    struct TemperatureDifference : TemperatureDifference_Base
    {
        using TemperatureDifference_Base::TemperatureDifference_Base;
        using TemperatureDifference_Base::operator=;
        constexpr TemperatureDifference(const TemperatureDifference_Base &base) : TemperatureDifference_Base(base) {}

        static constexpr TemperatureDifference from_kelvin(double k) { return TemperatureDifference(k); }
        static constexpr TemperatureDifference from_celsius(double c) { return TemperatureDifference(c); } // Same for differences

        constexpr double to_kelvin() const { return value(); }
        constexpr double to_celsius() const { return value(); } // Same for differences
    };

    // ============= ADDITIONAL TYPE ALIASES =============

    // Aliases used in other parts of the codebase

    using Distance = Length;
    using Radians = Angle;
    using Degrees = Angle;
    using BodyHeading = Angle;
    using BodyAngularVelocity = AngularVelocity;
    using BodyLinearVelocity = Velocity;
    using WheelLinearVelocity = Velocity;

    // ============= AFFINE TEMPERATURE TYPE =============

    template <typename QuantityType>
    class absolute
    {
    private:
        QuantityType value_;

    public:
        constexpr absolute() : value_(0.0) {}
        constexpr explicit absolute(double val) : value_(val) {}
        constexpr explicit absolute(QuantityType val) : value_(val) {}

        // abs - abs = diff (the only operation that makes sense between two absolute temps)
        constexpr QuantityType operator-(const absolute &other) const
        {
            return QuantityType(value_.value() - other.value_.value());
        }

        // abs + diff = abs
        constexpr absolute operator+(const QuantityType &diff) const
        {
            return absolute(value_.value() + diff.value());
        }

        // abs - diff = abs
        constexpr absolute operator-(const QuantityType &diff) const
        {
            return absolute(value_.value() - diff.value());
        }

        // Comparison operators
        constexpr bool operator==(const absolute &rhs) const
        {
            return value_.value() == rhs.value_.value();
        }

        constexpr bool operator!=(const absolute &rhs) const
        {
            return value_.value() != rhs.value_.value();
        }

        constexpr bool operator<(const absolute &rhs) const
        {
            return value_.value() < rhs.value_.value();
        }

        constexpr bool operator>(const absolute &rhs) const
        {
            return value_.value() > rhs.value_.value();
        }

        constexpr bool operator<=(const absolute &rhs) const
        {
            return value_.value() <= rhs.value_.value();
        }

        constexpr bool operator>=(const absolute &rhs) const
        {
            return value_.value() >= rhs.value_.value();
        }

        [[nodiscard]] constexpr double value() const { return value_.value(); }
    };

    // DELETE these operations (can't add two absolute temperatures - that's nonsensical!)
    template <typename Q>
    void operator+(const absolute<Q> &, const absolute<Q> &) = delete;

    struct AbsoluteTemperature : absolute<TemperatureDifference_Base>
    {
        using absolute::absolute;

        static constexpr AbsoluteTemperature from_kelvin(double k)
        {
            return AbsoluteTemperature(k);
        }
        static constexpr AbsoluteTemperature from_celsius(double c)
        {
            return AbsoluteTemperature(c + constants::CELSIUS_TO_KELVIN_OFFSET);
        }
        static constexpr AbsoluteTemperature from_fahrenheit(double f)
        {
            return AbsoluteTemperature(
                (f - constants::FAHRENHEIT_OFFSET) * constants::FAHRENHEIT_SCALE + constants::CELSIUS_TO_KELVIN_OFFSET);
        }

        constexpr double to_kelvin() const { return value(); }
        constexpr double to_celsius() const { return value() - constants::CELSIUS_TO_KELVIN_OFFSET; }
        constexpr double to_fahrenheit() const
        {
            return (value() - constants::CELSIUS_TO_KELVIN_OFFSET) * constants::KELVIN_TO_FAHRENHEIT_SCALE + constants::FAHRENHEIT_OFFSET;
        }
    };

    // ============= TRIGONOMETRIC FUNCTIONS =============
    // These work on Angle but return dimensionless double (as they should)

    inline double sin(const Angle &angle)
    {
        return std::sin(angle.value());
    }

    inline double cos(const Angle &angle)
    {
        return std::cos(angle.value());
    }

    inline double tan(const Angle &angle)
    {
        return std::tan(angle.value());
    }

    inline Angle asin(double x)
    {
        return Angle::from_radians(std::asin(x));
    }

    inline Angle acos(double x)
    {
        return Angle::from_radians(std::acos(x));
    }

    inline Angle atan(double x)
    {
        return Angle::from_radians(std::atan(x));
    }

    inline Angle atan2(double y, double x)
    {
        return Angle::from_radians(std::atan2(y, x));
    }

    // ============= LITERALS =============
    namespace literals
    {
        // Time literals
        constexpr Time operator""_s(long double x)
        {
            return Time::from_seconds(static_cast<double>(x));
        }

        constexpr Time operator""_s(unsigned long long x)
        {
            return Time::from_seconds(static_cast<double>(x));
        }

        constexpr Time operator""_ms(long double x)
        {
            return Time::from_milliseconds(static_cast<double>(x));
        }

        constexpr Time operator""_ms(unsigned long long x)
        {
            return Time::from_milliseconds(static_cast<double>(x));
        }

        constexpr Time operator""_min(long double x)
        {
            return Time::from_minutes(static_cast<double>(x));
        }

        constexpr Time operator""_min(unsigned long long x)
        {
            return Time::from_minutes(static_cast<double>(x));
        }

        constexpr Time operator""_h(long double x)
        {
            return Time::from_hours(static_cast<double>(x));
        }

        constexpr Time operator""_h(unsigned long long x)
        {
            return Time::from_hours(static_cast<double>(x));
        }

        // Length literals
        constexpr Length operator""_m(long double x)
        {
            return Length::from_meters(static_cast<double>(x));
        }

        constexpr Length operator""_m(unsigned long long x)
        {
            return Length::from_meters(static_cast<double>(x));
        }

        constexpr Length operator""_in(long double x)
        {
            return Length::from_inches(static_cast<double>(x));
        }

        constexpr Length operator""_in(unsigned long long x)
        {
            return Length::from_inches(static_cast<double>(x));
        }

        constexpr Length operator""_ft(long double x)
        {
            return Length::from_feet(static_cast<double>(x));
        }

        constexpr Length operator""_ft(unsigned long long x)
        {
            return Length::from_feet(static_cast<double>(x));
        }

        constexpr Length operator""_cm(long double x)
        {
            return Length::from_cm(static_cast<double>(x));
        }

        constexpr Length operator""_cm(unsigned long long x)
        {
            return Length::from_cm(static_cast<double>(x));
        }

        constexpr Length operator""_mm(long double x)
        {
            return Length::from_mm(static_cast<double>(x));
        }

        constexpr Length operator""_mm(unsigned long long x)
        {
            return Length::from_mm(static_cast<double>(x));
        }

        constexpr Length operator""_yd(long double x)
        {
            return Length::from_yards(static_cast<double>(x));
        }

        constexpr Length operator""_yd(unsigned long long x)
        {
            return Length::from_yards(static_cast<double>(x));
        }

        // Angle literals
        constexpr Angle operator""_rad(long double x)
        {
            return Angle::from_radians(static_cast<double>(x));
        }

        constexpr Angle operator""_rad(unsigned long long x)
        {
            return Angle::from_radians(static_cast<double>(x));
        }

        constexpr Angle operator""_deg(long double x)
        {
            return Angle::from_degrees(static_cast<double>(x));
        }

        constexpr Angle operator""_deg(unsigned long long x)
        {
            return Angle::from_degrees(static_cast<double>(x));
        }

        // Velocity literals
        constexpr Velocity operator""_mps(long double x)
        {
            return Velocity::from_mps(static_cast<double>(x));
        }

        constexpr Velocity operator""_mps(unsigned long long x)
        {
            return Velocity::from_mps(static_cast<double>(x));
        }

        constexpr Velocity operator""_ips(long double x)
        {
            return Velocity::from_ips(static_cast<double>(x));
        }

        constexpr Velocity operator""_ips(unsigned long long x)
        {
            return Velocity::from_ips(static_cast<double>(x));
        }

        // Mass literals
        constexpr Mass operator""_kg(long double x)
        {
            return Mass::from_kg(static_cast<double>(x));
        }

        constexpr Mass operator""_kg(unsigned long long x)
        {
            return Mass::from_kg(static_cast<double>(x));
        }

        constexpr Mass operator""_g(long double x)
        {
            return Mass::from_grams(static_cast<double>(x));
        }

        constexpr Mass operator""_g(unsigned long long x)
        {
            return Mass::from_grams(static_cast<double>(x));
        }

        constexpr Mass operator""_lb(long double x)
        {
            return Mass::from_pounds(static_cast<double>(x));
        }

        constexpr Mass operator""_lb(unsigned long long x)
        {
            return Mass::from_pounds(static_cast<double>(x));
        }

        // Voltage literals
        constexpr Voltage operator""_V(long double x)
        {
            return Voltage::from_volts(static_cast<double>(x));
        }

        constexpr Voltage operator""_V(unsigned long long x)
        {
            return Voltage::from_volts(static_cast<double>(x));
        }

        // Current literals
        constexpr Current operator""_A(long double x)
        {
            return Current::from_amps(static_cast<double>(x));
        }

        constexpr Current operator""_A(unsigned long long x)
        {
            return Current::from_amps(static_cast<double>(x));
        }

        // Resistance literals
        constexpr Resistance operator""_Ohm(long double x)
        {
            return Resistance::from_ohms(static_cast<double>(x));
        }

        constexpr Resistance operator""_Ohm(unsigned long long x)
        {
            return Resistance::from_ohms(static_cast<double>(x));
        }

        // Acceleration literals
        constexpr Acceleration operator""_mps2(long double x)
        {
            return Acceleration::from_mps2(static_cast<double>(x));
        }

        constexpr Acceleration operator""_mps2(unsigned long long x)
        {
            return Acceleration::from_mps2(static_cast<double>(x));
        }

        constexpr Acceleration operator""_ips2(long double x)
        {
            return Acceleration::from_mps2(static_cast<double>(x) * constants::INCH_TO_METER);
        }

        constexpr Acceleration operator""_ips2(unsigned long long x)
        {
            return Acceleration::from_mps2(static_cast<double>(x) * constants::INCH_TO_METER);
        }

        constexpr Acceleration operator""_G(long double x)
        {
            return Acceleration::from_G(static_cast<double>(x));
        }

        constexpr Acceleration operator""_G(unsigned long long x)
        {
            return Acceleration::from_G(static_cast<double>(x));
        }

        // Angular velocity literals
        constexpr AngularVelocity operator""_rad_per_sec(long double x)
        {
            return AngularVelocity::from_rad_per_sec(static_cast<double>(x));
        }

        constexpr AngularVelocity operator""_rad_per_sec(unsigned long long x)
        {
            return AngularVelocity::from_rad_per_sec(static_cast<double>(x));
        }

        constexpr AngularVelocity operator""_deg_per_sec(long double x)
        {
            return AngularVelocity::from_deg_per_sec(static_cast<double>(x));
        }

        constexpr AngularVelocity operator""_deg_per_sec(unsigned long long x)
        {
            return AngularVelocity::from_deg_per_sec(static_cast<double>(x));
        }

        constexpr AngularVelocity operator""_rpm(long double x)
        {
            return AngularVelocity::from_rpm(static_cast<double>(x));
        }

        constexpr AngularVelocity operator""_rpm(unsigned long long x)
        {
            return AngularVelocity::from_rpm(static_cast<double>(x));
        }

        // Angular acceleration literals
        constexpr AngularAcceleration operator""_rad_per_sec2(long double x)
        {
            return AngularAcceleration::from_rad_per_sec2(static_cast<double>(x));
        }

        constexpr AngularAcceleration operator""_rad_per_sec2(unsigned long long x)
        {
            return AngularAcceleration::from_rad_per_sec2(static_cast<double>(x));
        }

        constexpr AngularAcceleration operator""_deg_per_sec2(long double x)
        {
            return AngularAcceleration::from_deg_per_sec2(static_cast<double>(x));
        }

        constexpr AngularAcceleration operator""_deg_per_sec2(unsigned long long x)
        {
            return AngularAcceleration::from_deg_per_sec2(static_cast<double>(x));
        }

        // Jerk literals
        constexpr Jerk operator""_mps3(long double x)
        {
            return Jerk::from_mps3(static_cast<double>(x));
        }

        constexpr Jerk operator""_mps3(unsigned long long x)
        {
            return Jerk::from_mps3(static_cast<double>(x));
        }

        constexpr Jerk operator""_ips3(long double x)
        {
            return Jerk::from_mps3(static_cast<double>(x) * constants::INCH_TO_METER);
        }

        constexpr Jerk operator""_ips3(unsigned long long x)
        {
            return Jerk::from_mps3(static_cast<double>(x) * constants::INCH_TO_METER);
        }

        // Angular jerk literals
        constexpr AngularJerk operator""_rad_per_sec3(long double x)
        {
            return AngularJerk::from_rad_per_sec3(static_cast<double>(x));
        }

        constexpr AngularJerk operator""_rad_per_sec3(unsigned long long x)
        {
            return AngularJerk::from_rad_per_sec3(static_cast<double>(x));
        }

        constexpr AngularJerk operator""_deg_per_sec3(long double x)
        {
            return AngularJerk::from_rad_per_sec3(static_cast<double>(x) * constants::DEG_TO_RAD);
        }

        constexpr AngularJerk operator""_deg_per_sec3(unsigned long long x)
        {
            return AngularJerk::from_rad_per_sec3(static_cast<double>(x) * constants::DEG_TO_RAD);
        }

        // Temperature difference literals
        constexpr TemperatureDifference operator""_degC(long double x)
        {
            return TemperatureDifference::from_celsius(static_cast<double>(x));
        }

        constexpr TemperatureDifference operator""_degC(unsigned long long x)
        {
            return TemperatureDifference::from_celsius(static_cast<double>(x));
        }

        constexpr TemperatureDifference operator""_degK(long double x)
        {
            return TemperatureDifference::from_kelvin(static_cast<double>(x));
        }

        constexpr TemperatureDifference operator""_degK(unsigned long long x)
        {
            return TemperatureDifference::from_kelvin(static_cast<double>(x));
        }

        // Note: Fahrenheit differences use the same scale factor as Celsius (1°F change = 5/9 K change)
        constexpr TemperatureDifference operator""_degF(long double x)
        {
            return TemperatureDifference::from_kelvin(static_cast<double>(x) * constants::FAHRENHEIT_SCALE);
        }

        constexpr TemperatureDifference operator""_degF(unsigned long long x)
        {
            return TemperatureDifference::from_kelvin(static_cast<double>(x) * constants::FAHRENHEIT_SCALE);
        }

    } // namespace literals

} // namespace abclib::units