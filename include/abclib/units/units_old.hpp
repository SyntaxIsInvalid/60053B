#pragma once
#include <cmath>
#include <cstdint>
namespace abclib::units
{

    // Forward declarations
    struct Radians;
    struct Degrees;
    struct Distance;

    // ============= ANGULAR POSITION =============
    struct Radians
    {
        double value;

        constexpr Radians() : value(0) {}
        constexpr explicit Radians(double v) : value(v) {}

        constexpr Radians operator+(Radians other) const { return Radians(value + other.value); }
        constexpr Radians operator-(Radians other) const { return Radians(value - other.value); }
        constexpr Radians operator-() const { return Radians(-value); }
        constexpr Radians operator*(double scalar) const { return Radians(value * scalar); }
        constexpr Radians operator/(double scalar) const { return Radians(value / scalar); }

        Degrees to_degrees() const;
    };

    struct Degrees
    {
        double value;

        constexpr Degrees() : value(0) {}
        constexpr explicit Degrees(double v) : value(v) {}

        constexpr Degrees operator+(Degrees other) const { return Degrees(value + other.value); }
        constexpr Degrees operator-(Degrees other) const { return Degrees(value - other.value); }
        constexpr Degrees operator-() const { return Degrees(-value); }
        constexpr Degrees operator*(double scalar) const { return Degrees(value * scalar); }
        constexpr Degrees operator/(double scalar) const { return Degrees(value / scalar); }

        constexpr Radians to_radians() const { return Radians(value * M_PI / 180.0); }
    };

    inline Degrees Radians::to_degrees() const { return Degrees(value * 180.0 / M_PI); }

    // ============= MOTOR UNITS =============
    struct MotorPosition
    {
        Radians angle; // Changed from Degrees

        constexpr MotorPosition() : angle(0) {}
        constexpr explicit MotorPosition(Radians a) : angle(a) {}
        // Maybe keep a convenience constructor from degrees?
        constexpr explicit MotorPosition(Degrees deg) : angle(deg.to_radians()) {}
    };

    struct MotorAngularVelocity
    {
        double rad_per_sec;

        constexpr MotorAngularVelocity() : rad_per_sec(0) {}
        constexpr explicit MotorAngularVelocity(double v) : rad_per_sec(v) {}

        // Convert from RPM (common motor unit)
        static constexpr MotorAngularVelocity from_rpm(double rpm)
        {
            return MotorAngularVelocity(rpm * 2.0 * M_PI / 60.0);
        }

        constexpr double to_rpm() const
        {
            return rad_per_sec * 60.0 / (2.0 * M_PI);
        }
    };

    // ============= WHEEL UNITS =============
    // ============= WHEEL UNITS =============
    struct WheelPosition
    {
        Degrees angle;

        constexpr WheelPosition() : angle(0) {}
        constexpr explicit WheelPosition(Degrees a) : angle(a) {}
        constexpr explicit WheelPosition(double degrees) : angle(degrees) {}
    };

    struct WheelAngularVelocity
    {
        double rad_per_sec;

        constexpr WheelAngularVelocity() : rad_per_sec(0) {}
        constexpr explicit WheelAngularVelocity(double v) : rad_per_sec(v) {}

        constexpr WheelAngularVelocity operator+(WheelAngularVelocity other) const
        {
            return WheelAngularVelocity(rad_per_sec + other.rad_per_sec);
        }
        constexpr WheelAngularVelocity operator-(WheelAngularVelocity other) const
        {
            return WheelAngularVelocity(rad_per_sec - other.rad_per_sec);
        }
        constexpr WheelAngularVelocity operator*(double scalar) const
        {
            return WheelAngularVelocity(rad_per_sec * scalar);
        }
        constexpr WheelAngularVelocity operator/(double scalar) const
        {
            return WheelAngularVelocity(rad_per_sec / scalar);
        }
    };

    // ADD THIS NEW TYPE HERE:
    struct WheelLinearVelocity
    {
        double inches_per_sec;

        constexpr WheelLinearVelocity() : inches_per_sec(0) {}
        constexpr explicit WheelLinearVelocity(double v) : inches_per_sec(v) {}

        constexpr WheelLinearVelocity operator+(WheelLinearVelocity other) const
        {
            return WheelLinearVelocity(inches_per_sec + other.inches_per_sec);
        }
        constexpr WheelLinearVelocity operator-(WheelLinearVelocity other) const
        {
            return WheelLinearVelocity(inches_per_sec - other.inches_per_sec);
        }
        constexpr WheelLinearVelocity operator*(double scalar) const
        {
            return WheelLinearVelocity(inches_per_sec * scalar);
        }
        constexpr WheelLinearVelocity operator/(double scalar) const
        {
            return WheelLinearVelocity(inches_per_sec / scalar);
        }

        // Just declare - define later after Distance is complete
        constexpr WheelAngularVelocity to_angular(Distance wheel_radius) const;
        static constexpr WheelLinearVelocity from_angular(
            WheelAngularVelocity angular,
            Distance wheel_radius);
    };

    // ============= BODY FRAME UNITS =============
    struct BodyLinearVelocity
    {
        double inches_per_sec;

        constexpr BodyLinearVelocity() : inches_per_sec(0) {}
        constexpr explicit BodyLinearVelocity(double v) : inches_per_sec(v) {}

        constexpr BodyLinearVelocity operator+(BodyLinearVelocity other) const
        {
            return BodyLinearVelocity(inches_per_sec + other.inches_per_sec);
        }
        constexpr BodyLinearVelocity operator-(BodyLinearVelocity other) const
        {
            return BodyLinearVelocity(inches_per_sec - other.inches_per_sec);
        }
        constexpr BodyLinearVelocity operator*(double scalar) const
        {
            return BodyLinearVelocity(inches_per_sec * scalar);
        }
        constexpr BodyLinearVelocity operator/(double scalar) const
        {
            return BodyLinearVelocity(inches_per_sec / scalar);
        }
    };

    struct BodyAngularVelocity
    {
        double rad_per_sec;

        constexpr BodyAngularVelocity() : rad_per_sec(0) {}
        constexpr explicit BodyAngularVelocity(double v) : rad_per_sec(v) {}

        constexpr BodyAngularVelocity operator+(BodyAngularVelocity other) const
        {
            return BodyAngularVelocity(rad_per_sec + other.rad_per_sec);
        }
        constexpr BodyAngularVelocity operator-(BodyAngularVelocity other) const
        {
            return BodyAngularVelocity(rad_per_sec - other.rad_per_sec);
        }
        constexpr BodyAngularVelocity operator*(double scalar) const
        {
            return BodyAngularVelocity(rad_per_sec * scalar);
        }
        constexpr BodyAngularVelocity operator/(double scalar) const
        {
            return BodyAngularVelocity(rad_per_sec / scalar);
        }
    };

    struct BodyHeading
    {
        Radians angle;

        constexpr BodyHeading() : angle(0) {}
        constexpr explicit BodyHeading(Radians a) : angle(a) {}
        constexpr explicit BodyHeading(Degrees deg) : angle(deg.to_radians()) {}
    };

    struct BodyPosition
    {
        double x_inches;
        double y_inches;

        constexpr BodyPosition() : x_inches(0), y_inches(0) {}
        constexpr BodyPosition(double x, double y) : x_inches(x), y_inches(y) {}

        constexpr BodyPosition operator+(BodyPosition other) const
        {
            return BodyPosition(x_inches + other.x_inches, y_inches + other.y_inches);
        }
        constexpr BodyPosition operator-(BodyPosition other) const
        {
            return BodyPosition(x_inches - other.x_inches, y_inches - other.y_inches);
        }

        double distance_to(BodyPosition other) const
        {
            double dx = x_inches - other.x_inches;
            double dy = y_inches - other.y_inches;
            return std::sqrt(dx * dx + dy * dy);
        }
    };

    struct Time
    {
        double seconds;

        constexpr Time() : seconds(0) {}
        constexpr explicit Time(double s) : seconds(s) {}

        constexpr Time operator+(Time other) const { return Time(seconds + other.seconds); }
        constexpr Time operator-(Time other) const { return Time(seconds - other.seconds); }
        constexpr Time operator*(double scalar) const { return Time(seconds * scalar); }
        constexpr Time operator/(double scalar) const { return Time(seconds / scalar); }

        constexpr bool operator<(Time other) const { return seconds < other.seconds; }
        constexpr bool operator>(Time other) const { return seconds > other.seconds; }
        constexpr bool operator<=(Time other) const { return seconds <= other.seconds; }
        constexpr bool operator>=(Time other) const { return seconds >= other.seconds; }

        static constexpr Time from_seconds(double s) { return Time(s); }
        static constexpr Time from_millis(double ms) { return Time(ms / 1000.0); }
        static constexpr Time from_millis(uint32_t ms) { return Time(ms / 1000.0); }

        constexpr double to_seconds() const { return seconds; }
        constexpr double to_millis() const { return seconds * 1000.0; }
        constexpr uint32_t to_millis_uint() const { return static_cast<uint32_t>(seconds * 1000.0); }
    };

    // ============= VOLTAGE =============
    struct Voltage
    {
        double volts;

        constexpr Voltage() : volts(0) {}
        constexpr explicit Voltage(double v) : volts(v) {}

        constexpr Voltage operator+(Voltage other) const { return Voltage(volts + other.volts); }
        constexpr Voltage operator-(Voltage other) const { return Voltage(volts - other.volts); }
        constexpr Voltage operator-() const { return Voltage(-volts); }
        constexpr Voltage operator*(double scalar) const { return Voltage(volts * scalar); }
        constexpr Voltage operator/(double scalar) const { return Voltage(volts / scalar); }

        constexpr bool operator<(Voltage other) const { return volts < other.volts; }
        constexpr bool operator>(Voltage other) const { return volts > other.volts; }

        // PROS motors use ±127 scale representing ±12V
        static constexpr Voltage from_pros_units(double pros_voltage)
        {
            return Voltage(pros_voltage * 12.0 / 127.0);
        }

        constexpr double to_pros_units() const
        {
            return volts * 127.0 / 12.0;
        }

        // Direct voltage construction
        static constexpr Voltage from_volts(double v) { return Voltage(v); }
        static constexpr Voltage from_millivolts(double mv) { return Voltage(mv / 1000.0); }
    };

    // ============= CURRENT =============
    struct Current
    {
        double amps;

        constexpr Current() : amps(0) {}
        constexpr explicit Current(double a) : amps(a) {}

        constexpr Current operator+(Current other) const { return Current(amps + other.amps); }
        constexpr Current operator-(Current other) const { return Current(amps - other.amps); }

        static constexpr Current from_amps(double a) { return Current(a); }
        static constexpr Current from_milliamps(double ma) { return Current(ma / 1000.0); }

        constexpr double to_amps() const { return amps; }
        constexpr double to_milliamps() const { return amps * 1000.0; }
    };

    struct RPM
    {
        double value;

        constexpr RPM() : value(0) {}
        constexpr explicit RPM(double v) : value(v) {}

        constexpr double to_rad_per_sec() const
        {
            return value * 2.0 * M_PI / 60.0;
        }

        static constexpr RPM from_rad_per_sec(double rad_s)
        {
            return RPM(rad_s * 60.0 / (2.0 * M_PI));
        }
    };

    // ============= LINEAR DISTANCE =============
    struct Distance
    {
        double inches;

        constexpr Distance() : inches(0) {}
        constexpr explicit Distance(double i) : inches(i) {}

        constexpr Distance operator+(Distance other) const
        {
            return Distance(inches + other.inches);
        }
        constexpr Distance operator-(Distance other) const
        {
            return Distance(inches - other.inches);
        }
        constexpr Distance operator-() const
        {
            return Distance(-inches);
        }
        constexpr Distance operator*(double scalar) const
        {
            return Distance(inches * scalar);
        }
        constexpr Distance operator/(double scalar) const
        {
            return Distance(inches / scalar);
        }

        constexpr bool operator<(Distance other) const { return inches < other.inches; }
        constexpr bool operator>(Distance other) const { return inches > other.inches; }
        constexpr bool operator<=(Distance other) const { return inches <= other.inches; }
        constexpr bool operator>=(Distance other) const { return inches >= other.inches; }

        static constexpr Distance from_inches(double i) { return Distance(i); }
        static constexpr Distance from_feet(double f) { return Distance(f * 12.0); }
        static constexpr Distance from_meters(double m) { return Distance(m * 39.3701); }

        constexpr double to_inches() const { return inches; }
        constexpr double to_feet() const { return inches / 12.0; }
        constexpr double to_meters() const { return inches / 39.3701; }
    };

    struct BodyPose
    {
        BodyPosition position;
        BodyHeading heading; // STORES RADIANS internally

        constexpr BodyPose() : position(), heading() {}

        // Primary constructor - radians
        constexpr BodyPose(BodyPosition pos, BodyHeading head)
            : position(pos), heading(head) {}

        // Convenience constructors
        constexpr BodyPose(double x, double y, Radians theta)
            : position(x, y), heading(theta) {}

        constexpr BodyPose(double x, double y, Degrees theta)
            : position(x, y), heading(theta) {} // Converts to radians via BodyHeading constructor

        // Factory methods for clarity
        static constexpr BodyPose from_radians(double x, double y, double theta_rad)
        {
            return BodyPose(x, y, Radians(theta_rad));
        }

        static constexpr BodyPose from_degrees(double x, double y, double theta_deg)
        {
            return BodyPose(x, y, Degrees(theta_deg));
        }

        // Accessors
        constexpr double x() const { return position.x_inches; }
        constexpr double y() const { return position.y_inches; }
        constexpr double theta() const { return heading.angle.value; } // Returns radians
        Degrees theta_deg() const { return heading.angle.to_degrees(); }
    };

    constexpr Voltage DEFAULT_MIN_VOLTAGE = Voltage::from_volts(0);
    constexpr Voltage DEFAULT_MAX_VOLTAGE = Voltage::from_volts(12);
    constexpr Voltage DEFAULT_MAX_ANGULAR_VOLTAGE = Voltage::from_volts(6);
    constexpr Time DEFAULT_TIMEOUT = Time::from_seconds(5);

    // ============= CONVENIENCE LITERALS =============
    namespace literals
    {
        constexpr Time operator""_s(long double v) { return Time::from_seconds(static_cast<double>(v)); }
        constexpr Time operator""_s(unsigned long long v) { return Time::from_seconds(static_cast<double>(v)); }
        constexpr Time operator""_ms(long double v) { return Time::from_millis(static_cast<double>(v)); }
        constexpr Time operator""_ms(unsigned long long v) { return Time::from_millis(static_cast<double>(v)); }

        constexpr Voltage operator""_V(long double v) { return Voltage::from_volts(static_cast<double>(v)); }
        constexpr Voltage operator""_V(unsigned long long v) { return Voltage::from_volts(static_cast<double>(v)); }

    }

    // ============= CONVERSION IMPLEMENTATIONS =============
    // Define these AFTER Distance is fully defined
    inline constexpr WheelAngularVelocity WheelLinearVelocity::to_angular(Distance wheel_radius) const
    {
        return WheelAngularVelocity(inches_per_sec / wheel_radius.inches);
    }

    inline constexpr WheelLinearVelocity WheelLinearVelocity::from_angular(
        WheelAngularVelocity angular,
        Distance wheel_radius)
    {
        return WheelLinearVelocity(angular.rad_per_sec * wheel_radius.inches);
    }

} // namespace abclib::units