#pragma once

#include "abclib/estimation/odometry.hpp"
#include "abclib/estimation/pose.hpp"
namespace abclib::estimation
{
    class IStateEstimator
    {
    public:
        virtual ~IStateEstimator() = default;
        virtual void update() = 0;
        virtual Pose get_pose() const = 0;
        virtual void set_pose(const Pose& pose) = 0;
        virtual void reset() = 0;
        virtual void calibrate() = 0;
        virtual void init() = 0;
        virtual void stop() = 0;
    };
}