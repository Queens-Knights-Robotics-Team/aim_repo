#include "velocity_yaw_subsystem.hpp"

#include "tap/architecture/clock.hpp"

#include "modm/math/geometry/angle.hpp"

#include "drivers.hpp"

using tap::arch::clock::getTimeMilliseconds;
using tap::motor::DjiMotor;

namespace control::yaw
{
// create constructor
VelocityYawSubsystem::VelocityYawSubsystem(
    Drivers& drivers,
    const control::algorithms::EduPidConfig& pidConfig,
    tap::motor::DjiMotor& yaw)
    : tap::control::Subsystem(&drivers),
      yaw(yaw),
      velocityPid(pidConfig)
{
}

// initialize function
void VelocityYawSubsystem::initialize() { yaw.initialize(); }

// refresh function
void VelocityYawSubsystem::refresh()
{
    if (!isOnline())
    {
        calibrated = false;
    }

    if (isCalibrated())
    {
        const uint32_t curTime = getTimeMilliseconds();
        const uint32_t dt = curTime - prevTime;
        prevTime = curTime;

        const float error = velocitySetpoint - getCurrentValue();

        velocityPid.runControllerDerivateError(error, dt);
    }
    else
    {
        calibrateHere();
    }
}

// getSetpoint function
float VelocityYawSubsystem::getSetpoint() const { return velocitySetpoint; }

// getCurrentValue function
float VelocityYawSubsystem::getCurrentValue() const
{
    return (yaw.getShaftRPM() / AGITATOR_GEAR_RATIO_M2006) * (M_TWOPI / 60.0f);
}

// calibrateHere function
bool VelocityYawSubsystem::calibrateHere()
{
    if (!isOnline())
    {
        return false;
    }
    yawCalibratedZeroAngle = getUncalibratedYawAngle();
    calibrated = true;
    velocitySetpoint = 0.0f;
    return true;
}

// isOnline function
bool VelocityYawSubsystem::isOnline() { return yaw.isMotorOnline(); }

// getCurrentValueIntegral function
float VelocityYawSubsystem::getCurrentValueIntegral() const
{
    return getUncalibratedYawAngle() - yawCalibratedZeroAngle;
}

float VelocityYawSubsystem::getUncalibratedYawAngle() const
{
    return (2.0f * M_PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           yaw.getEncoderUnwrapped() / AGITATOR_GEAR_RATIO_M2006;
}
}  // namespace control::yaw
