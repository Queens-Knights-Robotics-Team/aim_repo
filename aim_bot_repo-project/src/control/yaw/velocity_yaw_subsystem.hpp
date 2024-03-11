#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/setpoint/interfaces/integrable_setpoint_subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "control/algorithms/edu_pid.hpp"

class Drivers;

namespace control::yaw
{
/**
 * Subsystem copies ARUW's Agitator program
 * 
 * Subsystem whose primary purpose is to encapsulate an agitator motor that operates using a
 * velocity controller. Also keeps track of absolute position to allow commands to rotate the
 * agitator some specific displacement.
 */
class VelocityYawSubsystem : public tap::control::setpoint::IntegrableSetpointSubsystem
{
public:
    /**
     * Agitator gear ratios of different motors, for determining shaft rotation angle.
     */
    static constexpr float AGITATOR_GEAR_RATIO_M2006 = 36.0f;

    /**
     * Construct an agitator with the passed in velocity PID parameters, gear ratio, and
     * agitator-specific configuration.
     *
     * @param[in] drivers Reference to aruwsrc drivers struct
     * @param[in] pidConfig PID configuration struct for the agitator motor controller.
     * @param[in] yaw The base motor that this yaw subsystem is is going to control.
     */
    VelocityYawSubsystem(
        Drivers& drivers,
        const control::algorithms::EduPidConfig& pidConfig,
        tap::motor::DjiMotor& yaw);

    void initialize() override;

    /**
     * @brief Checks if the yaw motor is connected and runs the velocity PID controller.
     */
    void refresh() override;

    const char* getName() override { return "velocity yaw"; }

    /// @return The velocity setpoint that some command has requested, in radians / second
    float getSetpoint() const override;

    /**
     * Sets the velocity setpoint to the specified velocity
     *
     * @param[in] velocity The desired velocity in radians / second.
     */
    void setSetpoint(float velocity) override { velocitySetpoint = velocity; }

    /// @return The agitator velocity in radians / second.
    float getCurrentValue() const override;

    /**
     * Meaningless function that nothing uses
     * @return 0
     */
    inline float getJamSetpointTolerance() const override { return 0; }

    /**
     * Attempts to calibrate the agitator at the current position, such that `getPosition` will
     * return 0 radians at this position.
     *
     * @return `true` if the agitator has been successfully calibrated, `false` otherwise.
     */
    bool calibrateHere() override;

    /**
     * @brief Because this agitator doesn't support jam detection and unjamming, this function
     * always returns false.
     */
    bool isJammed() override { return false; }

    /**
     * Because this agitator doesn't support unjamming, this function does nothing.
     */
    void clearJam() override {}

    /**
     * @return `true` if the agitator has been calibrated (`calibrateHere` has been called and the
     * agitator motor is online).
     */
    bool isCalibrated() override { return calibrated; }

    /**
     * @return `true` if the agitator motor is online (i.e.: is connected)
     */
    bool isOnline() override;

    /**
     * Since we don't keep track of the derivative of the velocity (since the velocity is the
     * setpoint), this function will always return 0.
     *
     * @return 0
     */
    inline float getVelocity() override { return 0; }

    /**
     * @return The calibrated agitator angle, in radians (integral of velocity is radians). If the
     * agitator is uncalibrated, 0 radians is returned.
     */
    float getCurrentValueIntegral() const override;

private:
    /// @return Uncalibrated agitator angle.
    float getUncalibratedYawAngle() const;

    tap::motor::DjiMotor& yaw;

    control::algorithms::EduPid velocityPid;

    float velocitySetpoint{0};

    bool calibrated{false};

    float yawCalibratedZeroAngle{0};

    uint32_t prevTime{0};
};

}  // namespace control::yaw