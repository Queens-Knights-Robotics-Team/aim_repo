#include "turret_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

using tap::algorithms::limitVal;

namespace control::turret
{
// create constructor
TurretSubsystem::TurretSubsystem(Drivers &drivers, const TurretConfig &config)
    : tap::control::Subsystem(&drivers),
      desiredOutput{},
      pidControllers{},
      positionPidControllers{}, // position control
      motors{
          Motor(&drivers, config.pitchId, config.canBus, false, "PITCH"),
          Motor(&drivers, config.yawId, config.canBus, false, "YAW"),
      }
{
    for (auto &controller : pidControllers)
    {
        controller.setParameter(config.velocityPidConfig);
    }
    // initialize new PID controller for position control using the provided configuration parameters
    for (auto &positionController : positionPidControllers)
    {
        positionController.setParameter(config.positionPidConfig);
    }
}

// Initialize function
void TurretSubsystem::initialize()
{
    for (auto &motor : motors)
    {
        motor.initialize();
    }
}

// setVelocityGimbal function
void TurretSubsystem::setVelocityGimbal(float pitch, float yaw)
{
    pitch = mpsToRpm(pitch);
    yaw = mpsToRpm(yaw);

    pitch = limitVal(pitch, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    yaw = limitVal(yaw, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitch;
    desiredOutput[static_cast<uint8_t>(MotorId::YAW)] = yaw;
}

// setPositionGimbal function
void TurretSubsystem::setPositionGimbal(float pitch, float yaw)
{
    // Convert desired positions to RPM
    pitch = mpsToRpm(pitch);
    yaw = mpsToRpm(yaw);

    // Limit desired positions
    pitch = limitVal(pitch, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    yaw = limitVal(yaw, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    // Set desired positions
    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitch;
    desiredOutput[static_cast<uint8_t>(MotorId::YAW)] = yaw;
}

// runPositionPid function
void TurretSubsystem::runPositionPid(Pid &pid, Motor &motor, float desiredPosition)
{
    pid.update(desiredPosition - motor.getCurrentPosition());  // Adjust this line based on the actual method to get the current position
    motor.setDesiredOutput(pid.getValue());
}

// refresh function
void TurretSubsystem::refresh()
{
    auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
        pid.update(desiredOutput - motor.getShaftRPM());
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        // run the position PID controllers
        runPositionPid(positionPidControllers[ii], motors[ii], desiredOutput[ii]);
        // run the velocity PID controllers
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }
}
}  // namespace control::turret