#include "standard.hpp"

#include "tap/util_macros.hpp"

#include "control/turret/turret_subsystem.hpp"
#include "control/turret/turret_gimbal_command.hpp"

#include "drivers.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

namespace control
{
Robot::Robot(Drivers &drivers)
    :drivers(drivers),
      // construct TurretSubsystem and TurretGimbalCommand
      turret(
          drivers,
          turret::TurretConfig{
            // left side motors
              .pitchId = MotorId::MOTOR5,
              .yawId = MotorId::MOTOR7,
              .canBus = CanBus::CAN_BUS1,
              .velocityPidConfig = modm::Pid<float>::Parameter(10, 0, 0, 0, 16'000),
          }),
      turretGimbal(turret, drivers.controlOperatorInterface),
      // construct VelocityYawSubsystem and MoveIntegralCommand
      yawMotor(&drivers, MotorId::MOTOR8, CanBus::CAN_BUS1, false, "yaw motor"),
      yawSubsystem(
          drivers,
          {
              .kp = 50'000,
              .ki = 0,
              .kd = 0,
              .maxICumulative = 0,
              .maxOutput = 16'000,
          },
          yawMotor),
      rotateYaw(
          yawSubsystem,
          {
              .targetIntegralChange = M_TWOPI / 10.0f,
              .desiredSetpoint = M_TWOPI,
              .integralSetpointTolerance = 0,
          }),
        // construct HoldRepeatCommandMapping and HoldCommandMapping
        rightSwitchUp(
            &drivers,
            {&rotateYaw},
            RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)
            )

{
}

void Robot::initSubsystemCommands()
{
    initializeSubsystems();
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}

void Robot::initializeSubsystems()
{
    // initialize declared ChassisSubsystem
    turret.initialize();
    // YAW initialize declared VelocityYawSubsystem
    yawSubsystem.initialize();
    
}

void Robot::registerSoldierSubsystems()
{
    // register declared ChassisSubsystem
    drivers.commandScheduler.registerSubsystem(&turret);
    // r YAW egister declared VelocityYawSubsystem
    drivers.commandScheduler.registerSubsystem(&yawSubsystem);
}

void Robot::setDefaultSoldierCommands()
{
    // set TurretGimbalCommand as default command for TurretSubsystem
    turret.setDefaultCommand(&turretGimbal);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
    // YAW register HoldRepeatCommandMapping and HoldCommandMapping
    drivers.commandMapper.addMap(&rightSwitchUp);
}
}  // namespace control