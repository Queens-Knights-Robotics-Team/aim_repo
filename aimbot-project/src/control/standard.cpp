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
              .pitchId = MotorId::MOTOR8,
              .yawId = MotorId::MOTOR3,
              .canBus = CanBus::CAN_BUS1,
              .velocityPidConfig = modm::Pid<float>::Parameter(10, 0, 0, 0, 16'000),
          }),
      turretGimbal(turret, drivers.controlOperatorInterface),
      // STEP 3 (Agitator Control): construct VelocityAgitatorSubsystem and MoveIntegralCommand
      agitatorMotor(&drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "agitator motor"),
      agitatorSubsystem(
          drivers,
          {
              .kp = 50'000,
              .ki = 0,
              .kd = 0,
              .maxICumulative = 0,
              .maxOutput = 16'000,
          },
          agitatorMotor),
      rotateAgitator(
          agitatorSubsystem,
          {
              .targetIntegralChange = M_TWOPI / 10.0f,
              .desiredSetpoint = M_TWOPI,
              .integralSetpointTolerance = 0,
          }),
      // STEP 8 (Agitator Control): construct HoldRepeatCommandMapping and HoldCommandMapping
      rightSwitchUp(
          &drivers,
          {&rotateAgitator},
          RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
          true),
      leftMousePressed(
          &drivers,
          {&rotateAgitator},
          RemoteMapState(RemoteMapState::MouseButton::LEFT))
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
    // STEP 4 (Tank Drive): initialize declared turret
    turret.initialize();
    // STEP 4 (Agitator Control): initialize declared VelocityAgitatorSubsystem
    agitatorSubsystem.initialize();

    
}

void Robot::registerSoldierSubsystems()
{
    // STEP 5 (Tank Drive): register declared TurretSubsystem
    // register declared TurretSubsystem
    drivers.commandScheduler.registerSubsystem(&turret);
    // STEP 5 (Agitator Control): register declared VelocityAgitatorSubsystem
    drivers.commandScheduler.registerSubsystem(&agitatorSubsystem);

}

void Robot::setDefaultSoldierCommands()
{
    // STEP 6 (Tank Drive): set TurretTanKDriveCommand as default command for TurretSubsystem
    // set TurretGimbalCommand as default command for TurretSubsystem
    turret.setDefaultCommand(&turretGimbal);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
    // STEP 9 (Agitator Control): register HoldRepeatCommandMapping and HoldCommandMapping
    drivers.commandMapper.addMap(&rightSwitchUp);
    drivers.commandMapper.addMap(&leftMousePressed);

}
}  // namespace control