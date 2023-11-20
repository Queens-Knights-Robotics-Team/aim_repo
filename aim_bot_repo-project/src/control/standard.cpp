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
    : drivers(drivers),
      // construct TurretSubsystem and TurretGimbalCommand
      turret(
          drivers,
          turret::TurretConfig{
              .pitchId = MotorId::MOTOR2,
              .yawId = MotorId::MOTOR3,
              .canBus = CanBus::CAN_BUS1,
              .velocityPidConfig = modm::Pid<float>::Parameter(10, 0, 0, 0, 16'000),
          }),
      turretGimbal(turret, drivers.controlOperatorInterface)
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
}

void Robot::registerSoldierSubsystems()
{
    // register declared ChassisSubsystem
    drivers.commandScheduler.registerSubsystem(&turret);
}

void Robot::setDefaultSoldierCommands()
{
    // set TurretGimbalCommand as default command for TurretSubsystem
    turret.setDefaultCommand(&turretGimbal);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
}
}  // namespace control