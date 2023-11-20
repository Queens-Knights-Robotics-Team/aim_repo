#pragma once

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"

#include "control/turret/turret_subsystem.hpp"
#include "control/turret/turret_gimbal_command.hpp"

class Drivers;

namespace control
{
class Robot
{
public:
    Robot(Drivers &drivers);

    void initSubsystemCommands();

private:
    void initializeSubsystems();
    void registerSoldierSubsystems();
    void setDefaultSoldierCommands();
    void startSoldierCommands();
    void registerSoldierIoMappings();

    Drivers &drivers;

    // declare TurretSubystem
    turret::TurretSubsystem turret;

    // declare TurretGimbalCommand
    turret::TurretGimbalCommand turretGimbal;
};
}  // namespace control