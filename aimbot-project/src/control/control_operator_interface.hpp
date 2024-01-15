#pragma once

#include "tap/util_macros.hpp" // includes mockable

namespace tap::communication::serial
{
class Remote;
}

namespace control
{
class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::communication::serial::Remote &remote);

    // Add getTurretPitchInput and getTurretYawInput function declarations
    mockable float getTurretPitchInput();                 

    mockable float getTurretYawInput();

private:
    tap::communication::serial::Remote &remote;
};
}  // namespace control
