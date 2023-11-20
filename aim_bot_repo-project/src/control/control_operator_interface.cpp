#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;

namespace control
{
ControlOperatorInterface::ControlOperatorInterface(Remote &remote) : remote(remote) {}

// Add getTurretPitchInput and getTurretYawInput function definitions
float ControlOperatorInterface::getTurretPitchInput()
{
    return limitVal(remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f);
}

float ControlOperatorInterface::getTurretYawInput()
{
    return limitVal(remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f);
}
}  // namespace control