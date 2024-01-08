#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "tap/communication/gpio/pwm.hpp"

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;

tap::gpio::Pwm pwm;


namespace control
{
ControlOperatorInterface::ControlOperatorInterface(Remote &remote) : remote(remote) {}

// Add getTurretPitchInput and getTurretYawInput function definitions
float ControlOperatorInterface::getTurretPitchInput()
{
    return limitVal(remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f);
    pwm.init();
    pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, 1000);
    pwm.start(tap::gpio::Pwm::Timer::TIMER8);
    pwm.write(limitVal(remote.getChannel(Remote::Channel::LEFT_VERTICAL), -1.0f, 1.0f), tap::gpio::Pwm::Pin::Z);
}

float ControlOperatorInterface::getTurretYawInput()
{
    return limitVal(remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), -1.0f, 1.0f);
}
}  // namespace control