#include "run_flywheel.hpp"

int run_fly(Drivers *drivers)
{
    while (1)

    {

        //min ia 0.25f max ia 0.50f (so not send 0.50f for shooting, might change motor direction)

        drivers->pwm.write(0.25f, tap::gpio::Pwm::X); // equivalent to 0.25 sec duty cycle

        drivers->pwm.write(0.25f, tap::gpio::Pwm::Y); // equivalent to 0.25 sec duty cycle



        modm::delay_ms(2000);



        //uncomment this to shoot

        drivers->pwm.write(0.30f, tap::gpio::Pwm::X);

        drivers->pwm.write(0.30f, tap::gpio::Pwm::Y);



        modm::delay_ms(2000);

    }

    return 0;
}