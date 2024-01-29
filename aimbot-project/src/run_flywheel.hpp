/*

 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>

 *

 * This file is part of taproot-examples.

 *

 * taproot-examples is free software: you can redistribute it and/or modify

 * it under the terms of the GNU General Public License as published by

 * the Free Software Foundation, either version 3 of the License, or

 * (at your option) any later version.

 *

 * taproot-examples is distributed in the hope that it will be useful,

 * but WITHOUT ANY WARRANTY; without even the implied warranty of

 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the

 * GNU General Public License for more details.

 *

 * You should have received a copy of the GNU General Public License

 * along with taproot-examples.  If not, see <https://www.gnu.org/licenses/>.

 */

#ifndef RUN_FLYWHEEL_CPP_
#define RUN_FLYWHEEL_CPP_


#include "tap/board/board.hpp"



#include "drivers_singleton.hpp"



#include <unistd.h>

int run_fly(Drivers *drivers);

#endif  // RUN_FLYWHEEL_CPP_