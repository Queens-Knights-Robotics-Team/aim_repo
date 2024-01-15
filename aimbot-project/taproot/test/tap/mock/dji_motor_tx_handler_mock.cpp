/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2021 QKRT
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "dji_motor_tx_handler_mock.hpp"

namespace tap::mock
{
DjiMotorTxHandlerMock::DjiMotorTxHandlerMock(tap::Drivers *drivers)
    : tap::motor::DjiMotorTxHandler(drivers)
{
}
DjiMotorTxHandlerMock::~DjiMotorTxHandlerMock() {}
}  // namespace tap::mock