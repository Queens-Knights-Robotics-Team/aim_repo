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

#ifndef TAPROOT_COMMAND_MAPPER_FORMAT_GENERATOR_HPP_
#define TAPROOT_COMMAND_MAPPER_FORMAT_GENERATOR_HPP_

#include <string>
#include <string_view>
#include <vector>

#include "tap/communication/serial/remote.hpp"

namespace tap
{
namespace control
{
class CommandMapper;
class RemoteMapState;
class Command;

/**
 * A utility for generating a readable format of the current command mappings in a particular
 * CommandMapper.
 */
class CommandMapperFormatGenerator
{
public:
    explicit CommandMapperFormatGenerator(const CommandMapper &mapper) : mapper(mapper) {}
    ~CommandMapperFormatGenerator() = default;

    /**
     * @return A list of mappings in string format, parsed from the CommandMapper
     *      passed into the class.
     * @note This is very slow because of the necessary std::string parsing. Never
     *      call this while performance matters.
     */
    const std::vector<std::string> generateMappings() const;

private:
    const CommandMapper &mapper;

    const std::string formattedRemoteMapState(const RemoteMapState &ms) const;
    const std::string formattedMappedCommands(const std::vector<Command *> mc) const;
    constexpr std::string_view switchStateToString(
        tap::communication::serial::Remote::SwitchState state) const;
    const std::string keyMapToString(uint16_t keys) const;
};  // class CommandMapperFormatGenerator
}  // namespace control
}  // namespace tap

#endif  // TAPROOT_COMMAND_MAPPER_FORMAT_GENERATOR_HPP_