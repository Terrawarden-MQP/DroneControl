#!/usr/bin/env python

################################################################################
#
# Copyright (c) 2018-2022, PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

"""
Example to launch a sensor_combined listener node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    XRCE_agent = DeclareLaunchArgument(
        'XRCE_agent',
        default_value='true',
        description='Start microROS agent'
    )

    drone_telemetry_debug = DeclareLaunchArgument(
        'drone_telemetry_debug',
        default_value='false',
        description='Enable drone telemetry debug prints'
    )

    # Create the microROS agent process
    micro_ros_agent = ExecuteProcess(
        condition=LaunchConfiguration('XRCE_agent'),
        cmd=['MicroXRCEAgent udp4 --port 8888 -v'],
        shell=True
    )

    # Create the vehicle position listener node
    vehicle_position_listener_node = Node(
        package='wpi_drone',
        executable='vehicle_position_listener',
        name='vehicle_position_listener',
        output='screen',
        shell=True,
        parameters=[{
            'drone_telemetry_debug': LaunchConfiguration('drone_telemetry_debug')
        }]
    )

    return LaunchDescription([
        XRCE_agent,
        drone_telemetry_debug,
        micro_ros_agent,
        vehicle_position_listener_node
    ])
