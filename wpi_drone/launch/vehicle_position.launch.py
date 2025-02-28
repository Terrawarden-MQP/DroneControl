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
    components = []
    # Declare launch arguments
    # Debug parameters
    components.append(
        DeclareLaunchArgument(
        'debug_printout',
        default_value='true',
        description='Enable telemetry debug printouts'
    ))
    
    components.append(
        DeclareLaunchArgument(
        'debug_printout_period_s',
        default_value='2.0',
        description='Debug printout period in seconds'
    ))

    # Topic parameters
    components.append(
        DeclareLaunchArgument(
        'drone_pose_topic',
        default_value='drone/waypoint',
        description='Topic for sending waypoints to drone'
    ))

    components.append(
        DeclareLaunchArgument(
        'drone_telemetry_topic',
        default_value='drone/telemetry',
        description='Topic for receiving drone telemetry'
    ))

    components.append(DeclareLaunchArgument(
        'XRCE_agent',
        default_value='true',
        description='Start microROS agent'
    ))


    if LaunchConfiguration('XRCE_agent'):
        # Create the microROS agent process
        components.append(
            ExecuteProcess(
            cmd=['MicroXRCEAgent udp4 --port 8888 -v'],
            shell=True
        ))

    # Create the vehicle position listener node with all parameters
    components.append(
        Node(
        package='wpi_drone',
        executable='vehicle_position_listener',
        name='vehicle_position_listener',
        output='screen',
        shell=True,
        parameters=[{
            'debug_printout': LaunchConfiguration('debug_printout'),
            'debug_printout_period_s': LaunchConfiguration('debug_printout_period_s'),
            'drone_pose_topic': LaunchConfiguration('drone_pose_topic'),
            'drone_telemetry_topic': LaunchConfiguration('drone_telemetry_topic')
        }]
    ))

    return LaunchDescription(components)
