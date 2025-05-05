#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"
__modified_by__ = "Jinrae Kim, kjl950403@gmail.com"
__modified_note__ = "Implemented different tracking modes for comparison"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from geometry_msgs.msg import Point

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition


class OffboardControl(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.vehicle_status_callback,
            qos_profile,
        )
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.publisher_desired_position = self.create_publisher(
            Point, "/desired_position", qos_profile
        )
        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.state_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.cmdloop_callback,
            qos_profile,
        )
        # self.dt = timer_period
        self.t = None
        self.declare_parameter("radius", 10.0)
        self.declare_parameter("omega", 5.0)
        self.declare_parameter("altitude", 5.0)
        self.declare_parameter("tracking_mode", "precise")
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.radius = self.get_parameter("radius").value
        self.omega = self.get_parameter("omega").value
        self.altitude = self.get_parameter("altitude").value
        self.tracking_mode = self.get_parameter("tracking_mode").value

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        vx = msg.vx
        vy = msg.vy
        vz = msg.vz
        p_x_ref = self.radius * np.cos(self.theta)
        p_y_ref = self.radius * np.sin(self.theta)
        p_z_ref = -self.altitude
        desired_position_msg = Point()
        desired_position_msg.x = p_x_ref
        desired_position_msg.y = p_y_ref
        desired_position_msg.z = p_z_ref
        self.publisher_desired_position.publish(desired_position_msg)
        v_x_ref = -self.radius * np.sin(self.theta) * self.omega
        v_y_ref = self.radius * np.cos(self.theta) * self.omega
        v_z_ref = 0.0
        a_x_ref = -(self.omega**2) * self.radius * np.cos(self.theta)
        a_y_ref = -(self.omega**2) * self.radius * np.sin(self.theta)
        a_z_ref = 0.0

        e_p_x = x - p_x_ref
        e_p_y = y - p_y_ref
        e_p_z = z - p_z_ref
        e_v_x = vx - v_x_ref
        e_v_y = vy - v_y_ref
        e_v_z = vz - v_z_ref

        self.get_logger().info(f"tracking_mode: {self.tracking_mode}")
        self.get_logger().info(f"e_p_x: {e_p_x}")
        self.get_logger().info(f"e_p_y: {e_p_y}")
        self.get_logger().info(f"e_p_z: {e_p_z}")
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        t_micro = int(Clock().now().nanoseconds / 1000)
        offboard_msg.timestamp = t_micro
        if self.tracking_mode == "precise":
            offboard_msg.position = False  # idk why but you should inject nan as well
            offboard_msg.velocity = False  # idk why but you should inject nan as well

            offboard_msg.acceleration = True
        elif self.tracking_mode == "with_ff":
            offboard_msg.position = True
            offboard_msg.velocity = True

            offboard_msg.acceleration = True
        elif self.tracking_mode == "default":
            offboard_msg.position = True
            offboard_msg.velocity = False

            offboard_msg.acceleration = False
        else:
            raise ValueError(f"Invalid tracking_mode: {self.tracking_mode}")
        self.publisher_offboard_mode.publish(offboard_msg)
        if (
            self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        ):
            trajectory_msg = TrajectorySetpoint()
            if self.tracking_mode == "precise":
                a_x_cmd = -2.0 * e_v_x - 4.0 * e_p_x + a_x_ref
                a_y_cmd = -2.0 * e_v_y - 4.0 * e_p_y + a_y_ref
                a_z_cmd = -2.0 * e_v_z - 4.0 * e_p_z + a_z_ref
                trajectory_msg.position[0] = float("nan")  # REQUIRED
                trajectory_msg.position[1] = float("nan")  # REQUIRED
                trajectory_msg.position[2] = float("nan")  # REQUIRED
                trajectory_msg.velocity[0] = float("nan")  # REQUIRED
                trajectory_msg.velocity[1] = float("nan")  # REQUIRED
                trajectory_msg.velocity[2] = float("nan")  # REQUIRED
                trajectory_msg.acceleration[0] = a_x_cmd
                trajectory_msg.acceleration[1] = a_y_cmd
                trajectory_msg.acceleration[2] = a_z_cmd
            elif self.tracking_mode == "with_ff":
                trajectory_msg.position[0] = p_x_ref
                trajectory_msg.position[1] = p_y_ref
                trajectory_msg.position[2] = p_z_ref
                trajectory_msg.velocity[0] = v_x_ref
                trajectory_msg.velocity[1] = v_y_ref
                trajectory_msg.velocity[2] = v_z_ref
                trajectory_msg.acceleration[0] = a_x_ref
                trajectory_msg.acceleration[1] = a_y_ref
                trajectory_msg.acceleration[2] = a_z_ref
            elif self.tracking_mode == "default":
                trajectory_msg.position[0] = p_x_ref
                trajectory_msg.position[1] = p_y_ref
                trajectory_msg.position[2] = p_z_ref
            else:
                raise ValueError(f"Invalid tracking_mode: {self.tracking_mode}")
            self.publisher_trajectory.publish(trajectory_msg)

            if self.t is not None:
                dt = t_micro / 1e6 - self.t
                self.theta = self.theta + self.omega * dt
            self.t = t_micro / 1e6


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
