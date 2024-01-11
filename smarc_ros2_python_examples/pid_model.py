#!/usr/bin/python3

from sam_view import SAMView

import numpy as np
import math

class PIDModel():
    """
    This is more or less a verbatim copy of the WaypointFollowingController
    from https://github.com/DoernerD/visual_feedback/blob/velocity_control/src/WaypointFollowingController.py
    and probably needs some modification before it works proper. /Ozer
    
    But just the parts that handle an abstract state and produce an abstract control input, no ROS stuff.
    """
    def __init__(self,
                 loop_freq: int
                 ):

        self.loop_freq = loop_freq

        # Init
        self.state_estimated = np.array([0., 0., 0.1, 1., 0., 0.])
        self.x_ref = 0.
        self.y_ref = 0.
        self.z_ref = 0.
        self.roll_ref = 0.
        self.pitch_ref = 0.
        self.yaw_ref = 0.
        self.ref = np.array([self.x_ref, self.y_ref, self.z_ref, self.roll_ref, self.pitch_ref, self.yaw_ref])

        self.ref_odom = np.array([0., 0., 0., 0., 0., 0.])

        self.velocity = np.array([0., 0., 0., 0., 0., 0.])
        self.vel_x_ref = 0.
        self.vel_y_ref = 0.
        self.vel_z_ref = 0.
        self.vel_roll_ref = 0.
        self.vel_pitch_ref = 0.
        self.vel_yaw_ref = 0.
        self.vel_ref = np.array([self.vel_x_ref, self.vel_y_ref, self.vel_z_ref, self.vel_roll_ref, self.vel_pitch_ref, self.vel_yaw_ref])

        # Control Gains
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]
        self.Kp = np.array([2000, 5, 5, 10, 70])      # P control gain
        self.Ki = np.array([10., 0.1, 0.1, 0.001, 0.5])    # I control gain
        self.Kd = np.array([1., 1., 1., 0., 0.])    # D control gain
        self.Kaw = np.array([1., 1., 1., 0., 6.])   # Anti windup gain

        self.eps_depth = 0.6 # offset for depth control
        self.eps_pitch = 0.3 # offset for pitch control

        self.error = np.array([0., 0., 0., 0., 0., 0.])
        self.error_prev = np.array([0., 0., 0., 0., 0., 0.])
        self.integral = np.array([0., 0., 0., 0., 0., 0.])

        self.distance_error = 0.

        self.heading_angle_error = 0.
        self.heading_angle_error_integral = 0.
        self.heading_angle_error_prev = 0.
        self.heading_angle_error_deriv = 0.

        self.error_velocity = 0.
        self.error_velocity_integral = 0.
        self.error_velocity_prev = 0.
        self.error_velocity_deriv = 0.

        self.stop_radius = 0.2
        self.rpm_limit = 230.   # hard rpm limit for safety in the tank

        # Neutral actuator inputs
        self.thruster_neutral = 0
        self.horizontal_thrust_vector_neutral = 0.
        self.vertical_thrust_vector_neutral = 0.
        self.vbs_neutral = 57.
        self.lcg_neutral = 80.

        self.u_neutral = np.array([self.thruster_neutral,
                                  self.horizontal_thrust_vector_neutral,
                                  self.vertical_thrust_vector_neutral,
                                  self.vbs_neutral,
                                  self.lcg_neutral])

        self.anti_windup_diff = np.array([0., 0., 0., 0., 0.])
        self.anti_windup_diff_integral = np.array([0., 0., 0., 0., 0.])

        self.limit_output_cnt = 0 


    def set_pose(self,
                 x:float, y:float, z:float,
                 roll:float, pitch:float, yaw:float):
        """
        Set the estimated pose of the vehicle.
        x,y,z in meters.
        roll, pitch, yaw in radians.
        """
        # dont create lists, update the array...
        self.state_estimated[0] = x
        self.state_estimated[1] = y
        self.state_estimated[2] = z
        self.state_estimated[3] = roll
        self.state_estimated[4] = pitch
        self.state_estimated[5] = yaw

    def set_velocity(self,
                     lx:float, ly:float, lz:float,
                     ax:float, ay:float, az:float):
        """
        Set the velocity of the vehicle.
        lx, ly, lz are linear components and ax, ay, az are angular components
        in m/s and radians/s respectively.
        """
        self.velocity[0] = lx
        self.velocity[1] = ly
        self.velocity[2] = lz
        self.velocity[3] = ax
        self.velocity[4] = ay
        self.velocity[5] = az


    def set_waypoint(self,
                     x:float, y:float, z:float,
                     roll:float, pitch:float, yaw:float,
                     lx:float):
        """
        Set the goal waypoint to reach in absolute coordinates, in the same
        frame of reference as the vehicle pose.
        x,y,z for position
        roll, pitch, yaw for orientation at given position
        lx for forward speed at given pose
        """
        self.ref[0] = x - self.state_estimated[0]
        self.ref[1] = y - self.state_estimated[1]
        # We don't transform the depth and the pitch
        # since we compare them to sensor data and 
        # therefore need absolute values. 
        self.ref[2] = z

        # Roll
        # self.ref[3] = roll
        # Pitch
        # FIXME: Something is off with the sign and potentially the controller /David
        self.ref[4] = 0.0 # pitch
        # Yaw
        # self.ref[5] = yaw

        # Velocity
        self.vel_ref[0] = lx


    def calculate_anti_windup_integral(self):
        """
        Calculate the anti windup integral
        """
        self.anti_windup_diff_integral += (self.anti_windup_diff) * (1/self.loop_freq)


    def calculate_velocity_control_action(self, velocity_desired):
        """
        Returns RPM based on the desired velocity in x direction.
        """
        u = 250

        # self.error_velocity_prev = self.error_velocity
        # self.error_velocity = velocity_desired - self.velocity[0]
        # self.error_velocity_integral += self.error_velocity * (1/self.loop_freq)
        # self.error_velocity_deriv = (self.error_velocity - self.error_velocity_prev) * self.loop_freq

        # u = self.Kp[0]*self.error_velocity + self.Ki[0]*(self.error_velocity_integral - self.anti_windup_diff_integral[0]) + self.Kd[0]*self.error_velocity_deriv

        return u


    def compute_control_action(self):
        """
        Sliding Mode Control for Depth First control.
        The control structure is as follows:
            The error is used for pitch and depth control.
            The heading angle is used for horizontal control and calculated separately.
            The distance to the target is used for forward and backward control.
        u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        x = [x, y, z, roll, pitch, yaw]
        """

        u = np.array([self.thruster_neutral,
                        self.horizontal_thrust_vector_neutral,
                        self.vertical_thrust_vector_neutral,
                        self.vbs_neutral,
                        self.lcg_neutral])

        self.error_prev = self.error
        self.error = self.ref - self.state_estimated

        # Depth is always negative, which is why we change the signs on the
        # depth error. Then we can keep the remainder of the control structure
        self.error[2] = -self.error[2]

        # Anti windup integral is calculated separately, because
        # dim(e) != dim(u).
        self.calculate_anti_windup_integral()

        self.integral += self.error * (1/self.loop_freq)
        self.deriv = (self.error - self.error_prev) * self.loop_freq

        # Calculate distance to reference pose
        # Workaround bc np.sign(0) = 0
        if np.sign(self.ref[0]) == 0:
            distance_sign = 1
        else:
            distance_sign = np.sign(self.ref[0])

        self.distance_error = np.sqrt(self.ref[0]**2 + self.ref[1]**2) * distance_sign

        # When on navigation plane
        if ((np.abs(self.state_estimated[2] - self.ref[2]) <= self.eps_depth)\
                and (np.abs(self.state_estimated[4] - self.ref[4]) <= self.eps_pitch)):

            # print("velocity control")

            # Compute heading angle
            self.heading_angle = math.atan2(self.ref[1], self.ref[0])
            self.heading_angle_error_prev = self.heading_angle_error
            self.heading_angle_error = 0 - self.heading_angle
            self.heading_angle_error_integral += self.heading_angle_error * (1/self.loop_freq)
            self.heading_angle_error_deriv = (self.heading_angle_error - self.heading_angle_error_prev) * self.loop_freq

            
            u[0] = 0
            # FIXME: Check the u[1] calculation. Seems sketchy with the flipping and esp. with the integral when you change signs. Not good!
            if self.distance_error > 1: #self.stop_radius:
                u[0] = self.calculate_velocity_control_action(self.vel_ref[0])

                u[1] = self.Kp[1]*self.heading_angle + self.Ki[1]*(self.heading_angle_error_integral - self.anti_windup_diff_integral[1]) + self.Kd[1]*self.heading_angle_error_deriv   # PID control vectoring (horizontal)
                u[1] = -u[1] # FIXME: This is a hack to get the sign right. This is due to the conversion from ENU to NED for the thruster commands

            elif self.distance_error < -0.2:    # FIXME: 1m past the waypoint is too much for now to actually stop. Once we have velocity control, we can change it back again.
                u[0] = self.calculate_velocity_control_action(-0.5*self.vel_ref[0])
                u[0] = -250

                self.heading_angle_scaled = np.sign(self.heading_angle) * (np.pi - np.abs(self.heading_angle))

                u[1] = -(self.Kp[1]*self.heading_angle_scaled + (self.Ki[1]*(self.heading_angle_error_integral - self.anti_windup_diff_integral[1])) + self.Kd[1]*self.heading_angle_error_deriv)   # PID control vectoring (horizontal)
                u[1] = -u[1] # FIXME: This is a hack to get the sign right. This is due to the conversion from ENU to NED for the thruster commands

            else:
                u[0] = 0 #self.calculate_velocity_control_action(0)

        u[3] = self.Kp[3]*self.error[2] + self.vbs_neutral + self.Ki[3]*(self.integral[2] - self.anti_windup_diff_integral[3]) + self.Kd[3]*self.deriv[2]   # PID control vbs
        u[4] = self.Kp[4]*self.error[4] + self.lcg_neutral + self.Ki[4]*(self.integral[4] - self.anti_windup_diff_integral[4]) + self.Kd[4]*self.deriv[4]   # PID control lcg

        return u




if __name__ == "__main__":
    # these are not required by the class above, just for testing
    # see sam_view.py for comments on how main works in ros2 now
    import rclpy, sys

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("SAMViewTestNode")

    view = SAMView(node)

    loop_freq = 10
    model = PIDModel(loop_freq)

    # just a simple waypoint dead-ahead
    model.set_waypoint(10,0,0,0,0,0,0)

    rate = node.create_rate(loop_freq)
    while(rclpy.ok()):
        u = model.compute_control_action()
        node.get_logger().info(f"u = {u}")
        view.set_control_inputs(*u)
        view.update()

        rclpy.spin_once(node)
        rate.sleep() 
