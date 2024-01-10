#!/usr/bin/python3

import rclpy
import sys

from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import PercentStamped, ThrusterAngles


class SAMView:
    """
    A View class that can tell sam to do things.
    If you wanted to be fancy, you could also create a
    base class for a generic thrust-vectoring system and extend it
    with sam-specifics here, but for simplicity, we shan't do that here.

    If needed, you can extend _this_ class with one that also writes the control
    inputs to file, to plot or something as well!
    """
    def __init__(self, node):
        # bunch of topics to publish to
        vbs_topic = node.declare_parameter("vbs_topic", "core/vbs_cmd").value
        lcg_topic = node.declare_parameter("lcg_topic", "core/lcg_cmd").value
        rpm1_topic= node.declare_parameter("rpm1_topic", "core/thruster1_cmd").value
        rpm2_topic= node.declare_parameter("rpm2_topic", "core/thruster2_cmd").value
        thrust_vector_topic = node.declare_parameter("thrust_vector_cmd_topic", "core/thrust_vector_cmd").value

        # lets actually use private variable notations this time around eh?
        # ROS2 introduces a QoS setting to publishing and subsciribing(optional)
        # that might be of use later, but for this simple example, we just set
        # a default value of "10" which more or less mimics ROS1
        self._vbs_pub = node.create_publisher(PercentStamped, vbs_topic, 10)
        self._lcg_pub = node.create_publisher(PercentStamped, lcg_topic, 10)
        self._rpm1_pub = node.create_publisher(ThrusterRPM, rpm1_topic, 10)
        self._rpm2_pub = node.create_publisher(ThrusterRPM, rpm2_topic, 10)
        self._thrust_vector_pub = node.create_publisher(ThrusterAngles, thrust_vector_topic, 10)

        self._t1_msg = ThrusterRPM()
        self._t2_msg = ThrusterRPM()
        self._vec_msg = ThrusterAngles()
        self._vbs_msg = PercentStamped()
        self._lcg_msg = PercentStamped()


    def set_u(self, u):
        """
        this should be called by a model to set the control inputs
        and can be done arbitrarily frequently
        """
        # check for validity and limits here if needed
        # also for types, like how rpms are ints but a control model
        # would likely give out floats and that setting radians=0
        # should make sure those ints become 0.0 floats
        self._t1_msg.rpm = int(u[0])
        self._t2_msg.rpm = int(u[0])
        self._vec_msg.thruster_horizontal_radians = float(u[1])
        self._vec_msg.thruster_vertical_radians = float(u[2])
        self._vbs_msg.value = float(u[3])
        self._lcg_msg.value = float(u[4])
        # you could also keep track of all the inputs given, record them,
        # write to file, even publish a different topic for debugging purposes
        # _lots of LOOKING, from the VIEW class_


    def update(self):
        """
        the actual publishing is done here for all the topics
        you can call this as fast the receiver needs the data repeated
        independently of the rate in which said data is updated
        """
        self._vbs_pub.publish(self._vbs_msg)
        self._lcg_pub.publish(self._lcg_msg)
        self._thrust_vector_pub.publish(self._vec_msg)
        self._rpm1_pub.publish(self._t1_msg)
        self._rpm2_pub.publish(self._t2_msg)



if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("SAMViewTestNode")
    view = SAMView(node)

    us = [
        [0,       0,   0,0,0],
        [500.14,  0,   0,0,0],
        [-500.21, 0,   0,0,0],
        [500.12,  0.15,0,0,0],
        [500.12, -0.15,0,0,0]
    ]

    # we wanna publish the above Us on a timer
    # could have also created a timer object instead of
    # doing the while loop, but this loop is a very common
    # thing we used to do in ros1 that an example is nice to have
    rate = node.create_rate(0.5)
    i = 0
    while(rclpy.ok()):
        # the new rate objects dont auto-spin, they just sleep
        # this is a general trend in ros2, most things do _one thing_
        # and dont do much else implicitly.
        view.set_u(us[i])
        view.update()
        node.get_logger().info(f"u = {us[i]}")

        i+=1
        i = i%len(us)

        rclpy.spin_once(node)
        rate.sleep() 
    