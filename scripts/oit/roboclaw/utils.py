# -*- coding: utf_8 -*-
import math
import tf

def clamp(min_val, max_val, val):
    """
    Cut a value from min to max.
    """
    val = max(min_val, val)
    val = min(max_val, val)
    return val


def clamp_abs(limit, val):
    return clamp(-abs(limit), abs(limit), val)

def print_odom(odom):
    """
    Odometry to string.
    """
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return ("Pose:%f %f %f"
            % (odom.pose.pose.position.x, odom.pose.pose.position.y, math.degrees(euler[2])))