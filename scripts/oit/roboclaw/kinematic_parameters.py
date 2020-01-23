# -*- coding: utf_8 -*-
"""
Description of differential drive robot's kinematic parameters.
"""
from collections import namedtuple


def build_kinematic_parameters(ticks_per_meter, linear_max, angular_max):
    """
    Factory of 'KinematicParameters' structure.
    """
    factory = namedtuple("KinematicParameters",
                         ['ticks_per_meter', 'linear_max', 'angular_max'])
    return factory(ticks_per_meter, linear_max, angular_max)
