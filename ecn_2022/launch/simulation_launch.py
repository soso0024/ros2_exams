#!/usr/bin/env python3

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    sl.rviz(sl.find('ecn_2022', 'config.rviz'))
    sl.node('baxter_simple_sim', 'simulator')

    if sl.ros_version() < 'humble':
        args = "0 0 .9 0 0 0 ground base"
    else:
        args = "--z .9 --frame-id ground --child-frame-id base"

    sl.node('tf2_ros', 'static_transform_publisher', arguments=args.split())

    return sl.launch_description()
