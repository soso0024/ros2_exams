#!/usr/bin/env python3

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('ecn_2022', 'twist.yaml')])

    return sl.launch_description()
