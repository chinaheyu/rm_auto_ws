#!/usr/bin/env python3
from robot_package.robot import Robot
import time

if __name__ == '__main__':
    robot = Robot()
    robot.echo('Flow control start.')
    while not robot.is_shutdown:
        pass
    

