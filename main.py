#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import random

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
WHEEL_DIAMETER = 56

# FUNCTIONS HERE
def inches_to_mm(inches):
    return inches * 25.4

def sec_to_ms(seconds):
    MS = 1000
    return seconds * MS

def robot_stop():
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    wait(seconds(0.25))

#Keeps robot going straight for given distance in mm
def gyro_straight(distance):
    robot.reset()
    gyro.reset_angle(0)

    angle = gyro.angle()

    while robot.distance() <= distance:
        angle = gyro.angle()
        print(angle)
        robot.drive(100, angle * -1)

    robot_stop()

#Turns the robot a certain amount (turn_angle) and turns back for accuracy
def gyro_turn(turn_angle):
    angle = gyro.angle()
    
    if angle <= turn_angle:
        while angle < turn_angle:
            robot.turn(7)
            angle=gyro.angle()
            print(angle)

        while angle > turn_angle:
            robot.turn(-1)
            angle=gyro.angle()
            print(angle)
   
    elif angle >=turn_angle:
        while angle > turn_angle:
            robot.turn(-7)
            angle=gyro.angle()
            print(angle)

        while angle < turn_angle:
            robot.turn(1)
            angle=gyro.angle()
            print(angle)

#MISSIONS
def mission_airdrop():
    #Getting there
    gyro_straight(256)
    gyro_turn(46)
    gyro_straight(390)
    gyro_turn(41)
    gyro_straight(800)
    gyro_turn(-31)
    gyro_straight(600)

    #Interact with mission

    #Getting back

# INITIALIZING OBJECTS
ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
front_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, 120)
robot.settings(200, 100, 150, 100)
gyro = GyroSensor(Port.S4)

# PROGRAM


