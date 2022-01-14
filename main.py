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
    wait(sec_to_ms(0.25))

#Keeps robot going straight for given distance in mm
def gyro_straight(distance):
    robot.reset()
    gyro.reset_angle(0)

    angle = gyro.angle()

    while robot.distance() <= distance:
        angle = gyro.angle()
        print(angle)
        robot.drive(175, angle * -1)

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
    gyro_straight(256)
    gyro_turn(50)
    gyro_straight(397)
    gyro_turn(36)
    gyro_straight(800)
    gyro_turn(-31)
    gyro_straight(620)

def missionplane():
    gyro_straight(inches_to_mm(23))
    gyro_turn(-29)
    gyro_straight(inches_to_mm(2.5))
    front_motor.run_angle(1000, -3000)
    front_motor.run_angle(1000, 3000)
    gyro_turn(-110)
    gyro_straight(inches_to_mm(25))

def truck():
    gyro_straight(inches_to_mm(18))
    robot.straight(inches_to_mm(-18))
    print("test")

def lift():
    front_motor.run_angle(1000, 300)

def wait_for_button(ev3):
    #This code comes from the button example project in the ev3 module
    pressed = []
    while len(pressed) != 1:
        pressed = ev3.buttons.pressed()
    button = pressed[0]

    while any(ev3.buttons.pressed()):
        pass

    return button

#This function waits for a button to be pressed.
#Once the button is pressed it plays the code
#The while true statement makes it so this only happens when the button is "true"
def button_choices():
    while True:
        button = wait_for_button(ev3)

        if button == Button.RIGHT:
            ev3.speaker.beep(400)
            mission_airdrop()
        elif button == Button.LEFT:
            ev3.speaker.beep(200) 
            missionplane()
        elif button == Button.DOWN:
            ev3.speaker.beep(600)
            truck()
        elif button == Button.UP:
            ev3.speaker.beep(800)
            lift()

# INITIALIZING OBJECTS
ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
front_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, 120)
robot.settings(200, 50, 150, 100)
gyro = GyroSensor(Port.S4)

# PROGRAM
button_choices()