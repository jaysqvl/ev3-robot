#!/usr/bin/env pybricks-micropython
#Initializes the brick down below
#from pybricks import ev3brick as brick
from pybricks.hubs import EV3Brick
#Initialize additional libraries
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

#Initialize motors + light sensors
brick = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
color_sensor = ColorSensor(Port.S1)
ultra_sensor = UltrasonicSensor(Port.S4)

# Initial Values
greenBeeped = 0
blueBeeped = 0
distance = 100000000
lastColor = 1

def stop_beep():
    #Stop the motors and beep
    right_motor.run(0)
    left_motor.run(0)
    brick.speaker.set_volume(100, which='_all_')
    brick.speaker.beep(frequency = 1500, duration = 2000)
    wait(2000)

def chooseAction():
    global greenBeeped
    global blueBeeped
    if distance <= 150:
        if (lastColor == 0):
            if blueBeeped == 0:
                blueBeeped = 1
                blue_sees_object()
        elif (lastColor == 1):
            if greenBeeped == 0:
                greenBeeped = 1
                green_sees_object()

#SPECIAL GREEN INSTRUCTION
def green_sees_object():
    stop_beep()

    # Static action
	# move it forward
    right_motor.run(300)
    left_motor.run(300)
    wait(400)

    # rotate right
    right_motor.run(-100)
    left_motor.run(100)
    wait(1000)

	# move it forward
    right_motor.run(300)
    left_motor.run(300)
    wait(1500)

	# move backwards
    right_motor.run(-300)
    left_motor.run(-300)
    wait(1800)

	# rotate back left
    right_motor.run(100)
    left_motor.run(-100)
    wait(1000)

#SPECIAL BLUE INSTRUCTION
def blue_sees_object():
    #Distance is a bit larger than it should be (100) as a safety range
    #Setting silent to true turns the sensor off after measuring the distance
    stop_beep()
    
    #Reverse
    right_motor.run(-300)
    left_motor.run(-300)
    wait(500)

    #Spin right a bit to adjust
    right_motor.run(200)
    left_motor.run(-200)
    wait(1000)

    #Move forward to cross the line to be on the right side of the tape
    right_motor.run(200)
    left_motor.run(200)
    wait(500)

    #Hurry the turning process, turn right some more
    right_motor.run(300)
    left_motor.run(0)
    wait(1500)

while True:
    distance = ultra_sensor.distance()
    
    #Display to the screen
    brick.screen.clear()

    #Displays current distance measured
    brick.screen.draw_text(0, 30, ultra_sensor.distance(), text_color=Color.BLACK, background_color=None)

    # Displays current rgb values
    r, g, b = color_sensor.rgb()
    brick.screen.draw_text(0, 50, r, text_color=Color.BLACK, background_color=None)
    brick.screen.draw_text(0, 70, g, text_color=Color.BLACK, background_color=None)
    brick.screen.draw_text(0, 90, b, text_color=Color.BLACK, background_color=None)

    #TURN LEFT WHEN SEE TABLE
    #original 44 58 60 61 95 100
    if r >= 44 and r <= 58 and g >= 60 and g <= 71 and b >= 98 and b <= 100:
        right_motor.run(100)
        left_motor.run(-50)

    #TURN RIGHT WHEN SEE GREEN DARK
    elif r >= 7 and r <= 11 and g >= 31 and g <= 45 and b >= 26 and b <= 50:
        right_motor.run(-50)
        left_motor.run(100)
        chooseAction()
        lastColor = 1

    #TURN RIGHT WHEN SEE BLUE DARK
    elif r >= 6 and r <= 15 and g >= 7 and g <= 28 and b >= 29 and b <= 81:
        right_motor.run(-50)
        left_motor.run(100)
        wait(10)
        chooseAction()
        lastColor = 0

    #GO STRAIGHT IF ANYTHING ELSE
    else:
        right_motor.run(-50)
        left_motor.run(100)
        chooseAction()