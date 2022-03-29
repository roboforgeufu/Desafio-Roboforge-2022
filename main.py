#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, 
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize.
test_motorB = Motor(Port.B)
test_motorC = Motor(Port.C)
sensorc1 = ColorSensor(Port.S1)
sensorc2 = ColorSensor(Port.S2)
cronometro = StopWatch()

#robo = DriveBase(test_motorB,test_motorC,8.5,24)

ev3.speaker.beep()

while sensorc1.color()!=Color.BLACK or sensorc2.color()!=Color.BLACK:
    test_motorB.run(150)
    test_motorC.run(150)
    cronometro.reset()
    if(sensorc1.color()==Color.BLACK):
        test_motorB.run(-300)
        test_motorC.run(150)
    if(sensorc2.color()==Color.BLACK):
        test_motorB.run(150)
        test_motorC.run(-300)

test_motorB.stop()
test_motorC.stop()
