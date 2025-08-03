from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

# def straight(distance): # In mm
#     left_motor.reset_angle(0)
#     right_motor.reset_angle(0)
#     last_current = 0 
#     while True:
#         distance_traveled = (left_motor.angle() / 360) * 62.4
#         if distance_traveled >= distance:
#             break
#         increment = umath.pi * umath.sin((umath.pi * distance_traveled) / 50)
#         current = last_current + increment
#         left_power = min(100, current)
#         right_power = min(100, current)
#         left_motor.dc(left_power)
#         right_motor.dc(right_power)
#         last_current = current      
#     left_motor.stop()
#     right_motor.stop()
