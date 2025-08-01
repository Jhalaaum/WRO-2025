from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
sensor = ColorSensor(Port.D)
db = DriveBase(right_motor,left_motor, 62.4, 144)
db.use_gyro(True)
BLACK = 10
WHITE = 97
TARGET = (BLACK + WHITE) / 2
error_array = []
dist_array = []
last_logged_distance = 0
last_error = 0

def singleSensorLinetrack(Direction="right"):
    global last_error
    baseSpeed = 670
    reflection = sensor.reflection()
    error = TARGET - reflection
    kP = 2.5
    kD = 1.2
    derivative = error - last_error
    steering_value = kP*error + kD*derivative
    if Direction == "right":
        right_motor.run(baseSpeed - steering_value)
        left_motor.run(baseSpeed + steering_value)
    else:
        left_motor.run(baseSpeed - steering_value)
        right_motor.run(baseSpeed + steering_value) 
    last_error = error
def singleLineTrackByDist(Distance, Direction="right"):
    while db.distance() < Distance:
        singleSensorLinetrack(Direction)
def singleLineTrackTillIntersection(Direction="right"):
    while sensor.reflection() < 87:
        singleSensorLinetrack(Direction)
def straight():
# while db.distance() < 1000:
#     singleSensorLinetrack("left")
#     if db.distance() - last_logged_distance >= 10:
#         dist_array.append(last_logged_distance)
#         reflection = sensor.reflection()
#         error = TARGET - reflection
#         error_array.append(error)
#         last_logged_distance = db.distance()
singleLineTrackByDist(100, "left")
singleLineTrackTillIntersection("left")
# print(dist_array)
# print("\n")
# print(error_array)