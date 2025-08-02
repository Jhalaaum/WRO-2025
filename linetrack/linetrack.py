from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

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
def degToMM(degress):
    return (degress/(2*62.4*umath.pi))

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

# while db.distance() < 1000:
#     singleSensorLinetrack("left")
#     if db.distance() - last_logged_distance >= 10:
#         dist_array.append(last_logged_distance)
#         reflection = sensor.reflection()
#         error = TARGET - reflection
#         error_array.append(error)
#         last_logged_distance = db.distance()
# singleLineTrackByDist(100, "left")
# singleLineTrackTillIntersection("left")
# print(dist_array)
# print("\n")
# print(error_array)

def calculateSpeed(
        db,
        initial_speed,
        max_speed,
        final_speed,
        accel_percentage,
        brake_percentage,
        driven_percentage,
    ):
        """
        Description:
        Given initial, max and final speeds, return the calculated speed given the percentage distance acceleration stops
        and the beginning of braking

        Inputs:
        - *initial_speed* -> Starting speed (0-500)
        - *max_speed* -> Highest (cruising) speed (0-500)
        - *final_speed* -> The ending speed (speed at the end of distance) (0-500)
        - *accel_percentage* -> Percentage of distance until which acceleration (from initial to max) occurs
        - *brake_percentage* -> Percentage of distance after which braking (from max to final) occurs
        - *driven_percentage* -> Percentage of distance driven

        Output:
        -  *speed* -> Speed (0-500)
        """
        a = 33.36
        b = 2/(umath.pi)

        # Formula: y = (10x/b)^2 => [0, 100]
        # NEW Formula: y = 100*sin(x/b)
        range_percentage = 0
        speed = max_speed
        both_ranges = 0
        #print("DEBUGLL", driven_percentage, accel_percentage)
        if driven_percentage < accel_percentage:
            both_ranges += 1
            progress = driven_percentage / accel_percentage
            # range_percentage = (10 * driven_percentage / accel_percentage) ** 2
            # speed = initial_speed + range_percentage / 100 * (max_speed - initial_speed)
            range_percentage = 100*umath.sin((progress)/b)
            speed = initial_speed + range_percentage / 100 * (max_speed - initial_speed)
           # print(progress)
           #print(speed)

        if driven_percentage > brake_percentage:
            both_ranges += 1
            progress = (1 - driven_percentage) / (1 - brake_percentage)
            range_percentage = (
                10 * (1 - driven_percentage) / (1 - brake_percentage)
            ) ** 2
            speed = min(
                speed, final_speed + range_percentage / 100 * (max_speed - final_speed)
            )
        return speed

def gyroDriveStraight(
        db,
        distance,
        hold=True,
        scan_color=None,
        track_heading="NA",
        initial_speed=200,
        max_speed=500,
        final_speed=75,
        accel_dist=2,
        brake_dist=400, #150 is stable
    ):
        """
        Description:
        Gyro function to drive straight

        Inputs:
        - *distance* -> The distance that the bot needs to travel (mm)
        - *initial_speed* -> Starting speed (0-500)
        - *max_speed* -> Highest (cruising) speed (0-500)
        - *final_speed* -> The ending speed (speed at the end of distance) (0-500)
        - *accel_dist* -> Distance until which acceleration (from initial to max) occurs (Default = 50)
        - *brake_dist* -> Distance for which braking (from max to final) occurs (Default = 150)
        - *track_heading* -> Set the heading to track, (Default = NA)
        - *hold* -> Indicates whether to stop the motors at the end of the straight (Default = True)
        """

        db.reset()
        initial_heading = hub.imu.heading()

        accel_percentage = abs(accel_dist / distance)
        brake_percentage = ((abs(distance) - abs(brake_dist)) / abs(distance))

        left_initial_angle = db.left_motor.angle()
        right_initial_angle = db.right_motor.angle()
        curr_distance = db.degToMM(0)

        print("L: ", db.left_motor.angle(), "R: ", db.right_motor.angle())

        while abs(curr_distance) < abs(distance):
            speed = db.calculateSpeed(
                initial_speed,
                max_speed,
                final_speed,
                accel_percentage,
                brake_percentage,
                abs(max(1, abs(curr_distance)) / distance),
            )
            speed_decimal = speed / 500
            if track_heading != "NA":
                if track_heading == "TARGET":
                    error = db.rebaseAngle(hub.imu.heading() - db.target_heading)
                else:
                    error = db.rebaseAngle(hub.imu.heading() - track_heading)
            else:
                error = db.rebaseAngle(hub.imu.heading() - initial_heading)
            db.heading_errors.append(error)
            db.heading_integral += error
            if len(db.heading_errors) > 100:
                db.heading_integral -= db.heading_errors[0]
                db.heading_errors.pop(0)

            heading_error = (
                error * db.kP_drive
            )

            if distance > 0:
                if heading_error > 0:
                    db.left_motor.run((speed_decimal * (1500 - heading_error)) / 1)
                    db.right_motor.run((speed_decimal * (1500)) / 1)
                else:
                    db.left_motor.run((speed_decimal * (1500)) / 1)
                    db.right_motor.run((speed_decimal * (1500 + heading_error)) / 1)
            else:
                if heading_error > 0:
                    db.left_motor.run((speed_decimal * (-1500)) / 1)
                    db.right_motor.run((speed_decimal * (-1500 + heading_error)) / 1)
                else:
                    db.left_motor.run((speed_decimal * (-1500 - heading_error)) / 1)
                    db.right_motor.run((speed_decimal * (-1500)) / 1)

            curr_distance = db.degToMM(((db.left_motor.angle() - left_initial_angle)) / 1)
            db.heading_errors.clear()
            db.heading_integral = 0

        
        db.rebaseIMUAngle()