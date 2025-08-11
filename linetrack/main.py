from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

HEADING_ERROR_WINDOW_SIZE = 100
kP_drive = 350 # for straight function
heading_errors = []
heading_integral = 0

hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
sensor = ColorSensor(Port.D)
db = DriveBase(left_motor, right_motor, 62.4, 144)
db.use_gyro(True)
BLACK = 14
WHITE = 99
TARGET = (BLACK + WHITE) / 2
error_array = []
dist_array = []
last_logged_distance = 0
last_error = 0

def singleSensorLinetrack(Direction="right", baseSpeed=670):
    """
    Follows a line using a single color sensor with a variable base speed.
    """
    global last_error
    reflection = sensor.reflection()
    error = TARGET - reflection
    kP = 2.5
    kD = 1.2
    derivative = error - last_error
    steering_value = kP * error + kD * derivative

    if Direction == "right":
        right_motor.run(baseSpeed - steering_value)
        left_motor.run(baseSpeed + steering_value)
    else:
        left_motor.run(baseSpeed - steering_value)
        right_motor.run(baseSpeed + steering_value)
    
    last_error = error
def singleLineTrackByDist(
    Distance,
    Direction="right",
    initial_speed=150,
    max_speed=670,
    final_speed=100,
    accel_dist=60,
    brake_dist=60
):
    """
    Tracks a line for a specific distance with controlled acceleration and deceleration.
    """
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    current_distance = 0
    
    if abs(Distance) == 0:
        return

    accel_percentage = abs(accel_dist / Distance)
    brake_percentage = (abs(Distance) - abs(brake_dist)) / abs(Distance)
    
    while abs(current_distance) < abs(Distance):
        driven_percentage = abs(current_distance / Distance)

        current_speed = calculateSpeed(
            db,
            initial_speed,
            max_speed,
            final_speed,
            accel_percentage,
            brake_percentage,
            driven_percentage,
        )
        
        singleSensorLinetrack(Direction, current_speed)
        
        avg_motor_angle = (left_motor.angle() + right_motor.angle()) / 2
        current_distance = degToMM(avg_motor_angle)

    left_motor.brake()
    right_motor.brake()
def singleLineTrackTillIntersection(Direction="right"):
    while sensor.reflection() > 20:
        singleSensorLinetrack(Direction)
    left_motor.brake()
    right_motor.brake()
def degToMM(degrees):
    return (degrees / 360) * (62.4 * umath.pi)
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
def rebaseAngle(angle):
  """
  Normalizes an angle to the range of -180 to 180 degrees.
  """
  while angle > 180:
    angle -= 360
  while angle < -180:
    angle += 360
  return angle

def rebaseIMUAngle():
    hub.imu.reset_heading(0)

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
        global heading_errors, heading_integral, kP_drive
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

        left_initial_angle = left_motor.angle()
        right_initial_angle = right_motor.angle()
        curr_distance = degToMM(0)

        print("L: ", left_motor.angle(), "R: ", right_motor.angle())
        log_interval = 1  # mm
        last_logged_dist = 0
        error_log = []
        distance_log = []
        while abs(curr_distance) < abs(distance):
            speed = calculateSpeed(
                db,
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
                    error = rebaseAngle(hub.imu.heading() - db.target_heading)
                else:
                    error = rebaseAngle(hub.imu.heading() - track_heading)
            else:
                error = rebaseAngle(hub.imu.heading() - initial_heading)
            heading_errors.append(error)
            heading_integral += error
            if len(heading_errors) > HEADING_ERROR_WINDOW_SIZE:
                heading_integral -= db.heading_errors[0]
                heading_errors.pop(0)

            heading_error = (
                error * kP_drive
            )

            if distance > 0:
                if heading_error > 0:
                    left_motor.run((speed_decimal * (1500 - heading_error)) / 1)
                    right_motor.run((speed_decimal * (1500)) / 1)
                else:
                    left_motor.run((speed_decimal * (1500)) / 1)
                    right_motor.run((speed_decimal * (1500 + heading_error)) / 1)
            else:
                if heading_error > 0:
                    left_motor.run((speed_decimal * (-1500)) / 1)
                    db.right_motor.run((speed_decimal * (-1500 + heading_error)) / 1)
                else:
                    left_motor.run((speed_decimal * (-1500 - heading_error)) / 1)
                    right_motor.run((speed_decimal * (-1500)) / 1)

            curr_distance = degToMM(((left_motor.angle() - left_initial_angle)) / 1)
            if abs(curr_distance - last_logged_dist) >= log_interval:
                error_log.append(error/kP_drive)
                distance_log.append(curr_distance)
                last_logged_dist = curr_distance
            heading_errors.clear()
            heading_integral = 0

        
        rebaseIMUAngle()
        left_motor.brake()
        right_motor.brake()
        # print("x_distance =", distance_log)
        # print("error_values =", error_log)

def turn(angle, max_speed=1000):
    """
    Description:
    Helps bot to turn a specific angle with a PD controller and controlled deceleration.

    Inputs:
    angle: angle of turn (degrees)
    max_speed: The maximum speed of the motors during the turn (deg/s)
    deceleration_angle: The angle (in degrees) from the target at which to start slowing down.
    """
    deceleration_angle=angle*(2//10)
    kP_turn = 8.8
    kD_turn = 5

    hub.imu.reset_heading(0)

    last_error = 0
    
    while True:
        current_heading = hub.imu.heading()
        error = angle - current_heading

        if abs(error) < 0.05 and abs(error - last_error) < 0.01:
            break

        derivative = error - last_error
        
        pd_speed = (error * kP_turn) + (derivative * kD_turn)
        
        last_error = error
        
        speed_limit = max_speed

        if abs(error) < deceleration_angle:
            speed_limit = (abs(error) / deceleration_angle) * max_speed

        turn_speed = umath.copysign(min(abs(pd_speed), abs(speed_limit)), pd_speed)
        
        if abs(error) > 1 and abs(turn_speed) < 150:
             turn_speed = umath.copysign(150, turn_speed)

        left_motor.run(turn_speed)
        right_motor.run(-turn_speed)

    left_motor.brake()
    right_motor.brake()

""" For graph for KP values"""

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



""" START MAIN CODE FROM HERE """
# At new locations
# while True:
#     print(sensor.reflection())
# at starting point --> balls picking up and depositing thingy
# assuming +ve angle is right turn
turn(45, 100)
gyroDriveStraight(db, 80)
turn(45)
singleLineTrackByDist(50, Direction="left")
singleLineTrackTillIntersection(Direction="left")
gyroDriveStraight(db, 80)
turn(-90, 100)
singleLineTrackByDist(200)
singleLineTrackTillIntersection()
gyroDriveStraight(db, 80)
turn(-90, 100)
# #claw go down then track to intersection to pick up balls 
# singleLineTrackByDist(40)
# singleLineTrackTillIntersection()
# gyroDriveStraight(db, 80)
# #picked up balls then move back to deposit
# gyroDriveStraight(db, -95)
# turn(180, 100)
# singleLineTrackByDist(40)
# singleLineTrackTillIntersection()
# gyroDriveStraight(db, 50)
# turn(-90, 100)
# singleLineTrackByDist(200)
# singleLineTrackTillIntersection()
# gyroDriveStraight(db, 50)
# turn(-90, 100)
# singleLineTrackByDist(40)
# singleLineTrackTillIntersection()
# # bot reached intersection for deposition, depositing now
# gyroDriveStraight(db, 80)
# #deposited block, going back to push down blue thingy in the middle of the map
# gyroDriveStraight(db, -95)
# turn(180, 100)
# singleLineTrackByDist(40)
# singleLineTrackTillIntersection()
# gyroDriveStraight(db, 50)
# turn(90, 100)
# singleLineTrackByDist(200)
# singleLineTrackTillIntersection()
# gyroDriveStraight(db, 50)
# turn(-90, 100)
# singleLineTrackByDist(40)
# singleLineTrackTillIntersection()
# # reached intersection of blue thingy --> claw down