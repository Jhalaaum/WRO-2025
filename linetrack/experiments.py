from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

HEADING_ERROR_WINDOW_SIZE = 100
kP_drive = 400 # for straight function
kD_drive = 1500
heading_errors = []
heading_integral = 0
Axle_track = 127

hub = PrimeHub()
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B, Direction.CLOCKWISE)
sensor = ColorSensor(Port.D)
claw = Motor(Port.F)
side_sensor = ColorSensor(Port.C)
db = DriveBase(left_motor, right_motor, 62.4, Axle_track)

db.use_gyro(True)
BLACK = 13
WHITE = 97
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
    kP = 4
    kD = 65

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
    current_speed=670,
):
    """
    Tracks a line for a specific distance with controlled acceleration and deceleration.
    """
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    current_distance = 0
    
    if abs(Distance) == 0:
        return

    
    while abs(current_distance) < abs(Distance):

        singleSensorLinetrack(Direction, current_speed)
        
        avg_motor_angle = (left_motor.angle() + right_motor.angle()) / 2
        current_distance = degToMM(avg_motor_angle)

    left_motor.hold()
    right_motor.hold()
def singleLineTrackTillIntersection(Direction="right", baseSpeed=670):
    while sensor.reflection() > 20:
        singleSensorLinetrack(Direction, baseSpeed=baseSpeed)
    left_motor.hold()
    right_motor.hold()
def degToMM(degrees):
    return (degrees / 360) * (62.4 * umath.pi)
def mmToDeg(mm):
    return (mm/(62.4 * umath.pi))*360

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
        max_speed=670,
        final_speed=75,
        accel_dist=100,
        brake_dist=300,
    ):
        global heading_errors, heading_integral, kP_drive, kD_drive
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
        last_error = 0
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
            derivative = error - last_error
            if len(heading_errors) > HEADING_ERROR_WINDOW_SIZE:
                heading_integral -= db.heading_errors[0]
                heading_errors.pop(0)

            heading_error = (
                error * kP_drive + kD_drive*derivative
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
                    right_motor.run((speed_decimal * (-1500 + heading_error)) / 1)
                else:
                    left_motor.run((speed_decimal * (-1500 - heading_error)) / 1)
                    right_motor.run((speed_decimal * (-1500)) / 1)

            curr_distance = degToMM(((left_motor.angle() - left_initial_angle)) / 1)
            if abs(curr_distance - last_logged_dist) >= log_interval:
                # error_log.append(error/kP_drive)
                # distance_log.append(curr_distance)
                last_logged_dist = curr_distance
            heading_errors.clear()
            heading_integral = 0
            last_error=error

        
        rebaseIMUAngle()
        left_motor.hold()
        right_motor.hold()
        # print("x_distance =", distance_log)
        # print("error_values =", error_log)

def turn(angle, max_speed=500):
    """
    Description:
    Helps bot to turn a specific angle with a PD controller and controlled deceleration.

    Inputs:
    angle: angle of turn (degrees)
    max_speed: The maximum speed of the motors during the turn (deg/s)
    deceleration_angle: The angle (in degrees) from the target at which to start slowing down.
    """
    deceleration_angle=angle*0.4
    kP_turn = 17
    kD_turn = 15

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

    left_motor.hold()
    right_motor.hold()
def straightTillIntersection(db, speed=500, track_heading="NA"):
    """
    Drives straight using the full PD controller from gyroDriveStraight until a
    line is detected by the color sensor.

    Args:
        db (DriveBase): The DriveBase object for the robot.
        speed (int): The forward speed for the robot (in deg/s).
        track_heading (int or str): The target heading to maintain. Defaults to 'NA',
                                    which uses the heading at the start of the function.
    """
    kP_drive = 200
    kD_drive = 400

    if track_heading == "NA":
        target_heading = hub.imu.heading()
    else:
        target_heading = track_heading

    last_error = 0
    
    while sensor.reflection() > 20:
        error = rebaseAngle(hub.imu.heading() - target_heading)
        derivative = error - last_error
        
        heading_error = (error * kP_drive) + (derivative * kD_drive)
        
        last_error = error
        if heading_error > 0:
            left_motor.run(speed - heading_error)
            right_motor.run(speed)
        else:
            left_motor.run(speed)
            right_motor.run(speed + heading_error)

        wait(10)
    right_motor.hold()
    left_motor.hold()
def get_detected_color(color_sensor):
    """
    Detects a specific color based on HSV values and returns its name.

    IMPORTANT: You MUST calibrate the HSV ranges below for your environment.
    To calibrate, place a block under the sensor and print the output of
    'color_sensor.hsv()' to find the correct Hue, Saturation, and Value.

    Args:
        color_sensor (ColorSensor): The color sensor to use for detection.

    Returns:
        str: The name of the detected color ("Red", "Yellow", "Green", "White")
             or None if no target color is detected.
    """
    hsv = color_sensor.hsv()
    h, s, v = hsv.h, hsv.s, hsv.v

    if s < 25 and v > 80:
        return "White"
    if (h > 340 or h < 15) and s > 60:
        return "Red"
    if 30 < h < 75 and s > 45:
        return "Yellow"
    if 110 < h < 170 and s > 50:
        return "Green"
    
    return h, s, v
print(get_detected_color(side_sensor))
def scan_and_report_colors(db, color_sensor):
    """
    Scans 6 possible locations in a line to find four colored blocks.

    The robot drives 64mm, takes a reading, and repeats this 6 times.
    At each location, it now takes 10 readings and uses the most common
    color to improve accuracy. It assumes the robot starts with its color
    sensor 64mm before the center of the first location.

    Args:
        db (DriveBase): The robot's drive base.
        color_sensor (ColorSensor): The downward-facing color sensor.

    Returns:
        tuple: A tuple containing two lists:
               - ([green_pos, white_pos], [yellow_pos, red_pos])
               - Positions are 1-6. A value of None means the block was not found.
    """
    detected_locations = {}
    
    print("Starting color scan...")

    for position_index in range(1, 7):
        gyroDriveStraight(db, distance=85)
        
        color_readings = []
        for _ in range(10):
            reading = get_detected_color(color_sensor)
            if reading:
                color_readings.append(reading)
        
        detected_color = None
        if color_readings:
            detected_color = max(set(color_readings), key=color_readings.count)
        
        if detected_color:
            print(f"Position {position_index}: Found {detected_color}!")
            detected_locations[detected_color] = position_index
        else:
            print(f"Position {position_index}: No target block found.")
            
    arrayOfPositions = [None, None]
    red_yellow_positions = [None, None]
    arrayOfPositions[0] = detected_locations.get("Green")
    arrayOfPositions[1] = detected_locations.get("White")

    red_yellow_positions[0] = detected_locations.get("Yellow")
    red_yellow_positions[1] = detected_locations.get("Red")

    print("Scan complete.")
    print(f"Green/White Array: {arrayOfPositions}")
    print(f"Yellow/Red Array: {red_yellow_positions}")

    return (arrayOfPositions, red_yellow_positions)

def oneWheelMotorTurn(
        db,
        angle,
        direction=1,
        hold=True,
        speed=1000
    ):
        rotation_angle = mmToDeg(2 * umath.pi * Axle_track * (abs(angle)/360))

        left_angle = left_motor.angle()
        right_angle = right_motor.angle()

        if angle > 0:
            if direction == 1 or direction == "FORWARD" or direction == "FRONT":
                #self.left_motor.run_target(speed, left_angle+rotation_angle, Stop.HOLD)
                left_motor.run_angle(speed, rotation_angle, Stop.HOLD)
            else:
                #self.right_motor.run_target(speed, left_angle-rotation_angle, Stop.HOLD)
                right_motor.run_angle(speed, -(rotation_angle), Stop.HOLD)
        
        else:
            if direction == 1 or direction == "FORWARD" or direction == "FRONT":
                #self.right_motor.run_target(speed, left_angle+rotation_angle, Stop.HOLD)
                right_motor.run_angle(speed, rotation_angle, Stop.HOLD)
            else:
                #self.left_motor.run_target(speed, left_angle-rotation_angle, Stop.HOLD)
                left_motor.run_angle(speed, -(rotation_angle), Stop.HOLD)
def go_and_pickup(arrayOfPos, green=True):
    """
    Drives to and picks up blocks in the exact order specified in the array.
    This version is optimized to be faster by eliminating unnecessary turns.

    Args:
        arrayOfPos (list): A 0-indexed list with the positions [pos1, pos2, ...].
                           The robot will service the positions in this exact order.
    """
    if green == True:
        target_distances = {
            1: 237.5, 2: 142.5, 3: 47.5,
            4: -47.5,   5: -142.5,   6: -237.5,
        }
    else:
        target_distances = {
        1: -237.5, 2: -142.5, 3: -47.5,
        4: 47.5,   5: 142.5,   6: 237.5,
        }
    
    APPROACH_DISTANCE = 158 

    positions_to_visit = [pos for pos in arrayOfPos if pos is not None]

    if not positions_to_visit:
        print("No blocks were found to pick up.")
        return

    print(f"Starting OPTIMIZED pickup sequence for: {positions_to_visit}")

    current_x_position = 0

    print("Turning parallel to the line...")

    for position in positions_to_visit:
        target_dist = target_distances.get(position)

        distance_to_travel = target_dist - current_x_position
        print(f"Traveling {distance_to_travel:.1f}mm to position {position}...")
        gyroDriveStraight(db, distance_to_travel, max_speed=400)
        current_x_position = target_dist

        if green == True:
            turn(90)
        else:
            turn(-90)
        
        print("Approaching and grabbing block...")
        claw.run_angle(100, 85)
        gyroDriveStraight(db, -APPROACH_DISTANCE)
        claw.run_until_stalled(-300)
        gyroDriveStraight(db, APPROACH_DISTANCE)

        if green == True:
            turn(-90)
        else:
            turn(90)

    print("All blocks collected. Returning to center...")
    return_distance = 0 - current_x_position
    gyroDriveStraight(db, return_distance, max_speed=400)

    if green == True:
        turn(90)
    else:
        turn(-90)
    
    print("Optimized pickup mission complete.")


""" For graph for KP values"""
# while db.distance() < 200:
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
# At new location
# while True:
#     print(sensor.reflection())
# get_detected_color(side_sensor)
# at starting point --> balls picking up and depositing thingy
# assuming +ve angle is right turn
# claw.run_until_stalled(-300)
# gyroDriveStraight(db, 80)
# singleLineTrackByDist(50, Direction="left", current_speed=200)
# singleLineTrackTillIntersection("Direction="left")
# gyroDriveStraight(db, 95)
# turn(-90)
# rebaseIMUAngle()
# wait(100)
# singleLineTrackByDist(100)
# singleLineTrackTillIntersection()
# gyroDriveStraight(db, 85)
# wait(100)
# rebaseIMUAngle()
# turn(90)
# rebaseIMUAngle()
# wait(100)
# claw.run_angle(100, 110)
# gyroDriveStraight(db, -40)
# rebaseIMUAngle()
# wait(100)
# gyroDriveStraight(db, -110)
# claw.run_until_stalled(-300)
# rebaseIMUAngle()
# wait(100)
# gyroDriveStraight(db, 20)
# rebaseIMUAngle()
# wait(100)
# claw.run_until_stalled(300)
# gyroDriveStraight(db, -30)
# rebaseIMUAngle()
# wait(100)
# gyroDriveStraight(db, 210)
# claw.run_angle(100, -20)
# turn(-135)
# gyroDriveStraight(db, 190)
# turn(45)
# rebaseIMUAngle()
# wait(100)
singleLineTrackByDist(200, "left")
singleLineTrackTillIntersection("left", baseSpeed=300)
gyroDriveStraight(db, 140)
turn(90)
rebaseIMUAngle()
straightTillIntersection(db)
gyroDriveStraight(db, 215)
claw.run_until_stalled(-300)
turn(90)
rebaseIMUAngle()
db.straight(-200)
rebaseIMUAngle()
gyroDriveStraight(db, 150)
green_white_array,  yellow_red_array = scan_and_report_colors(db, side_sensor)
gyroDriveStraight(db, 230)
turn(-90)
gyroDriveStraight(db, -900, accel_dist=200, brake_dist=300)
gyroDriveStraight(db, 1000, accel_dist=200, brake_dist=300)
turn(-90)
db.straight(-180)
rebaseIMUAngle()
gyroDriveStraight(db, 90)
straightTillIntersection(db)
gyroDriveStraight(db, 80)
if green_white_array != [None, None]:
    green_white_pickup = []
    green_white_pickup.append(green_white_array[1])
    green_white_pickup.append(green_white_array[0])
else:
    green_white_pickup = [1, 2]
if yellow_red_array != [None, None]:
    yellow_white_pickup = []
    yellow_white_pickup.append(yellow_red_array[1])
    yellow_white_pickup.append(yellow_red_array[0])
else:
    yellow_white_pickup = [3, 4]
go_and_pickup(green_white_pickup)

#deposit blocks 
singleLineTrackByDist(100, "left", current_speed=200)
singleLineTrackTillIntersection("left")
gyroDriveStraight(db, 100)
turn(-90)
gyroDriveStraight(db, 350, accel_dist=50)
turn(90)
straightTillIntersection(db)
singleLineTrackByDist(100, "left", current_speed=300)
#deposit green/white blocks 
singleLineTrackTillIntersection("left", baseSpeed=300)
turn(-155)
gyroDriveStraight(db, -135)
claw.run_angle(100, 82)
gyroDriveStraight(db, 55)
claw.run_angle(100, -82)

gyroDriveStraight(db, 155)
turn(-30)
gyroDriveStraight(db, -205)
claw.run_angle(100, 82)
gyroDriveStraight(db, 100)
claw.run_angle(100, -82)

turn(-45)
gyroDriveStraight(db, 50)
turn(45)
singleLineTrackByDist(100, current_speed=200)
singleLineTrackTillIntersection()

gyroDriveStraight(db, 530)
turn(-90)
rebaseIMUAngle()
db.straight(-250)
rebaseIMUAngle()
straightTillIntersection(db)
gyroDriveStraight(db, 80)
go_and_pickup(yellow_white_pickup, False)

#deposit blocks 
singleLineTrackByDist(100, "right", current_speed=300)
singleLineTrackTillIntersection("right", baseSpeed=300)
turn(90)
gyroDriveStraight(db, 365, max_speed=600, accel_dist=85)
turn(-90)
straightTillIntersection(db)
singleLineTrackByDist(100, "left", current_speed=300)
#deposit yellow/red blocks 
singleLineTrackTillIntersection("left", baseSpeed=300)
turn(-155)
gyroDriveStraight(db, -135)
claw.run_angle(100, 82)
gyroDriveStraight(db, 55)
claw.run_angle(100, -82)

gyroDriveStraight(db, 155)
turn(-30)
gyroDriveStraight(db, -205)
claw.run_angle(100, 82)
gyroDriveStraight(db, 100)
claw.run_angle(100, -82)

turn(-45)
gyroDriveStraight(db, 50)
turn(45)
singleLineTrackByDist(100, current_speed=200)
singleLineTrackTillIntersection()

gyroDriveStraight(db, 500)
turn(90)
db.straight(-250)
rebaseIMUAngle()
straightTillIntersection(db)
gyroDriveStraight(db, 80)
turn(90)
gyroDriveStraight(db, 700)