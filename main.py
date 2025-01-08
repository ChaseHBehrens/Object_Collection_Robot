#--------------------------------------------------------------------------------------------------
# Title: Finalv6
# Author: Chase Behrens
# Date: 25 April 2024
#--------------------------------------------------------------------------------------------------

from vex import *
import math

from vex import RotationUnits

brain = Brain()

running = True
main_clock = Timer()

#--------------------------------------------------------------------------------------------------
# Utilities
#--------------------------------------------------------------------------------------------------

def brainprint(word, line=1):
    """ Prints onto the brain screen.

    Parameters:
        word: what is printed
        line: the line to print to
    """
    brain.screen.clear_line(line)
    brain.screen.set_cursor(line,1)
    brain.screen.print(word)


def brainclear(line_range_start=0, line_range_end=0):
    """Clears specified lines of the brain screen.

    Parameters:
        line_range_start: The begining of cleared lines
        line_range_end: The end of cleared lines
    """
    line_range_start = min(max(line_range_start, 0), 32)
    line_range_end = min(max(line_range_end, 0), 32)
    if line_range_start or line_range_end:
        if line_range_start and not line_range_end:
            brain.screen.clear_line(line_range_start)
            return
        if line_range_start > line_range_end:
            line_range_start, line_range_end = line_range_end, line_range_start
        for line in range(line_range_start, line_range_end):
            brain.screen.clear_line(line)
    else:
        brain.screen.clear_screen()


def sin(number):
    """Simplifies calls of the sin function when working in degrees."""
    return math.sin(math.radians(number))


def cos(number):
    """Simplifies calls of the cos function when working in degrees."""
    return math.cos(math.radians(number))


def arctan(x, y):
    """Return arctan results normalized to within 0 to 360 degrees."""
    answer = math.degrees(math.atan2(x, y))
    if answer >= 0:
        return answer
    else:
        return 360 + answer


def sine(number):
    """Determines weather a number is positive or negative."""
    if number > 0:
        return 1
    elif number < 0:
        return -1
    else:
        return 0

#--------------------------------------------------------------------------------------------------
# PID Controller
#--------------------------------------------------------------------------------------------------

class PIDController():
    """A Motor controller.

    Attributes:
        proportional_gain: The proportional gain value.
        derivative_gain: The derivative gain value.
        integral_gain: The integral gain value.
        min_output: The minimum output of the controller.
        max_output: The maximum output of the controller.
    """
    def __init__(self, proportional_gain, derivative_gain, integral_gain, min_output, max_output):
        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.integral_gain = integral_gain
        self.min_output = min_output
        self.max_output = max_output
        self.error = [0, 0]
        self.errorsum = 0
        self.output = 0

    def reset(self):
        """Resets the controller, mainly for clearing intergral value when doing a new action."""
        self.error = [0, 0]
        self.errorsum = 0
        self.output = 0

    def calculate(self, target_value, current_value):
        """Calculates the controller ouput given current and target values.

        Parameters:
            target_value: The value the controller is attempting to reach.
            current_value: The current value.
        """
        if (self.output == self.max_output and self.errorsum > 0) or \
           (self.output == -1 * self.max_output and self.errorsum < 0):
            integral_clamp = 0
        else:
            integral_clamp = 1
        self.error.pop()
        self.error.insert(0, target_value - current_value)
        self.errorsum += self.error[0] * integral_clamp
        self.output = min(self.max_output, max(-1 * self.max_output,
                      (self.proportional_gain * self.error[0]) + \
                      (self.derivative_gain * (self.error[0] - self.error[-1])) + \
                      (integral_clamp * self.integral_gain * self.errorsum) + \
                      (self.min_output * sine(self.error[0]))))
        return self.output

#--------------------------------------------------------------------------------------------------
# Ultrasonic Sensor
#--------------------------------------------------------------------------------------------------

class UltrasonicSensor(Sonar):
    def __init__(self, port):
        super().__init__(port)
    
    def display_distance_reading(self, name, line=1):
        """Prints formated sensor data to the brain."""
        distance = self.distance(INCHES)
        brainprint("{} Distance Reading: {:.2f}".format(name, distance), line)

#--------------------------------------------------------------------------------------------------
# Line Sensor
#--------------------------------------------------------------------------------------------------

class LineSensor(Line):
    def __init__(self, port):
        super().__init__(port)

    def display_reflectivity_reading(self, line=1):
        """Prints formated sensor data to the brain."""
        reflectivity = self.reflectivity()
        brainprint("Reflectivity Reading: {:.2f}".format(reflectivity), line)
        

#--------------------------------------------------------------------------------------------------
# Bump Sensor
#--------------------------------------------------------------------------------------------------

class BumpSensor(Bumper):
    def __init__(self, port):
        super().__init__(port)
    
    def display_bump_reading(self, line=1):
        """Prints formated sensor data to the brain."""
        if self.pressing():
            pressing = "Pressed"
        else:
            pressing = "Not Pressed"
        brainprint("Bump Reading: {}".format(pressing), line)

#--------------------------------------------------------------------------------------------------
# Limit Sensor
#--------------------------------------------------------------------------------------------------

class LimitSensor(Limit):
    def __init__(self, port):
        super().__init__(port)
    
    def display_limit_reading(self, line=1):
        """Prints formated sensor data to the brain."""
        if self.pressing():
            pressing = "Pressed"
        else:
            pressing = "Not Pressed"
        brainprint("Limit Reading: {}".format(pressing), line)

#--------------------------------------------------------------------------------------------------
# Shaft Encoder
#--------------------------------------------------------------------------------------------------

class ShaftEncoder(Encoder):
    def __init__(self, port):
        super().__init__(port)
    
    def display_encoder_reading(self, line=1):
        """Prints formated sensor data to the brain."""
        direction = self.position()
        brainprint("encoder_reading: {}".format(direction), line)

#--------------------------------------------------------------------------------------------------
# Inertial Measurment Unit
#--------------------------------------------------------------------------------------------------

class IMU(Inertial):
    def __init__(self, port):
        super().__init__(port)

    # Rotates heading for more intuative values when initalizing in a certain corner.
    def heading(self, units=RotationUnits.DEG):
        direction = super().heading(units) + 90
        if direction > 360:
            direction -= 360
        return direction
    
    def display_imu_reading(self, line=1):
        """Prints formated sensor data to the brain."""
        heading = self.heading()
        brainprint("Heading Reading: {:.2f}".format(heading), line)

#--------------------------------------------------------------------------------------------------
# Vision Sensor
#--------------------------------------------------------------------------------------------------

class VisionSensor(Vision):
    """Expands Vision class

    Attributes:
        target_position: The coordinate [x, y] in pixels of current target.
        target_lost_frame_count: The number of frames the sensor has not seen the target for.
    """
    def __init__(self, port, brightness, *sigs):
        super().__init__(port, brightness, *sigs)
        self.target_position = None
        self.target_lost_frame_count = 0

    def take_snapshots(self, signatures, number_of_objects_per_type):
        """Runs take_snapshot for multiple signitures."""
        fruits_detected = ()
        for signature in signatures:
            snapshot = self.take_snapshot(signature, number_of_objects_per_type)
            if snapshot:
                snapshot = snapshot
                fruits_detected += snapshot
        return fruits_detected
        
    def largest_object(self, signatures):
        """Finds the larget object given multiple signitures."""
        fruits_detected = self.take_snapshots(signatures, 8)
        fruits_by_size = list(map(lambda x: x.width * x.height, fruits_detected))
        if fruits_detected:
            return fruits_detected[fruits_by_size.index(max(fruits_by_size))]
        else:
            return None
    
    def set_target(self, signatures):
        """Targets the largest object given multiple signitures"""
        target = self.largest_object(signatures)
        if target:
            self.target_position = [target.centerX, target.centerY]
        else:
            self.target_position = None
        
    def update_target_position(self, signatures):
        """Updates target_position based on a new picture"""
        def distance_from_past_target_location(element, target_position):
            """calculates the distance from one point [x, y] to another"""
            threshold = 20
            distance = threshold + 1
            if target_position:
                distance = (((target_position[0] - element.centerX) ** 2) + ((target_position[1] - element.centerY) ** 2)) ** 0.5
            return distance < threshold
        target_within_range = []
        fruits_detected = list(self.take_snapshots(signatures, 8))
        if fruits_detected:
            for fruit in fruits_detected:
                if distance_from_past_target_location(fruit, self.target_position):
                    target_within_range.append(fruit)
        if target_within_range:
            self.target_position = [target_within_range[0].centerX, target_within_range[0].centerY]
            self.target_lost_frame_count = 0
        elif self.target_lost_frame_count > 10:
            self.target_position = None
        else:
            self.target_lost_frame_count += 1


#--------------------------------------------------------------------------------------------------
# Drivetrain
#--------------------------------------------------------------------------------------------------

class Drivetrain():
    """Controles the dirve system of the robot.

    Attributes:
        front_left_motor: The front left drive motor.
        front_right_motor: The front right drive motor.
        back_left_motor: The back left drive motor.
        back_right_motor: The back right drive motor.
        front_left_motor_velocity: The velocity of the font left drive motor in RPM.
        front_right_motor_velocity: The velocity of the front right drive motor in RPM.
        back_left_motor_velocity: The velocity of the front left drive motor in RPM.
        back_rigt_motor_velocity: The velocity of the back right drive motor in RPM.
        encoder: The shaft encoder used to measure the direction the robot is driving in.
        drivetrain_direction_controller: The PI controller used to correct drift.
        imu: The IMU sensor used to measure the direction the robot is facing.
        rotation_velocity: The velocity to rotate the robot.
        rotation_controller: The P controller used to point the robot in a direction.
        x_position: The x position the robot thinks its located at.
        y_position: The y position the robot thinks its located at.
    """
    def __init__(self, front_left_motor : Motor, front_right_motor : Motor, back_left_motor : Motor, back_right_motor : Motor, imu : IMU, encoder : ShaftEncoder):
        self.front_left_motor = front_left_motor
        self.front_right_motor = front_right_motor
        self.back_left_motor = back_left_motor
        self.back_right_motor = back_right_motor
        self.front_left_motor_velocity = 0
        self.front_right_motor_velocity = 0
        self.back_left_motor_velcity = 0
        self.back_right_motor_velocity = 0
        self.encoder = encoder
        self.drive_direction_controller = PIDController(1,0,1,0,10)
        self.imu = imu
        self.rotation_velocity = 0
        self.rotation_controller = PIDController(1, 0, 0, 1, 50)
        self.x_position = 0
        self.y_position = 0

    def update_position(self, direction, velocity):
        """Updates the coordinate positioning based on expected movement.

        Parameters:
            direction: The direction the robot is driving in.
            velocity: the velocity the robot is traveling in.
        """
        velocity /= 50
        self.x_position += sin(direction) * velocity
        self.y_position += cos(direction) * velocity

    def rotate(self, velocity, number_of_degrees):
        """Rotates the robot withough sensing.

        Parameters:
            velocity: The velocity to rotate in RPM.
            number_of_degrees: The distance to rotate in degrees.
        """
        self.front_left_motor.spin_for(FORWARD, 2.8 * number_of_degrees, DEGREES, velocity, RPM, False)
        self.front_right_motor.spin_for(FORWARD, 2.8 * number_of_degrees, DEGREES, velocity, RPM, False)
        self.back_left_motor.spin_for(FORWARD, 2.8 * number_of_degrees, DEGREES, velocity, RPM, False)
        self.back_right_motor.spin_for(FORWARD, 2.8 * number_of_degrees, DEGREES, velocity, RPM, False)
        while self.front_left_motor.is_spinning():
            sleep(50, MSEC)

    def point_in_direction(self, target_direction):
        """Sets rotational velocity to rotates the robot to face in a direction relative to its suroundings using an IMU sensor.

        Parameter:
            target_direction: The direction to rotate to in degrees.
        """
        current_direction = (self.imu.heading() - (360 * math.floor((self.imu.heading() - target_direction + 180) / 360)))
        self.rotation_velocity = self.rotation_controller.calculate(target_direction, current_direction)

    def drive_in_direction_no_correction(self, direction):
        """Sets motor Velocit to drive in a direction relitive to the robot without any drift correction.
        
        Parameter:
            direction: The direction to drive in.
        """
        direction += 45
        velocity = 30
        self.front_left_motor_velocity = velocity * sin(direction)
        self.front_right_motor_velocity = velocity * cos(direction) * -1
        self.back_left_motor_velcity = velocity * cos(direction)
        self.back_right_motor_velocity = velocity * sin(direction) * -1

    # Corrects for drift using a shaft encoder when readings are in a believable range.
    def drive_in_direction(self, velocity, direction):
        """Sets motor velocity to drives in a direction relative to its suroundings using an IMU sensor.
        
        Parameters:
            velocity: The velocity to drive in RPM.
            direction: The direction to drive the robot relative to its suroundings.
        """
        self.update_position(direction, velocity)
        direction -= self.imu.heading()
        current_direction = (self.encoder.position() - (360 * math.floor((self.encoder.position() - direction + 180) / 360)))
        if abs(direction - current_direction) < 20:
            direction -= self.drive_direction_controller.calculate(direction, current_direction)
        direction += 45
        self.front_left_motor_velocity = velocity * sin(direction)
        self.front_right_motor_velocity = velocity * cos(direction) * -1
        self.back_left_motor_velcity = velocity * cos(direction)
        self.back_right_motor_velocity = velocity * sin(direction) * -1

    def drive_to_position(self, velocity, target_position):
        """Sets motor velocity to drive in the apropriate direction to reach a target coordinate
        
        Parameters:
            velocity: The velocity to drive in RPM.
            target_position: the coordinate to dirve to [x, y].
        """
        target_x_position = target_position[0]
        target_y_position = target_position[1]
        direction = arctan((target_x_position - self.x_position), (target_y_position - self.y_position))
        self.drive_in_direction(velocity, direction)
    
    def run(self):
        """Drives the drive motors."""
        self.front_left_motor.spin(FORWARD, self.front_left_motor_velocity + self.rotation_velocity, RPM)
        self.front_right_motor.spin(FORWARD, self.front_right_motor_velocity + self.rotation_velocity , RPM)
        self.back_left_motor.spin(FORWARD, self.back_left_motor_velcity + self.rotation_velocity, RPM)
        self.back_right_motor.spin(FORWARD, self.back_right_motor_velocity + self.rotation_velocity, RPM)

    def stop(self):
        """Stops the drive motors, and retests motor velocities and drive controllers."""
        self.rotation_velocity = 0
        self.front_left_motor_velocity = 0
        self.front_right_motor_velocity = 0
        self.back_left_motor_velcity = 0
        self.back_right_motor_velocity = 0
        self.front_left_motor.stop(BRAKE)
        self.front_right_motor.stop(BRAKE)
        self.back_left_motor.stop(BRAKE)
        self.back_right_motor.stop(BRAKE)
        self.rotation_controller.reset()
        self.drive_direction_controller.reset()

#--------------------------------------------------------------------------------------------------
# Arm
#--------------------------------------------------------------------------------------------------

class Arm():
    """Controles the arm system of the robot.
    
    Attributes:
        arm_motor: The motor that controles elivation of the arm.
        gripper_motor: The motor that controles the gripper.
        grabbing: Weather or not the gripper is currently grabing.
    """
    def __init__(self, arm_motor : Motor, gripper_motor : Motor):
        self.arm_motor = arm_motor
        self.gripper_motor = gripper_motor
        self.grabbing = True

    def grab_fruit(self):
        """Closes the gripper."""
        if not self.grabbing:
            self.gripper_motor.spin(FORWARD, 5, VOLT)
            self.grabbing = True
    
    def release_fruit(self):
        """Opens the gripper."""
        if self.grabbing:
            self.gripper_motor.spin(REVERSE, 3, VOLT)
            self.grabbing = False
            Timer().event(self.gripper_motor.stop, 1000)

    def move_to_position(self, position):
        """Moves the arm to a preset position.
        
        Parameter:
            position: What position to move the arm to.
        """
        if position == 1:
            self.drop()
        if position == 2:
            self.arm_motor.spin_for(FORWARD, 210, DEGREES, True)
        if position == 3:
            self.arm_motor.spin_for(FORWARD, 380, DEGREES, True)

    def drop(self):
        """Drops the arm to its lowest position."""
        self.arm_motor.stop(COAST)

    def stop(self):
        """Holds the arm at its current position."""
        self.arm_motor.stop(BRAKE)

#--------------------------------------------------------------------------------------------------
# State Machine
#--------------------------------------------------------------------------------------------------      

class StateMachine():
    """The main state machine of the robot.

    Parameters:
        event: A container for triggerable events.
        row_number: The current row the robot is attempting to pick.
        empty_rows: A list of rows that the robot did not find a fruit in.
        has_fruit: The signature of the fruit the robot thinks it has.
    """
    def __init__(self):
        self.event = Events()
        self.row_number = -1
        self.empty_rows = []
        self.has_fruit = None
        self.change_state(Start())

    def run(self):
        """Runs the current state and checks if conditions are met to change states."""
        self.state.run()
        self.check_condition()

    def check_condition(self):
        """Checks if state shoud be changed and provides next state."""
        if self.state.substate == "done":
            if type(self.state) == Start:
                self.change_state(MoveingToRow())
                return
            if type(self.state) == MoveingToRow:
                self.change_state(TargetingFruit())
                return
            if type(self.state) == TargetingFruit:
                self.change_state(PickingFruit())
                return
            if type(self.state) == PickingFruit:
                self.has_fruit = self.event.update_has_fruit(self.row_number)
                self.change_state(ReturningToBasket())
                return
            if type(self.state) == ReturningToBasket:
                if self.has_fruit:
                    self.change_state(DropingOffFruit())
                else:
                    self.change_state(ReturningToStart())
                return
            if type(self.state) == DropingOffFruit:
                self.change_state(ReturningToStart())
                return
            if type(self.state) == ReturningToStart:
                self.change_state(Start())
                return

        if self.state.substate == "failed":
            if type(self.state) == TargetingFruit:
                self.empty_rows.append(self.row_number)
                arm.drop()
                self.change_state(ReturningToBasket()) 
                return

    def change_state(self, new_state):
        """Changes the current state to a new state
        
        Parameter:
            new_state: The state to change to.
        """
        if type(new_state) == Start:
            self.has_fruit = None
        if type(new_state) == MoveingToRow:
            if len(self.empty_rows) == 6:
                self.state = End()
            else:
                self.row_number = self.event.increment_row(self.row_number)
                while self.row_number in self.empty_rows:
                    self.row_number = self.event.increment_row(self.row_number)
                self.state = MoveingToRow(self.row_number)
            return
        if type(new_state) == TargetingFruit:
            arm.move_to_position(2)
            self.state = TargetingFruit(self.row_number)
            return
        if type(new_state) == PickingFruit:
            self.state = PickingFruit(self.row_number)
            return
        if type(new_state) == End:
            self.event.end()
        self.state = new_state

#--------------------------------------------------------------------------------------------------
# States
#-------------------------------------------------------------------------------------------------- 

class Start():
    """Initalizes the robot and moves to starting location.
    
    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
    """
    def __init__(self):
        self.event = Events()
        self.substate = "ramming left wall"

    def run(self):
        """Runs the state machine."""
        if self.substate == "ramming left wall":
            drivetrain.drive_in_direction_no_correction(270)
        if self.substate == "ramming front wall":
            drivetrain.drive_in_direction_no_correction(0)
        if self.substate == "driving right":
            drivetrain.point_in_direction(90)
            drivetrain.drive_to_position(50, [-5, -30])
        if self.substate == "driving back":
            drivetrain.point_in_direction(90)
            drivetrain.drive_to_position(50, [-30, -30])
        if self.substate == "turning":
            drivetrain.point_in_direction(180)
        self.check_condition()

    def check_condition(self):
        """Changes state when conditions are met"""
        if self.substate == "ramming left wall":
            if bump_sensor_g.pressing() and bump_sensor_h.pressing():
                drivetrain.stop()
                self.substate = "ramming front wall"
                return
        if self.substate == "ramming front wall":
            if ultrasonic_sensor_e.distance(INCHES) > 3500:
                drivetrain.stop()
                self.event.initialize()
                self.substate = "driving right"
                return
        if self.substate == "driving right":
            if abs(drivetrain.x_position + 5) < 1 and abs(drivetrain.y_position + 30) < 1:
                drivetrain.stop()
                self.substate = "driving back"
                return
        if self.substate == "driving back":
            if abs(drivetrain.x_position + 30) < 1 and abs(drivetrain.y_position + 30) < 1:
                drivetrain.stop()
                drivetrain.rotate(50, 360)
                drivetrain.encoder.set_position(-176)
                self.substate = "turning"
                return
        if self.substate == "turning":
            if abs(drivetrain.imu.heading() - 180) < 0.1:
                drivetrain.stop()
                self.substate = "done"
                return


class MoveingToRow():
    """Moves the robot to the corresponding row.

    Parameter:
        row_number: The current row the robot is navigating to.
    
    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
        destination: The coordinate of the start of the current row.
        direction: The direction the robot should face in the row.
    """
    def __init__(self, row_number=0):
        self.event = Events()
        self.substate = "aligning x"
        self.destination = [[-10, -50],[-10, -120],[-10, -190],[0, 0],[0, 0],[0, 0]][row_number]
        self.direction = [180, 180, 180, 0, 0, 0][row_number]

    def run(self):
        """Runs the state machine."""
        if self.substate == "aligning x":
            drivetrain.point_in_direction(180)
            drivetrain.drive_to_position(50, [self.destination[0], drivetrain.y_position])
        if self.substate == "aligning y":
            drivetrain.point_in_direction(180)
            drivetrain.drive_to_position(50, [drivetrain.x_position, self.destination[1]])
        if self.substate == "turning":
            drivetrain.point_in_direction(self.direction)
        self.check_condition()

    def check_condition(self):
        """Changes state when conditions are met."""
        if self.substate == "aligning x":
            if abs(drivetrain.x_position - self.destination[0]) < 1:
                drivetrain.stop()
                self.substate = "aligning y"
                return
        if self.substate == "aligning y":
            if abs(drivetrain.y_position - self.destination[1]) < 1:
                drivetrain.stop()
                self.substate = "turning"
                return
        if self.substate == "turning":
            if abs(drivetrain.imu.heading() - self.direction) < 0.1:
                drivetrain.stop()
                self.substate = "done"
                return


class TargetingFruit():
    """Finds and aligns with a fruit on a tree.
    
    Parameter:
        row_number: The current row the robot is in.

    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
        destination: The coordinates of the end of the row the robot is on.
        direction: The direction the robot should face in the row.
        color: The signiture of the fruit the robot is looking for.
    """
    def __init__(self, row_number=0):
        self.event = Events()
        self.substate = "turning"
        self.destination = [[-210, -50],[-210, -120],[-210, -190],[0, 0],[0, 0],[0, 0]][row_number]
        self.alignment_point = drivetrain.x_position
        self.direction = [180, 180, 180, 0, 0, 0][row_number]
        self.color = [[Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)],
                      [Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0)],
                      [Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0)],
                      [Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)],
                      [Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0)],
                      [Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0)]][row_number]

    def run(self):
        """Runs the state machine."""
        if self.substate == "turning":
            drivetrain.point_in_direction(self.direction)
        if self.substate == "searching":
            drivetrain.point_in_direction(self.direction)
            drivetrain.drive_to_position(30, self.destination)
            self.alignment_point = drivetrain.x_position
            vision_sensor_7.set_target(self.color)
        if self.substate == "aligning x":
            if vision_sensor_7.target_position:
                drivetrain.point_in_direction(self.direction)
                drivetrain.drive_to_position(30, [self.alignment_point, self.destination[1]])        
        self.check_condition()

    def check_condition(self):
        """Changes state when conditions are met."""
        if self.substate == "turning":
            if abs(drivetrain.imu.heading() - self.direction) < 0.1:
                drivetrain.stop()
                self.substate = "searching"
                return
        if self.substate == "searching":
            if vision_sensor_7.target_position:
                self.substate = "aligning x"
                return
            if abs(self.destination[0] - drivetrain.x_position) < 1 and abs(self.destination[1] - drivetrain.y_position) < 1:
                drivetrain.stop()
                self.substate = "failed"
                return
        if self.substate == "aligning x":
            if not vision_sensor_7.target_position:
                drivetrain.stop()
                self.substate = "searching"
                return
            if vision_sensor_7.target_position:
                if abs(100 - vision_sensor_7.target_position[1]) < 7:
                    drivetrain.stop()
                    arm.drop()
                    sleep(1000, MSEC)
                    self.event.raise_arm(vision_sensor_7.target_position[0])
                    self.substate = "done"
                    return
            if abs(self.alignment_point - drivetrain.x_position) < 1:
                drivetrain.stop()
                vision_sensor_7.update_target_position(self.color)
                if vision_sensor_7.target_position:
                    if vision_sensor_7.target_position[1] < 105:
                        self.alignment_point = drivetrain.x_position - 3
                    else:
                        self.alignment_point = drivetrain.x_position + 3


class PickingFruit():
    """Aproaches and picks a fruit, before returning to the center of its current row.
    
    Parameter:
        row_number: The current row the robot is in.

    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
        destination_y: The y position of the center of the current row.
        direction: The direction the robot should face in the row.
        color: The signiture of the fruit the robot is looking for.
    """
    def __init__(self, row_number=0):
        self.event = Events()
        self.substate = "driving to fruit"
        self.destination_y = [-50, -120, -190, 0, 0, 0][row_number]
        self.direction = [180, 180, 180, 0, 0, 0][row_number]
        self.color = [[Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)],
                      [Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0)],
                      [Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0)],
                      [Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)],
                      [Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0)],
                      [Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0)]][row_number]
    
    def run(self):
        """Runs the state machine."""
        if self.substate == "driving to fruit":
            drivetrain.drive_in_direction(10, self.direction)
        if self.substate == "aligning y":
            drivetrain.drive_to_position(30, [drivetrain.x_position, self.destination_y])
        self.check_condition()

    def check_condition(self):
        """Changes state when conditions are met."""
        if self.substate == "driving to fruit":
            if limit_switch_left.pressing() or limit_switch_right.pressing():
                sleep(500, MSEC)
                drivetrain.stop()
                arm.grab_fruit()
                sleep(1000, MSEC)
                arm.drop()
                self.substate = "aligning y"
                return
        if self.substate == "aligning y":
            if abs(self.destination_y - drivetrain.y_position) < 5:
                drivetrain.stop()
                self.substate = "done"
                return


class ReturningToBasket():
    """Returns to the location of the baskets.

    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
        destination: The coordinates to position in front of the basket.
    """
    def __init__(self):
        self.event = Events()
        self.substate = "aligning x"
        self.destination = [-210, -40]

    def run(self):
        """Runs the state machine."""
        if self.substate == "turning":
            drivetrain.point_in_direction(270)
        if self.substate == "aligning x":
            drivetrain.point_in_direction(270)
            velocity = -1 * PIDController(3, 0, 0, 3, 50).calculate(4.5, ultrasonic_sensor_e.distance(INCHES))
            drivetrain.drive_in_direction(velocity, 270)
        if self.substate == "aligning y":
            drivetrain.point_in_direction(270)
            drivetrain.drive_to_position(50, [drivetrain.x_position, self.destination[1]])
        self.check_condition()

    def check_condition(self):
        """Changes the state when conditions are met"""
        if self.substate == "turning":
            if abs(270 - drivetrain.imu.heading()) < 0.1:
                drivetrain.stop()
                self.substate = "aligning x"
                return
        if self.substate == "aligning x":
            if abs(4.5 - ultrasonic_sensor_e.distance(INCHES)) < 0.1:
                drivetrain.stop()
                drivetrain.x_position = 210
                self.substate = "aligning y"
                return
        if self.substate == "aligning y":
            if abs(drivetrain.y_position - self.destination[1]) < 1:
                drivetrain.stop()
                self.substate = "done"
                return


class DropingOffFruit():
    """Deposites a fruit in a basket.

    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
    """
    def __init__(self):
        self.event = Events()
        self.substate = "turning"

    def run(self):
        """Runs the state machine."""
        if self.substate == "turning":
            drivetrain.point_in_direction(0)
        if self.substate == "locating basket":
            drivetrain.point_in_direction(0)
            drivetrain.drive_in_direction(30, 90)
        self.check_condition()

    def check_condition(self):
        """Changes the state when conditions are met."""
        if self.substate == "turning":
            if abs(drivetrain.imu.heading()) < 0.1:
                drivetrain.stop()
                self.substate = "locating basket"
                return
        if self.substate == "locating basket":
            if ultrasonic_sensor_e.distance(INCHES) < 6:
                drivetrain.stop()
                self.event.align_with_basket()
                arm.release_fruit()
                sleep(1000, MSEC)
                drivetrain.drive_in_direction_no_correction(180)
                drivetrain.run()
                sleep(1000, MSEC)
                drivetrain.stop()
                self.substate = "done"
                return


class ReturningToStart():
    """Returns to the starting corner.
    
    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
    """
    def __init__(self):
        self.event = Events()
        self.substate = "turning"

    def run(self):
        """Runs the state machine."""
        if self.substate == "turning":
            drivetrain.point_in_direction(90)
        if self.substate == "aligning x":
            drivetrain.point_in_direction(90)
            velocity = PIDController(3, 0, 0, 3, 50).calculate(4, ultrasonic_sensor_e.distance(INCHES))
            drivetrain.drive_in_direction(velocity, 270)
        self.check_condition()

    def check_condition(self):
        """Changes state when conditions are met."""
        if self.substate == "turning":
            if abs(90 - drivetrain.imu.heading()) < 0.1:
                drivetrain.stop()
                self.substate = "aligning x"
                return
        if self.substate == "aligning x":
            if abs(4 - ultrasonic_sensor_e.distance(INCHES)) < 0.1:
                drivetrain.stop()
                self.substate = "done"
                return


class End():
    """Finds and aligns with a fruit on a tree.

    Attributes:
        event: A container for triggerable events.
        substate: The current action the robot is executing.
    """
    def __init__(self):
        self.event = Events()
        self.substate = "ending"
    
    def run(self):
        self.event.end()

    def check_condition(self):
        pass

#--------------------------------------------------------------------------------------------------
# Events
#-------------------------------------------------------------------------------------------------- 
     
class Events():
    """Possible events that can be triggered."""
    @staticmethod
    def initialize():
        """Initalizes the robot"""
        arm.release_fruit()
        sleep(1000, MSEC)
        drivetrain.imu.calibrate()
        while drivetrain.imu.is_calibrating():
            sleep(50, MSEC)
        sleep(100, MSEC)
        drivetrain.x_position = 0
        drivetrain.y_position = 0

    @staticmethod
    def increment_row(row_number):
        """Changes the row the robot will search.
        
        Parameter:
            row_number: The row the robot previously checked.
        """
        if row_number == 2:
            row_number = 0
        else:
            row_number += 1
        return row_number

    @staticmethod
    def raise_arm(target_height):
        """Moves the arm to one of three preset position
        
        Parameter:
            target_height: The position of the target fruit in pixels.
        """
        if target_height < 150:  # ≈100
            arm.move_to_position(3)
        if 150 < target_height < 220:  # ≈200
            arm.move_to_position(2)
        if 220 < target_height:  # ≈280
            arm.move_to_position(1)

    @staticmethod
    def update_has_fruit(row_number):
        """Returns the signature of the fruit the robot has.
        
        Parameter: 
            row_number: The row the robot previously checked.
        """
        return [[Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)],
                [Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0)],
                [Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0)],
                [Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)],
                [Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0)],
                [Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0)]][row_number]
    
    @staticmethod
    def align_with_basket():
        """Drives forward until touching a basket."""
        while ultrasonic_sensor_e.distance(INCHES) < 3500:
            drivetrain.drive_in_direction_no_correction(0)
            drivetrain.run()
        drivetrain.stop()

    @staticmethod
    def end():
        """Stops the program"""
        global running
        running = False

#--------------------------------------------------------------------------------------------------
# Initialization
#--------------------------------------------------------------------------------------------------

ultrasonic_sensor_e = UltrasonicSensor(brain.three_wire_port.e)

bump_sensor_g = BumpSensor(brain.three_wire_port.g)
bump_sensor_h = BumpSensor(brain.three_wire_port.h)

drivetrain = Drivetrain(Motor(Ports.PORT1),
                        Motor(Ports.PORT2),
                        Motor(Ports.PORT4),
                        Motor(Ports.PORT3),
                        IMU(Ports.PORT21),
                        ShaftEncoder(brain.three_wire_port.a))

limit_switch_right = LimitSensor(brain.three_wire_port.c)
limit_switch_left = LimitSensor(brain.three_wire_port.d)

arm = Arm(Motor(Ports.PORT5), Motor(Ports.PORT6))

vision_color = {"lemon" : Signature(1, 4055, 4535, 4295,-3915, -3533, -3724,3, 0),
                "lime" : Signature(2, -3479, -2341, -2910,-3543, -2227, -2885,3, 0),
                "grapefruit" : Signature(3, 5221, 6203, 5712,-433, 1, -216,3, 0),
                "orange" : Signature(4, 5449, 8937, 7193,-2923, -2515, -2719,2.5, 0)}
vision_sensor_7 = VisionSensor(Ports.PORT7, 50, *vision_color.values())

main_timer = 0

state_machine = StateMachine()

#--------------------------------------------------------------------------------------------------
# Debugging
#--------------------------------------------------------------------------------------------------

def print_data():
    """Displays sensor data on the robot's brain"""
    brainprint("X Position: {}".format(drivetrain.x_position), 1)
    brainprint("Y Position: {}".format(drivetrain.y_position), 2)
    drivetrain.imu.display_imu_reading(3)
    drivetrain.encoder.display_encoder_reading(4)
    ultrasonic_sensor_e.display_distance_reading("Front", 5)
    bump_sensor_g.display_bump_reading(6)
    bump_sensor_h.display_bump_reading(7)
    limit_switch_left.display_limit_reading(8)
    limit_switch_right.display_limit_reading(9)
    brainprint("Targetted Fruit Position: {}".format(vision_sensor_7.target_position), 10)
    brainprint("Current State: {}".format(type(state_machine.state)), 11)
    brainprint("Current Substate: {}".format(state_machine.state.substate), 12)

#--------------------------------------------------------------------------------------------------
# Main Loop
#--------------------------------------------------------------------------------------------------

main_clock.reset()
while running:
    if main_clock.time() - main_timer >= 50:
        print_data()
        state_machine.run()
        drivetrain.run()
        
        main_timer = main_clock.time()
drivetrain.stop()