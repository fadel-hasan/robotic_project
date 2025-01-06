"""project_cont_test_youbot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot

import math
from controller import Supervisor
import numpy as np
from time import time


YOUBOT_MAX_VELOCITY = 12
YOUBOT_WHEEL_RADUIS =  0.067
YOUBOT_RADUIS = 0.5
# PID Factors
Kp = 0.01
Kd = 0.01
Ki = 0.0001

# Last error to be used by the PID.
last_error = 0

#Integral (the accumulation of errors) to be used by the PID.
integral = 0

def range_conversion(s_start, s_end, d_start, d_end, value):
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ration = abs((value - s_start) / (s_end - s_start))
    if(d_start < d_end):
        return  d_start + abs(d_end - d_start) * ration 
    if(d_start > d_end):
        return  d_start - abs(d_end - d_start) * ration 
    
class RobotController(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.timestep = int(self.getBasicTimeStep())
        
        self.color = list()
        
        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)
        
        self.plane_colors_1 = ['r','g','b','y']
        self.plane_colors_2 = ['r','g','b','y']
        
        self.sensors = list(map(lambda v: self.getDevice(f"inf{v}"), range(1,5)))
        self.weights = [-1000,1000,1000,-1000]

        for sensor in self.sensors:
            print("enabled: ", sensor)
            sensor.enable(self.timestep)
            
            
        self.f_right_w_sensor = self.getDevice("wheel1sensor")
        self.f_left_w_sensor = self.getDevice("wheel2sensor")
        self.b_right_w_sensor = self.getDevice("wheel3sensor")
        self.b_left_w_sensor = self.getDevice("wheel4sensor")
        
        

        # enabling sensors
        self.f_right_w_sensor.enable(self.timestep)
        self.f_left_w_sensor.enable(self.timestep)
        self.b_right_w_sensor.enable(self.timestep)
        self.b_left_w_sensor.enable(self.timestep)
        
        
        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")
        
        self.front_right_wheel.setPosition(float("inf"))
        self.front_left_wheel.setPosition(float("inf"))
        self.back_right_wheel.setPosition(float("inf"))
        self.back_left_wheel.setPosition(float("inf"))

        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        # plane_node = self.getFromDef("plane_1")
        # color_field = plane_node.getField("appearance").getSFNode().getField("material").getSFNode().getField("emissiveColor").getSFColor()
        # print(color_field)
        # self.change_plane_color()
        # print(color_field)
        # color_field.setSFColor([1, 0, 0]) 
        # print(color_field)
        # self.getRoot().getField("children")
        # root_children = self.getRoot().getField("children")
        # for i in range(root_children.getCount()):
        #     child_node = root_children.getMFNode(i)
        #     print(f"العقدة الفرعية {i}: {child_node.getTypeName()}")
        self.step()

    def change_plane_color(self):
        # plane_node = self.getFromDef("plane_5")
        dict_map = {'r':[1, 0, 0],
                    'g':[0, 1, 0],
                    'b':[0, 0, 1],
                    'y':[1, 1, 0]}
        
        for index,p in enumerate([f'plane_{i}' for i in range(1,5)]) :
            color = dict_map[self.plane_colors_1[index]]
            self.getFromDef(p).getField("appearance").getSFNode().getField("material").getSFNode().getField("emissiveColor").setSFColor(color)
            
    
    def get_sensors_value(self):
        value = 0

        for index, sensor in enumerate(self.sensors):
            print(sensor.getValue())
            if(sensor.getValue() > 200):
                value += self.weights[index]

        return value
    
    
    def PID_step(self, velocity = YOUBOT_MAX_VELOCITY):
        global last_error, integral
        value = self.get_sensors_value()
        error = 0 - value
        # Get P term of the PID.
        P = Kp * error
        # Get D term of the PID.
        D = Kd * (last_error - error)
        # Update last_error to be used in the next iteration.
        last_error = error
        # Get I term of the PID.
        I = Ki * integral
        # Update intergral to be used in the next iteration.
        integral += error

        steering = P + D + I
        self.run_motors_stearing(steering,velocity)
    
    def run_motors_stearing(self, stearing, velocity = YOUBOT_MAX_VELOCITY):
        """
        A function that is responsible for the steering functionality for the motor
        Steering value:
            - from -100 to 0 will turn left
            - from 0 to 100 will turn right
            - if equals 100 will turn the robot around it self to the right
            - if equals -100 will turn the robot around it self to the left
            - if equals 50 will turn the robot around right wheel to the right
            - if equals -50 will turn the robot around left wheel to the left
        """
        front_right_velocity = velocity if stearing > 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        front_left_velocity = velocity if stearing < 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        back_right_velocity = velocity if stearing < 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        back_left_velocity = velocity if stearing > 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        
        self.set_motors_velocity(front_right_velocity,front_left_velocity,back_right_velocity,back_left_velocity)
        
    def set_motors_velocity(self, wheel1_v, wheel2_v, wheel3_v, wheel4_v):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)

    def move_forward(self, velocity):
        self.set_motors_velocity(velocity, velocity, velocity, velocity)

    def move_backward(self, velocity):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)

    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)
        
    def turn_cw(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)
        
    def read_array_color(self):
        cameraArray = self.camera.getImageArray()
        red = cameraArray[0][0][0]
        green = cameraArray[0][0][1]
        blue = cameraArray[0][0][2]
        if green == 0 and blue == 0 and red != 0: self.color.append('red') if 'red' not in self.color else ...
        if red == 0 and blue == 0 and green != 0: self.color.append('green') if 'green' not in self.color else ...
        if green == 0 and red == 0 and blue != 0 : self.color.append('blue') if 'blue' not in self.color else ...
        if green != 0 and red != 0 and blue == 0 : self.color.append('yellow') if 'yellow' not in self.color else ...
        if len(r.color) == 4:self.camera.disable()
    
    def run_motors_for_rotations_by_right_motor(
        self,
        rotations,
        left_velocity,
        right_velocity
    ):
        """
            A funtion that will run two motors for specafic velocity
            for specafic rotations
            and will depend on right motor sensor to calculate rotations
        """

        # calculate angle = number_of_rotations / 2 * PI
        angle = rotations * 2 * math.pi
        print('angle: ',angle)
        # get first value of the sensor
        curr = self.b_left_w_sensor.getValue()
        print('curr: ',curr)
        print('sesor live: ',self.b_left_w_sensor.getValue())
        # set motors velocities
        self.turn_cw(right_velocity)

        # do stepping until the differance between initial sensor value
        # and current value is less than the required angle
        while(self.b_left_w_sensor.getValue() - curr < angle):
            print('move')
            print('sesor live move: ',self.b_left_w_sensor.getValue())
            self.step(self.timestep)

        # reset motors velocities to zero
        self.set_motors_velocity(0,0,0,0)
    
    def move_distance_by_right_motor(
        self,
        distance,
        left_velocity = YOUBOT_MAX_VELOCITY,
        right_velocity = YOUBOT_MAX_VELOCITY
    ):
        """
            A funtion that will move the robot by specafic distance,
            with specafic motors velocities,
            and will depend on right motor sensor to calculate the distance
        """

        rotations = distance / (2 * math.pi * YOUBOT_WHEEL_RADUIS)
        self.run_motors_for_rotations_by_right_motor(
            rotations,
            left_velocity,
            right_velocity
        )
    
    def turn_angle(self, angle):
        """
            A funtion that will turn the robot by specafic angle (in degrees) counterclockwise
        """
        distance = (2 * math.pi * YOUBOT_RADUIS) / (360 / angle)
        self.move_distance_by_right_motor(
            distance,
            -2,
            2
        )
    def loop(self):
        while(self.step(self.timestep) != -1):
            # pass
            # print(dir(self.camera))
            # self.move_forward(2)
            # self.PID_step(2)
            self.turn_angle(98)
            break
            # self.read_array_color() if len(r.color) != 4 else ...


r = RobotController()
r.loop()
