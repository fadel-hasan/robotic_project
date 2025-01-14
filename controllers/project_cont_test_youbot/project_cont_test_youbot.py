"""project_cont_test_youbot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot

import math
import queue
from controller import Supervisor
import numpy as np
from time import time


YOUBOT_MAX_VELOCITY = 14
YOUBOT_WHEEL_RADIUS =  0.05
LX = 0.228  
LY = 0.158
YOUBOT_RADIUS = LX + LY #math.sqrt((LX**2) + (LY**2))
# YOUBOT_RADUIS = 0.277
# PID Factors
Kp = 0.02
Kd = 0.01
Ki = 0

# Last error to be used by the PID.
last_error = 0

#Integral (the accumulation of errors) to be used by the PID.
integral = 0


class RobotController(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.timestep = int(self.getBasicTimeStep())
        
        self.color = list()
        self.color_queue = queue.Queue(4)
        self.rotate_err = 10
        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)
        
        self.read_array = False
        
        self.plane_colors_1 = ['r','g','b','y']
        self.plane_colors_2 = ['r','g','b','y']
        # توابع لتغير مصفوات الالوان حسب الملف النصي color.txt
        # self.read_colors_from_file()
        # self.change_plane_color()
        
        self.sensors = list(map(lambda v: self.getDevice(f"inf{v}"), range(1,9)))
        # self.weights = [-1000,1000,1000,-1000,1000,-1000,-1000,1000]
        self.weights_speed = [-10,10,10,-10,10,-10,-10,10]
        
        self.positions = {
            "start": (0, 0),
            "intersection_red_green": (0.9, 0),
            "red_area": (0.9, 3.6),
            "green_area": (0.9, -3.6),
            "intersection_blue_yellow":(4,0),
            "yellow_area":(4,3.6),
            "blue_area":(4,-3.6),
            "wall": (6.4, 0)
        }
        
        
        self.current_position = "start"
        self.current_direction = 0
        
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
            
        for index,p in enumerate([f'plane_{i}' for i in range(5,9)]) :
            color = dict_map[self.plane_colors_2[index]]
            self.getFromDef(p).getField("appearance").getSFNode().getField("material").getSFNode().getField("emissiveColor").setSFColor(color)
            
            
    def read_colors_from_file(self):
        try:
            with open("color.txt", "r") as file:
                lines = file.read().strip().splitlines()
                self.plane_colors_1 = lines[0].split(',')
                self.plane_colors_2 = lines[1].split(',')
        except FileNotFoundError:
            pass
        
        
        
    def get_sensors_value(self):
        value = 0

        for index, sensor in enumerate(self.sensors):
            # print(sensor.getValue())
            if(sensor.getValue() > 200):
                value += self.weights_speed[index]
        return value
    
    
    def PID_step(self, velocity = YOUBOT_MAX_VELOCITY):
        global last_error, integral
        value = self.get_sensors_value()
        error = 0 - value
        # print('error',error)
        # Get P term of the PID.
        P = Kp * error
        # print("P",P)
        # Get D term of the PID.
        D = Kd * (last_error - error)
        # print("D",D)
        # Update last_error to be used in the next iteration.
        last_error = error
        # Get I term of the PID.
        I = Ki * integral
        # Update intergral to be used in the next iteration.
        integral += error

        PID = P + D + I
        # print('pid',PID)
        # self.run_motors_stearing(steering,velocity)
        self.run_motors_speed(PID,velocity)
    
    def run_motors_speed(self,control_signal,velocity):
        self.set_motors_velocity(velocity + control_signal, velocity - control_signal, velocity + control_signal, velocity - control_signal)
        
        
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
        if not self.read_array:
            self.PID_step()
            cameraArray = self.camera.getImageArray()
            red = cameraArray[0][0][0]
            green = cameraArray[0][0][1]
            blue = cameraArray[0][0][2]
            if green == 0 and blue == 0 and red != 0: self.color_queue.put('red') if 'red' not in self.color_queue.queue else ...
            if red == 0 and blue == 0 and green != 0: self.color_queue.put('green') if 'green' not in self.color_queue.queue else ...
            if green == 0 and red == 0 and blue != 0 : self.color_queue.put('blue') if 'blue' not in self.color_queue.queue else ...
            if green != 0 and red != 0 and blue == 0 : self.color_queue.put('yellow') if 'yellow' not in self.color_queue.queue else ...
            if self.color_queue.qsize() == 4:
                self.camera.disable()
                self.read_array = True
                self.set_motors_velocity(0,0,0,0)

    def run_motors_for_rotations_by_motor(
        self,
        rotations,
        velocity,
    ):
        """
            A funtion that will run two motors for specafic velocity
            for specafic rotations
            and will depend on right motor sensor to calculate rotations
        """

        # calculate angle = number_of_rotations / 2 * PI
        angle = rotations * 2 * math.pi
        # print('angle: ',angle)
        
        curr_front_left = self.f_left_w_sensor.getValue()
        curr_back_left = self.b_left_w_sensor.getValue()

        self.turn_cw(velocity)

        while True:
            current_front_left = self.f_left_w_sensor.getValue()
            current_back_left = self.b_left_w_sensor.getValue()
            
            
            diff_front_left = current_front_left - curr_front_left
            diff_back_left = current_back_left - curr_back_left

        
            avg_angle = (diff_front_left + diff_back_left) /2
            
            if abs(avg_angle) >= angle:
                break
            # print('move')
            # print('sesor live move: ',avg_angle)
            self.step(self.timestep)

        self.set_motors_velocity(0,0,0,0)
    
    def move_distance_by_motor(
        self,
        distance,
        velocity = YOUBOT_MAX_VELOCITY
    ):
        """
            A funtion that will move the robot by specafic distance,
            with specafic motors velocities,
            and will depend on right motor sensor to calculate the distance
        """

        rotations = distance / (2 * math.pi * YOUBOT_WHEEL_RADIUS)
        self.run_motors_for_rotations_by_motor(
            rotations,
            velocity
        )
    
    def turn_angle(self, angle,clockwise=True,velocity = YOUBOT_MAX_VELOCITY):
        """
            A funtion that will turn the robot by specafic angle (in degrees) counterclockwise
        """
        if abs(angle) < 0.001:
            return
        distance = (2 * math.pi * YOUBOT_RADIUS) / (360 / (abs(angle) + self.rotate_err))
        
        
        if clockwise:
            self.current_direction += angle  
        else:
            self.current_direction -= angle  
            velocity = -velocity
        self.move_distance_by_motor(
            distance,
            velocity
        )
        
        self.current_direction = self.current_direction % 360
        print('curr direc in turn', self.current_direction)


    def move_distance_PID(
        self,
        distance,
        velocity=YOUBOT_MAX_VELOCITY
    ):
    
        rotations = distance / (2 * math.pi * YOUBOT_WHEEL_RADIUS)

        angle = rotations * 2 * math.pi
        # print('angle:',angle)
        initial_right_sensor = self.f_right_w_sensor.getValue()
        initial_left_sensor = self.f_left_w_sensor.getValue()
        # print('initial_right_sensor:',initial_right_sensor)
        # print('initial_left_sensor:',initial_left_sensor)
        

        while True:
            self.PID_step(velocity)
            current_right_sensor = self.f_right_w_sensor.getValue()
            current_left_sensor = self.f_left_w_sensor.getValue()
            # print('current_right_sensor:',current_right_sensor)
            # print('current_left_sensor:',current_left_sensor)
            right_diff = abs(current_right_sensor - initial_right_sensor)
            left_diff = abs(current_left_sensor - initial_left_sensor)

            avg_diff = (right_diff + left_diff) / 2

            if avg_diff >= angle:
                break

            self.step(self.timestep)

        self.set_motors_velocity(0, 0, 0, 0)
        
    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def calculate_angle(self, pos1, pos2):
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        # print('target:', target_angle)
        # print('curr direc',self.current_direction)
        angle_diff = target_angle - self.current_direction
        # print('angle_diff befor norm',angle_diff)
        # تطبيع الفرق بين -180 و 180 درجة
        angle_diff = (angle_diff + 180) % 360 - 180
        print('angle_diff',angle_diff)
        # angle_diff = angle_diff - 15 if abs(angle_diff) > 90 else angle_diff - 10 if abs(angle_diff) == 90 else angle_diff
        return angle_diff#+ (self.rotate_err * angle_diff)
    
    def move_to_position(self, target_position):
        current_pos = self.positions[self.current_position]
        target_pos = self.positions[target_position]

        distance = self.calculate_distance(current_pos, target_pos)
        angle = self.calculate_angle(current_pos, target_pos)
        
        if angle > 0:
            self.turn_angle(abs(angle), clockwise=True,velocity=4)
        else:
            self.turn_angle(abs(angle), clockwise=False,velocity=4)
        self.current_position = target_position
        self.move_distance_PID(distance,4)
        # return distance, angle
    
    def go_to_red_area(self):
        self.move_to_position("intersection_red_green")
        self.move_to_position("red_area")
        
    def go_to_green_area(self):
        self.move_to_position("intersection_red_green")
        self.move_to_position("green_area")
    
    def go_to_yellow_area(self):
        self.move_to_position("intersection_blue_yellow")
        self.move_to_position("yellow_area")
    
    def go_to_blue_area(self):
        self.move_to_position("intersection_blue_yellow")
        self.move_to_position("blue_area")
        
    def go_to_wall(self):
        if self.current_position == 'red_area' or self.current_position == 'green_area':
            self.move_to_position("intersection_red_green")
        if self.current_position == 'yellow_area' or self.current_position == 'blue_area':
            self.move_to_position("intersection_blue_yellow")
        self.move_to_position("wall")
        
    def process_colors(self):
        while not self.color_queue.empty():  
            color = self.color_queue.get()  
            if color == 'red':
                self.go_to_red_area()
                self.go_to_wall()
            elif color == 'green':
                self.go_to_green_area()
                self.go_to_wall()
            elif color == 'blue':
                self.go_to_blue_area()
                self.go_to_wall()
            elif color == 'yellow':
                self.go_to_yellow_area()
                self.go_to_wall()
        
    def loop(self):

        while(self.step(self.timestep) != -1):
            if not self.read_array:
                self.read_array_color()            
            else:
                self.process_colors()



r = RobotController()
r.loop()

