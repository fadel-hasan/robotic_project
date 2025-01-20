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
Kp = 0.01
Kd = 0.01
Ki = 0.00001
# Last error to be used by the PID.
last_error = 0

#Integral (the accumulation of errors) to be used by the PID.
integral = 0


class RobotController(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)
        self.timestep = int(self.getBasicTimeStep())
        self.robot_name = self.getName()
        print(self.robot_name)
        self.color = list()
        self.color_queue = queue.Queue(4)
        self.rotate_err = 10
        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)
        if self.robot_name == 'youBot(2)':
            self.arm_camera = self.getDevice('arm_camera')
            self.arm_camera.enable(self.timestep)
            self.arm_distance_sensor = self.getDevice('arm_dis_sens')
            self.arm_distance_sensor.enable(self.timestep)
        self.read_array = False
        
        self.plane_colors_1 = ['g','r','b','y']
        self.plane_colors_2 = ['g','y','r','b']
        # توابع لتغير مصفوات الالوان حسب الملف النصي color.txt
        # self.read_colors_from_file()
        self.change_plane_color()
        
        self.sensors = list(map(lambda v: self.getDevice(f"inf{v}"), range(1,9)))
        # self.weights = [-1000,1000,1000,-1000,1000,-1000,-1000,1000]
        self.weights_speed = [-14,14,14,-14,14,-14,-14,14]
        
        self.positions = {
            "start": (0, 0),
            "intersection_red_green": (0.95, 0),
            "red_area": (0.95, 3.6),
            "green_area": (0.95, -3.6),
            "intersection_blue_yellow":(3.95,0),
            "yellow_area":(3.95,3.6),
            "blue_area":(3.95,-3.6),
            "wall": (6.52, 0)
        }
        
        
        self.current_position = "start"
        self.current_direction = 0
        self.pick_count = 0
        self.box_pick_step = 0
        self.fix_left_right_err = 0
        self.distance = 0.12
        for sensor in self.sensors:
            # print("enabled: ", sensor)
            sensor.enable(self.timestep)
            
        self.N_char = [[1,0,1],
                       [1,1,1],
                       [1,0,1]]   
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
        
        self.armMotors = []
        self.armMotors.append(self.getDevice("arm1"))
        self.armMotors.append(self.getDevice("arm2"))
        self.armMotors.append(self.getDevice("arm3"))
        self.armMotors.append(self.getDevice("arm4"))
        self.armMotors.append(self.getDevice("arm5"))
        # Set the maximum motor velocity.
        self.armMotors[0].setVelocity(1.5) # maxVelocity = 1.5
        self.armMotors[1].setVelocity(1.5)
        self.armMotors[2].setVelocity(1.5)
        self.armMotors[3].setVelocity(0.5)
        self.armMotors[4].setVelocity(1.5)
        
        # self.armMotors[0].setPosition(0) # maxVelocity = 1.5
        # self.armMotors[1].setPosition(0)
        # self.armMotors[2].setPosition(0)
        # self.armMotors[3].setPosition(0)
        # self.armMotors[4].setPosition(0)
        
        self.armPositionSensors = []
        self.armPositionSensors.append(self.getDevice("arm1sensor"))
        self.armPositionSensors.append(self.getDevice("arm2sensor"))
        self.armPositionSensors.append(self.getDevice("arm3sensor"))
        self.armPositionSensors.append(self.getDevice("arm4sensor"))
        self.armPositionSensors.append(self.getDevice("arm5sensor"))
        for sensor in self.armPositionSensors:
            sensor.enable(self.timestep)

        self.finger1 = self.getDevice("finger::left")
        self.finger2 = self.getDevice("finger::right")
        self.finger1.setVelocity(1.5)
        self.finger2.setVelocity(1.5) # 0.03
        self.fingerMinPosition = self.finger1.getMinPosition()
        self.fingerMaxPosition = self.finger1.getMaxPosition()
        
        self.dis_arm_sensor = self.getDevice('dis_arm')
        self.dis_arm_sensor.enable(self.timestep)
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
        # print('value',value)
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

    def move_right(self,velocity):
        self.back_left_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(velocity)
        self.front_right_wheel.setVelocity(-velocity)
        
        
        
   
    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
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
    
    def move_distance_left_right(
        self,
        distance,
        velocity = YOUBOT_MAX_VELOCITY
    ):

        rotations = abs(distance) / (2 * math.pi * YOUBOT_WHEEL_RADIUS)
        self.run_motors_for_left_right(
            rotations,
            velocity,
            'left' if distance < 0 else 'right'
        )
        
    
    def run_motors_for_left_right(
        self,
        rotations,
        velocity,
        direction
    ):

        # calculate angle = number_of_rotations / 2 * PI
        angle = rotations * 2 * math.pi
        # print('angle: ',angle)
        print('angle: ', angle)
        curr_front_left = self.f_left_w_sensor.getValue()
        curr_back_left = self.b_right_w_sensor.getValue()

        # print('curr_front_left: ',curr_front_left)
        # print('curr_back_left: ',curr_back_left)
        
        if direction == 'left':
            self.move_left(velocity)
        else:
            self.move_right(velocity)

        while True:
            current_front_left = self.f_left_w_sensor.getValue()
            current_back_left = self.b_right_w_sensor.getValue()
            
            # print('current_front_left: ',current_front_left)
            # print('current_back_left: ',current_back_left)
            
            diff_front_left = current_front_left - curr_front_left
            diff_back_left = current_back_left - curr_back_left

            # print('diff_front_left: ',diff_front_left)
            # print('diff_back_left: ',diff_back_left)
            avg_angle = (diff_front_left + diff_back_left) /2
            # print('angle avg: ',avg_angle )
            if abs(avg_angle) >= angle:
                self.set_motors_velocity(0,0,0,0)
                break
            # print('move')
            # print('sesor live move: ',avg_angle)
            self.step(self.timestep)
    
    def turn_angle(self, angle,clockwise=True,direc = True,velocity = YOUBOT_MAX_VELOCITY):
        """
            A funtion that will turn the robot by specafic angle (in degrees) counterclockwise
        """
        if abs(angle) < 0.001:
            return
        if not direc:
            self.rotate_err = 0
            if clockwise:
                self.current_direction += angle
                velocity = +velocity
            elif not clockwise:
                self.current_direction -= angle
                velocity = -velocity
        else:
            self.rotate_err = 10
            if clockwise:
                self.current_direction += angle
                velocity = +velocity
            elif not clockwise:
                self.current_direction -= angle
                velocity = -velocity
        # print('eerrr rot',self.rotate_err)
        print('angle for rotation now from turn angle function from {}'.format(self.robot_name),angle)
        distance = (2 * math.pi * YOUBOT_RADIUS) / (360 / (abs(angle) + self.rotate_err))
        
        
        self.move_distance_by_motor(
            distance,
            velocity
        )
        if direc:
            self.current_direction = self.current_direction % 360
            # print('curr direc in turn', self.current_direction)


    def move_distance_PID(
        self,
        distance,
        velocity=YOUBOT_MAX_VELOCITY
    ):
    
        rotations = abs(distance) / (2 * math.pi * YOUBOT_WHEEL_RADIUS)

        angle = rotations * 2 * math.pi
        # print('angle:',angle)
        initial_right_sensor = self.f_right_w_sensor.getValue()
        initial_left_sensor = self.f_left_w_sensor.getValue()
        # print('initial_right_sensor:',initial_right_sensor)
        # print('initial_left_sensor:',initial_left_sensor)
        

        while True:
            if distance < 0:
                self.PID_step(-velocity)
            else:
                self.PID_step(velocity)
            current_right_sensor = self.f_right_w_sensor.getValue()
            current_left_sensor = self.f_left_w_sensor.getValue()
            # print('current_right_sensor:',current_right_sensor)
            # print('current_left_sensor:',current_left_sensor)
            right_diff = abs(current_right_sensor - initial_right_sensor)
            left_diff = abs(current_left_sensor - initial_left_sensor)

            avg_diff = (right_diff + left_diff) / 2

            if avg_diff >= angle:
                # if self.current_position == 'wall':
                #     print('distance value',self.dis_arm_sensor.getValue())
                #     if self.dis_arm_sensor.getValue() > 107:
                #         continue
                #     else:
                #         break
                # else:
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
        # print('current direc',self.current_direction)
        angle_diff = target_angle - self.current_direction
        # print('angle_diff befor norm',angle_diff)
        # تطبيع الفرق بين -180 و 180 درجة
        angle_diff = (angle_diff + 180) % 360 - 180
        # print('angle_diff after norm',angle_diff)
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
        self.move_distance_PID(distance,7)
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
        
    def go_to_intersection(self):
        if self.current_position == 'red_area' or self.current_position == 'green_area':
            self.move_to_position("intersection_red_green")
        if self.current_position == 'yellow_area' or self.current_position == 'blue_area':
            self.move_to_position("intersection_blue_yellow")
    
    def go_to_wall(self):
        self.go_to_intersection()
        self.move_to_position("wall")
        
    
        
    def fold_arms(self):
        self.armMotors[0].setPosition(-2.9)
        self.armMotors[1].setPosition(1.5)
        self.armMotors[2].setPosition(-2.6)
        self.armMotors[3].setPosition(1.7)
        self.armMotors[4].setPosition(0)
        
    def stretch_arms(self):
        self.armMotors[0].setPosition(2.9)
        self.armMotors[1].setPosition(-1.0)
        self.armMotors[2].setPosition(2.5)
        self.armMotors[3].setPosition(-1.7)
        self.armMotors[4].setPosition(0)
        
    def pick_up(self,ang = 0):
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[3].setPosition(-0.5)
        self.armMotors[2].setPosition(-1.15)
        self.armMotors[4].setPosition(ang)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        # Monitor the arm joint position to 
        # detect when the motion is completed.
        # print(self.armPositionSensors[0].getValue())
        # print(self.armPositionSensors[1].getValue())
        # print(self.armPositionSensors[2].getValue())
        # print(self.armPositionSensors[3].getValue())
        # print(self.armPositionSensors[4].getValue())
        
        while self.step(self.timestep) != -1:
            # print('arm sensor 1',self.armPositionSensors[0].getValue())
            # print('arm sensor 2',self.armPositionSensors[1].getValue())
            # print('arm sensor 3',self.armPositionSensors[2].getValue())
            # print('arm sensor 4',self.armPositionSensors[3].getValue())
            # print('arm sensor 5',self.armPositionSensors[4].getValue())
            if abs(self.armPositionSensors[3].getValue() - (-0.5)) < 0.01:
                
            # Motion completed.
                break
        self.close_grippers()
        # self.step(50 * self.timestep)    # Wait until the gripper is closed.
        # print('pick')
        self.armMotors[1].setPosition(0)    # Lift arm.
        self.armMotors[2].setPosition(-0.5)
        self.armMotors[3].setPosition(-0.2)
        self.armMotors[4].setPosition(-ang)
        # print('after pick')
        # Wait until the arm is lifted.
        self.step(50 * self.timestep)
        
    def pick_up_r2(self,ang = 0):
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[3].setPosition(-0.5)
        self.armMotors[2].setPosition(-1.15)
        self.armMotors[4].setPosition(ang)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)

        while self.step(self.timestep) != -1:
            if abs(self.armPositionSensors[3].getValue() - (-0.5)) < 0.01:
                
            # Motion completed.
                break
        self.finger1.setPosition(0.001)     # Close gripper.
        self.finger2.setPosition(0.001)
        self.step(50 * self.timestep)    # Wait until the gripper is closed.
        # print('pick')
        self.armMotors[1].setPosition(0)    # Lift arm.
        self.armMotors[2].setPosition(-0.5)
        self.armMotors[3].setPosition(-0.2)
        
        # print('after pick')
        # Wait until the arm is lifted.
        self.step(100 * self.timestep)
        
        
    def hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(100 * self.timestep)
    
    def open_grippers(self):
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(200 * self.timestep)
        
    def close_grippers(self):
        self.finger1.setPosition(0.015/2)
        self.finger2.setPosition(0.015/2)
        self.step(100 * self.timestep)
    
    def place_on_wall(self):
        self.armMotors[2].setPosition(-0.15)
        self.armMotors[3].setPosition(-1.5)
        self.armMotors[4].setPosition(0)
        self.step(300 * self.timestep)
        self.open_grippers()
        
    def place_on_area(self,ang = 0):
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[3].setPosition(-0.5)
        self.armMotors[2].setPosition(-1.15)
        self.armMotors[4].setPosition(ang)
        self.step(200 * self.timestep)
        self.open_grippers()
        self.hand_up()
        
    def wait(self,time = 2500):
        self.step(time * self.timestep)
        
    def pick_From_wall(self):
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        # self.armMotors[1].setPosition(-0.1)
        self.armMotors[2].setPosition(-0.15)
        self.armMotors[3].setPosition(-1.5)
        self.step(300 * self.timestep)
        color_arr = self.arm_camera.getImageArray()
        print(color_arr)
        self.close_grippers()
        self.pick_count+=1
        
    def halt(self):
        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        
    def drop(self):
        self.armMotors[0].setPosition(-2.9)
        self.wait(50)
        self.armMotors[1].setPosition(-0.13)
        self.wait(50)
        self.armMotors[2].setPosition(-1)
        self.wait(100)
        self.armMotors[3].setPosition(-1)
        self.wait(100)
        self.armMotors[2].setPosition(-1.6)
        self.wait(100)
        # self.armMotors[4].setPosition(1.2)
        # self.wait(100)
        self.open_grippers()
        
    def pick_from_drop(self):
        self.armMotors[0].setPosition(-2.9)
        self.wait(50)
        self.armMotors[1].setPosition(-0.17)
        self.wait(100)
        self.armMotors[2].setPosition(-1)
        self.wait(100)
        self.armMotors[3].setPosition(-0.9)
        self.wait(100)
        # self.armMotors[4].setPosition(1.2)
        # self.wait(100)
        self.armMotors[2].setPosition(-1.63)
        self.wait(100)
        # self.armMotors[3].setPosition(-1)
        # self.wait(150)
        # self.armMotors[2].setPosition(-1.63)
        # self.wait(100)
        self.close_grippers()
        
    def from_drop_to_wall(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
        self.wait(300)
        
    def pick_up_cubes(self):
        while True:
            distance = self.dis_arm_sensor.getValue()
            # print(distance)
            if distance < 450:  
                self.pick_up()  
                self.pick_count+=1
            else:
                # self.rotate_err = 0
                for angle in range(0, 40, 1):  
                    self.turn_angle(1, True, False,velocity=2)  
                    distance = self.dis_arm_sensor.getValue()  
                    if distance < 900:
                        dis = self.calculate_move_distance(distance)
                        self.move_distance_PID(distance= dis)
                        # print('dis',distance)
                        # print('norm:',dis)
                        # print('factor',angle/100)
                        self.turn_angle(3, True,False ,velocity=2)
                        factor = angle/70
                        self.pick_up(ang= -factor)
                        self.pick_count+=1
                        self.move_distance_PID(distance= -dis)
                        self.turn_angle(angle + 5,False,False,velocity=2)
                        # print(f"Current Angle: {angle}, Distance: {distance}")
                        return
                else:
                    self.turn_angle(angle + 2,False,False,velocity=2)
                # print('round2')
                for angle in range(0, -40, -1):  
                    self.turn_angle(-1, False,False,velocity=2)  
                    distance = self.dis_arm_sensor.getValue()
                    if distance < 900:
                        dis = self.calculate_move_distance(distance)
                        self.move_distance_PID(distance= dis)
                        # print('dis',distance)
                        # print('norm:',dis)
                        # print('factor',angle/100)
                        self.turn_angle(-3, False, False,velocity=2)
                        factor = angle/70
                        self.pick_up(ang= -factor)
                        self.pick_count+=1
                        self.move_distance_PID(distance= -dis)
                        self.turn_angle(angle - 5,True,False,velocity=2)
                        # print(f"Current Angle: {angle}, Distance: {distance}")
                        return
                else:
                    self.turn_angle(angle - 2,True,False,velocity=2)
            return
    
    def place_up_cubes(self):
        print('box step: ',self.box_pick_step)
        step = self.box_pick_step
        dis = 0.1
        while True:
            distance = self.dis_arm_sensor.getValue()
            # print(distance)
            if distance > 900:  
                self.move_distance_PID(distance= dis)
                self.place_on_area()  
                self.move_distance_PID(distance= -dis)
            else:
                # self.rotate_err = 0
                if self.box_pick_step <4:
                    for angle in range(0, 40, 5):  
                        self.turn_angle(5, True, False,velocity=2)  
                        distance = self.dis_arm_sensor.getValue()  
                        if distance > 900:
                            # dis = self.calculate_place_distance()
                            self.turn_angle(5, True,False ,velocity=2)
                            angle_fix = step
                            print('step: ',step)
                            while step > 1:
                                self.turn_angle(5 + (angle_fix -1)*5 , True,False ,velocity=2)
                                step-=1
                            self.move_distance_PID(distance= dis)
                            # print('dis',distance)
                            # print('norm:',dis)
                            # print('factor',angle/100)
                            factor = angle/70
                            # self.pick_up(ang= -factor)
                            self.place_on_area(ang = -factor)
                            self.move_distance_PID(distance= -dis)
                            self.step(50 * self.timestep)
                            self.turn_angle(angle + (angle_fix * 5) + ((angle_fix -1)*5),False,False,velocity=2)
                            print(f"Current Angle: {angle}, Distance: {distance}")
                            return
                    else:
                        self.turn_angle(angle + 2,False,False,velocity=2)
                else:
                    print('round2')
                    for angle in range(0, -40, -5):  
                        self.turn_angle(-5, False,False,velocity=2)  
                        distance = self.dis_arm_sensor.getValue()
                        if distance > 900:
                            # dis = self.calculate_place_distance()
                            self.turn_angle(-5, False, False,velocity=2)
                            angle_fix = step - 3
                            print('step: ',step)
                            while (step - 3) > 1:
                                self.turn_angle(-5 - (angle_fix -1)*5 , False,False ,velocity=2)
                                step-=1
                            self.move_distance_PID(distance= dis)
                            # print('dis',distance)
                            # print('norm:',dis)
                            # print('factor',angle/100)
                            factor = angle/70
                            self.place_on_area(ang = -factor)
                            self.move_distance_PID(distance= -dis)
                            self.step(50 * self.timestep)
                            self.turn_angle(angle - (angle_fix * 5) - ((angle_fix -1)*5),True,False,velocity=2)
                            print(f"Current Angle: {angle}, Distance: {distance}")
                            return
                    else:
                        self.turn_angle(angle - 2,True,False,velocity=2)
            return

    def calculate_move_distance(self, distance):
        a = 0.000352  
        b = -0.1525   
        move_distance = a * distance + b
        return max(move_distance, 0)
    
    def calculate_place_distance(self):

        move_distances = [0.00272063, 0.0187, 0.0712]
    
        if self.box_pick_step == 1 or self.box_pick_step == 4:
            index = 0
        elif self.box_pick_step == 2 or self.box_pick_step == 5:
            index = 1
        else:
            index = 2
        return move_distances[index]
    
    def move_place_wall(self):
        position_in_set = (self.pick_count - 1) % 4 + 1

        if position_in_set == 1:
            # Cube 1: Do not move
            self.place_on_wall()
            self.hand_up()
        elif position_in_set == 2:
            # Cube 2 or 4: Move left by 0.1 and then back
            self.move_distance_left_right(0.12)
            self.wait(20)
            self.place_on_wall()
            self.hand_up()
            self.move_distance_left_right(-0.12)
            self.wait(20)
        elif position_in_set == 3:
            # Cube 3: Move right by 0.1 and then back
            self.move_distance_left_right(0.23)
            self.wait(20)
            self.place_on_wall()
            self.hand_up()
            self.move_distance_left_right(-0.23)
            self.wait(20)
        elif position_in_set == 4:
            # Cube 4: Move left by 0.1 and then back
            self.move_distance_left_right(-self.distance)
            self.wait(20)
            self.place_on_wall()
            self.hand_up()
            self.move_distance_left_right(self.distance)
            self.wait(20)
        
    def process_colors_r1(self):
        if self.box_pick_step>=1:
            self.wait(3000)
        for color in list(self.color_queue.queue):
            print('color:',color)  
            # color = self.color_queue.get()  
            if color == 'red':
                self.go_to_red_area()
                self.pick_up_cubes()
                self.go_to_wall()
                # self.place_on_wall()
                # self.hand_up()
                self.move_place_wall()
            elif color == 'green':
                self.go_to_green_area()
                self.pick_up_cubes()
                self.go_to_wall()
                # self.place_on_wall()
                # self.hand_up()
                self.move_place_wall()
                
            elif color == 'blue':
                self.go_to_blue_area()
                self.pick_up_cubes()
                self.go_to_wall()
                # self.place_on_wall()
                # self.hand_up()
                self.move_place_wall()
                
            elif color == 'yellow':
                self.go_to_yellow_area()
                self.pick_up_cubes()
                self.go_to_wall()
                # self.place_on_wall()
                # self.hand_up()
                self.move_place_wall()
                
    
    def process_colors_r2(self):
        for color in list(self.color_queue.queue):  
            if color == 'red':
                self.go_to_wall()
                # self.pick_From_wall()
                while True:
                    done = self.search_for_color(color)
                    if done:
                        break
                self.go_to_red_area()
                # self.place_on_area()
                self.place_up_cubes()
            elif color == 'green':
                self.go_to_wall()
                while True:
                    done = self.search_for_color(color)
                    if done:
                        break
                
                # self.pick_From_wall()
                self.go_to_green_area()
                # self.place_on_area()
                self.place_up_cubes()
            elif color == 'blue':
                self.go_to_wall()
                # self.pick_From_wall()
                while True:
                    done = self.search_for_color(color)
                    if done:
                        break
                
                self.go_to_blue_area()
                # self.place_on_area()
                self.place_up_cubes()
            elif color == 'yellow':
                self.go_to_wall()
                # self.pick_From_wall()
                while True:
                    done = self.search_for_color(color)
                    if done:
                        break
                self.go_to_yellow_area()
                # self.place_on_area()
                self.place_up_cubes()
                
    def detect_color(self, camera_data):

        red = camera_data[0][0][0]
        green = camera_data[0][0][1]
        blue = camera_data[0][0][2]

        if red > 200 and green < 100 and blue < 100:
            return "red"
        elif green > 200 and red < 100 and blue < 100:
            return "green"
        elif blue > 200 and red < 100 and green < 100:
            return "blue"
        elif red > 200 and green > 200 and blue < 100:
            return "yellow"
        else:
            return "unknown"
    
    def dynamic_pick_from_wall(self):
        data = self.arm_distance_sensor.getValue()
        self.armMotors[2].setPosition(self.armMotors[2].getTargetPosition() - data/10000)
        self.armMotors[3].setPosition(self.armMotors[3].getTargetPosition() + data/12000)
        self.step(self.timestep * 10)
        if data <= 103:
            self.close_grippers()
            return True
        
    def search_for_color(self, target_color):

        check = 0
        self.armMotors[2].setPosition(0.25)
        self.armMotors[3].setPosition(-1.8)
        self.step(100 * self.timestep)
        self.open_grippers()
        print('fix err',self.fix_left_right_err)
        # تحريك الذراع للبحث عن اللون
        camera_data = self.arm_camera.getImageArray()
        detected_color = self.detect_color(camera_data)
        if detected_color == target_color:
            while self.step(self.timestep) != -1:
                done=self.dynamic_pick_from_wall()    
                if done:
                    self.pick_count+=1
                    self.armMotors[2].setPosition(0)
                    self.armMotors[3].setPosition(-0.5)
                    self.step(100 * self.timestep)
                    return done
        
        while check <= 0.2:
            check+=0.11
            self.move_distance_left_right(0.11,2)
            self.step(self.timestep * 20)
            self.fix_left_right_err+=0.11
            camera_data = self.arm_camera.getImageArray()
            if camera_data:
                detected_color = self.detect_color(camera_data)
                if detected_color == target_color:
                    while self.step(self.timestep) != -1:
                            # self.wait(100)
                            done=self.dynamic_pick_from_wall()
                            if done:
                                self.pick_count+=1
                                self.armMotors[2].setPosition(0)
                                self.armMotors[3].setPosition(-0.5)
                                self.step(100 * self.timestep)
                                self.move_distance_left_right(-check,2)
                                self.step(self.timestep * 50)
                                return done
        else:
            self.move_distance_left_right(-check,2)
            self.fix_left_right_err-=check
            self.step(self.timestep * 50)
            check = 0
        print('check',check)
        while check >= -0.2:
            check-=0.11
            self.move_distance_left_right(-0.11,2)
            self.step(self.timestep * 20)
            self.fix_left_right_err-=0.1
            camera_data = self.arm_camera.getImageArray()
            if camera_data:
                detected_color = self.detect_color(camera_data)
                if detected_color == target_color:
                    while self.step(self.timestep) != -1:
                            # self.wait(100)
                            done=self.dynamic_pick_from_wall()
                            if done:
                                self.pick_count+=1
                                self.armMotors[2].setPosition(0)
                                self.armMotors[3].setPosition(-0.5)
                                self.step(100 * self.timestep)
                                self.move_distance_left_right(-check,2)
                                self.step(self.timestep * 50)
                                return done
        else:
            print('cheak in -',check)
            self.move_distance_left_right(-check,2)
            self.fix_left_right_err+=check
            self.step(self.timestep * 50)
            check = 0
                   
        return False
    
    def choose_colors_box2_r1(self):
        color_list = list(self.color_queue.queue)
        
        # Iterate over the colors in pairs
        for i in range(0, len(color_list), 2):
            if i + 1 < len(color_list):
                # Get the current and next colors
                current_color = color_list[i]
                next_color = color_list[i + 1]
                
                self.pick_up_box2_color(current_color=current_color,next_color=next_color)
    
    def pick_up_box2_color(self, current_color,next_color):
        color_actions = {
            'red': self.go_to_red_area,
            'green': self.go_to_green_area,
            'blue': self.go_to_blue_area,
            'yellow': self.go_to_yellow_area
        }
        
        if next_color in color_actions:
                color_actions[next_color]()
                self.pick_up_cubes()
                self.drop()
                # self.place_on_wall()
                self.hand_up()
                self.go_to_intersection()
                color_actions[current_color]()
                self.pick_up_cubes()
                self.go_to_wall()
                self.pick_count-=1
                self.move_place_wall()
                self.pick_from_drop()
                self.from_drop_to_wall()
                self.pick_count+=1
                self.move_place_wall()

    def choose_colors_box2_r2(self):
        color_list = list(self.color_queue.queue)
        self.wait(18000)
        # Iterate over the colors in pairs
        for i in range(0, len(color_list), 2):
            if i + 1 < len(color_list):
                # Get the current and next colors
                current_color = color_list[i]
                next_color = color_list[i + 1]
                print('enter 2')
                self.pick_up_box2_color_r2(current_color=current_color,next_color=next_color)
    
    
    def pick_up_box2_color_r2(self, current_color,next_color):
        color_actions = {
            'red': self.go_to_red_area,
            'green': self.go_to_green_area,
            'blue': self.go_to_blue_area,
            'yellow': self.go_to_yellow_area
        }
        print('enter 3')
        if next_color in color_actions:
                self.go_to_wall()
                while True:
                    done = self.search_for_color(next_color)
                    if done:
                        break
                self.from_drop_to_wall()
                self.drop()
                self.hand_up()
                while True:
                    done = self.search_for_color(current_color)
                    if done:
                        break
                
                color_actions[current_color]()
                self.place_up_cubes()
                self.go_to_intersection()
                color_actions[next_color]()
                self.pick_from_drop()
                self.from_drop_to_wall()
                self.place_up_cubes()
                
                # self.place_on_wall()
                # self.pick_count-=1
                # self.move_place_wall()
                # # self.pick_count+=1
                # self.move_place_wall()
                
    def process_colors_ch_r1(self):
        if self.box_pick_step>=1:
            self.wait(3000)
        for color in list(self.color_queue.queue):
            print('color:',color)  
            # color = self.color_queue.get()  
            if color == 'red':
                # self.go_to_red_area()
                # self.pick_up_cubes()
                # self.go_to_wall()
                # # self.place_on_wall()
                # # self.hand_up()
                # self.move_place_wall()
                pass
            elif color == 'green':
                for _ in range(7):
                    self.go_to_green_area()
                    self.pick_up_cubes()
                    self.go_to_wall()
                    # self.place_on_wall()
                    # self.hand_up()
                    self.move_place_wall()
                    
            elif color == 'blue':
                self.go_to_blue_area()
                self.pick_up_cubes()
                self.go_to_wall()
                # self.place_on_wall()
                # self.hand_up()
                self.move_place_wall()
                
            elif color == 'yellow':
                self.go_to_yellow_area()
                self.pick_up_cubes()
                self.go_to_wall()
                # self.place_on_wall()
                # self.hand_up()
                self.move_place_wall()
                
                
    def process_colors_ch_r2(self):
        for color in list(self.color_queue.queue):  
            if color == 'red':
                current_position = (0, 0)  # Initialize the starting position
                for _ in range(7):
                    self.go_to_wall()
                    while True:
                        done = self.search_for_color(color)
                        if done:
                            break
                    self.go_to_red_area()
                    
                    # Continue drawing the character from the last position
                    current_position = self.draw_character(self.N_char, *current_position)
                    
                    # If drawing is complete, reset the current_position
                    if current_position is None:
                        current_position = (0, 0)
               
            elif color == 'green':
                current_position = (0, 0)  # Initialize the starting position
                for _ in range(7):
                    self.go_to_wall()
                    while True:
                        done = self.search_for_color(color)
                        if done:
                            break
                    self.go_to_green_area()
                    
                    # Continue drawing the character from the last position
                    current_position = self.draw_character(self.N_char, *current_position)
                    
                    # If drawing is complete, reset the current_position
                    if current_position is None:
                        current_position = (0, 0)
            elif color == 'blue':
                self.go_to_wall()
                # self.pick_From_wall()
                while True:
                    done = self.search_for_color(color)
                    if done:
                        break
                
                self.go_to_blue_area()
                # self.place_on_area()
                self.place_up_cubes()
            elif color == 'yellow':
                self.go_to_wall()
                # self.pick_From_wall()
                while True:
                    done = self.search_for_color(color)
                    if done:
                        break
                self.go_to_yellow_area()
                # self.place_on_area()
                self.place_up_cubes()
                
                
    def move_distance_left_right2(
        self,
        distance,
        direction,
        velocity = YOUBOT_MAX_VELOCITY,      
    ):

        rotations = abs(distance) / (2 * math.pi * YOUBOT_WHEEL_RADIUS)
        self.run_motors_for_left_right2(
            rotations,
            velocity,
            direction
        )
        
    
    def run_motors_for_left_right2(
        self,
        rotations,
        velocity,
        direction
    ):

        # calculate angle = number_of_rotations / 2 * PI
        angle = rotations * 2 * math.pi
        # print('angle: ',angle)
        print('angle: ', angle)
        curr_front_left = self.f_left_w_sensor.getValue()
        curr_back_left = self.b_right_w_sensor.getValue()

        # print('curr_front_left: ',curr_front_left)
        # print('curr_back_left: ',curr_back_left)
        
        if direction == 'left':
            self.move_left(velocity)
        else:
            self.move_right(velocity)

        while True:
            current_front_left = self.f_left_w_sensor.getValue()
            current_back_left = self.b_right_w_sensor.getValue()
            
            # print('current_front_left: ',current_front_left)
            # print('current_back_left: ',current_back_left)
            
            diff_front_left = current_front_left - curr_front_left
            diff_back_left = current_back_left - curr_back_left

            # print('diff_front_left: ',diff_front_left)
            # print('diff_back_left: ',diff_back_left)
            avg_angle = (diff_front_left + diff_back_left) /2
            # print('angle avg: ',avg_angle )
            if abs(avg_angle) >= angle:
                self.set_motors_velocity(0,0,0,0)
                break
            # print('move')
            # print('sesor live move: ',avg_angle)
            self.step(self.timestep)
            
    def place_on_area2(self, ang=0, row =1):
        # Adjust arm position based on the row
        vertical_offset = 0.25 * (2 - row)  # Calculate the vertical offset based on the row
        self.armMotors[1].setPosition(-1.13 - vertical_offset*10)  # Adjust arm height
        self.armMotors[3].setPosition(-1.1 + vertical_offset)
        self.armMotors[2].setPosition(-1.15 + vertical_offset*0.5)  # Adjust arm height
        self.armMotors[4].setPosition(ang)
        self.wait(500)
        self.open_grippers()
        self.hand_up()
        
        
    def draw_character(self, char_array, start_row=0, start_col=0):
        for row_index in range(start_row, len(char_array)):
            for col_index in range(start_col if row_index == start_row else 0, len(char_array[row_index])):
                if char_array[row_index][col_index] == 1:
                    if col_index == 1:  # Center column should not trigger movement
                        self.place_on_area2(row=row_index)
                        return (row_index, col_index + 1)  # Return the next starting position
                    else:
                        # Determine the direction: move "left" for (0, 0) and "right" for (0, 2)
                        direction = "left" if col_index < 1 else "right"
                        direction_2 = "right" if col_index < 1 else "left"
                        print(f"Moving to ({row_index}, {col_index}): distance={0.25}, direction={direction}")
                        self.move_distance_left_right2(0.20, direction, 7)
                        self.place_on_area2(row=row_index)
                        self.move_distance_left_right2(0.20, direction_2, 7)
                        return (row_index, col_index + 1)  # Return the next starting position
                else:
                    print(f"Skipping cell ({row_index}, {col_index})")
        return None  # Indicate completion of the character array
    
    
    def step1(self):
        while(self.step(self.timestep) != -1):
            # # self.go_to_wall()
            if not self.read_array:
                self.read_array_color()            
            else:
                if self.robot_name == 'youBot(2)':
                    self.process_colors_r2()
                    self.box_pick_step+=1
                    # break
                else:    
                    self.process_colors_r1()
                    self.box_pick_step+=1
                    # break
    
    def step2(self):
        while(self.step(self.timestep) != -1):
            # # self.go_to_wall()
            self.distance= 0.11
            if not self.read_array:
                self.read_array_color()            
            else:
                if self.robot_name == 'youBot(2)':
                    self.choose_colors_box2_r2()
                    self.box_pick_step+=1

                    break
                else:    
                    self.choose_colors_box2_r1()
                    self.box_pick_step+=1
                    break
    
    def loop(self):

        while(self.step(self.timestep) != -1):
            if not self.read_array:
                self.read_array_color()            
            else:
                if self.robot_name == 'youBot(2)':
                    self.process_colors_ch_r2()
                    self.box_pick_step+=1
                    break
                else:    
                    self.process_colors_ch_r1()
                    self.box_pick_step+=1
                    break
            



r = RobotController()
# r.step1()
r.step2()
# r.loop()

