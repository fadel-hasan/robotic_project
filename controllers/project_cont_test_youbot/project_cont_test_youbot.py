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
        
        self.read_array = False
        
        self.plane_colors_1 = ['r','g','b','y']
        self.plane_colors_2 = ['r','g','b','y']
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
        
        for sensor in self.sensors:
            # print("enabled: ", sensor)
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
        print('eerrr rot',self.rotate_err)
        print('angle for rotation now from turn angle function',angle)
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
                break

            self.step(self.timestep)

        self.set_motors_velocity(0, 0, 0, 0)
        
    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def calculate_angle(self, pos1, pos2):
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        print('target:', target_angle)
        print('current direc',self.current_direction)
        angle_diff = target_angle - self.current_direction
        print('angle_diff befor norm',angle_diff)
        # تطبيع الفرق بين -180 و 180 درجة
        angle_diff = (angle_diff + 180) % 360 - 180
        print('angle_diff after norm',angle_diff)
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
        
    def go_to_wall(self):
        if self.current_position == 'red_area' or self.current_position == 'green_area':
            self.move_to_position("intersection_red_green")
        if self.current_position == 'yellow_area' or self.current_position == 'blue_area':
            self.move_to_position("intersection_blue_yellow")
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
        self.step(50 * self.timestep)
    
    def open_grippers(self):
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(100 * self.timestep)
        
    def close_grippers(self):
        self.finger1.setPosition(0.001)
        self.finger2.setPosition(0.001)
        self.step(50 * self.timestep)
    
    def place_on_wall(self):
        self.armMotors[2].setPosition(-0.2)
        self.armMotors[3].setPosition(-1.6)
        self.step(150 * self.timestep)
        self.open_grippers()
        
    def place_on_area(self,ang = 0):
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[3].setPosition(-0.5)
        self.armMotors[2].setPosition(-1)
        self.armMotors[4].setPosition(ang)
        self.step(200 * self.timestep)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.hand_up()
        
    def wait(self,time = 2500):
        self.step(time * self.timestep)
        
    def pick_From_wall(self):
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        # self.armMotors[1].setPosition(-0.1)
        self.armMotors[2].setPosition(-0.1)
        self.armMotors[3].setPosition(-1.5)
        self.step(300 * self.timestep)
        self.close_grippers()
        
    def halt(self):
        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)
        
    def drop(self):
        self.armMotors[0].setPosition(-2.9)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(-1)
        self.armMotors[3].setPosition(-1)
        self.armMotors[2].setPosition(-1.7)
        self.armMotors[4].setPosition(2.9)
        
    def pick_up_cubes(self):
        while True:
            distance = self.dis_arm_sensor.getValue()
            print(distance)
            if distance < 450:  
                self.pick_up()  

            else:
                # self.rotate_err = 0
                for angle in range(0, 40, 1):  
                    self.turn_angle(1, True, False,velocity=2)  
                    distance = self.dis_arm_sensor.getValue()  
                    if distance < 900:
                        dis = self.calculate_move_distance(distance)
                        self.move_distance_PID(distance= dis)
                        print('dis',distance)
                        print('norm:',dis)
                        print('factor',angle/100)
                        self.turn_angle(3, True,False ,velocity=2)
                        factor = angle/70
                        self.pick_up(ang= -factor)
                        self.move_distance_PID(distance= -dis)
                        self.turn_angle(angle + 5,False,False,velocity=2)
                        print(f"Current Angle: {angle}, Distance: {distance}")
                        return
                else:
                    self.turn_angle(angle + 2,False,False,velocity=2)
                print('round2')
                for angle in range(0, -40, -1):  
                    self.turn_angle(-1, False,False,velocity=2)  
                    distance = self.dis_arm_sensor.getValue()
                    if distance < 900:
                        dis = self.calculate_move_distance(distance)
                        self.move_distance_PID(distance= dis)
                        print('dis',distance)
                        print('norm:',dis)
                        print('factor',angle/100)
                        self.turn_angle(-3, False, False,velocity=2)
                        factor = angle/70
                        self.pick_up(ang= -factor)
                        self.move_distance_PID(distance= -dis)
                        self.turn_angle(angle - 5,True,False,velocity=2)
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
    
    
    def process_colors_r1(self):
        while not self.color_queue.empty():  
            color = self.color_queue.get()  
            if color == 'red':
                self.go_to_red_area()
                self.pick_up_cubes()
                self.go_to_wall()
                self.place_on_wall()
                self.hand_up()
            elif color == 'green':
                self.go_to_green_area()
                self.pick_up_cubes()
                self.go_to_wall()
                self.place_on_wall()
                self.hand_up()
            elif color == 'blue':
                self.go_to_blue_area()
                self.pick_up_cubes()
                self.go_to_wall()
                self.place_on_wall()
                self.hand_up()
            elif color == 'yellow':
                self.go_to_yellow_area()
                self.pick_up_cubes()
                self.go_to_wall()
                self.place_on_wall()
                self.hand_up()
    
    def process_colors_r2(self):
        while not self.color_queue.empty():  
            color = self.color_queue.get()  
            if color == 'red':
                self.go_to_wall()
                self.wait()
                self.pick_From_wall()
                self.go_to_red_area()
                self.place_on_area()
            elif color == 'green':
                self.go_to_wall()
                self.wait(1000)
                self.pick_From_wall()
                self.go_to_green_area()
                self.place_on_area()
            elif color == 'blue':
                self.go_to_wall()
                self.pick_From_wall()
                self.go_to_blue_area()
                self.place_on_area()
            elif color == 'yellow':
                self.go_to_wall()
                self.pick_From_wall()
                self.go_to_yellow_area()
                self.place_on_area()
                
    def loop(self):

        # self.pick_up_cubes()
        while(self.step(self.timestep) != -1):
            # self.go_to_wall()
            if not self.read_array:
                self.read_array_color()            
            else:
                if self.robot_name == 'youBot(2)':
                    self.process_colors_r2()
                else:    
                    self.process_colors_r1()
                break                
                # self.process_colors()
            #     self.go_to_red_area()
            #     self.halt()
            #     # self.pick_up()
            #     break
            # self.pick_up()
            # self.fold_arms()
            # self.stretch_arms()
            
            # self.hand_up()
            # print(self.dis_arm_sensor.getValue())
            



r = RobotController()
r.loop()
# r.pick_up()

