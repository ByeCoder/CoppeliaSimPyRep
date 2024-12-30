import tkinter as tk
from tkinter import ttk
from pyrep import PyRep
import numpy as np
import math, threading, time, random,sys
from pyrep.backend import sim
from pyrep.objects import ProximitySensor, Shape, VisionSensor, Dummy
from pyrep.objects.joint import Joint


print("Please Enter File name:\n")
SF = input()
SF = f"{SF}.ttt"

pr = PyRep()
pr.launch(SF, headless=False)
pr.start()


try:
    left_wheel = Joint('leftjointp')
    right_wheel = Joint('rightjointp')
except Exception as e:
    print(f"Error initializing joints: {e}")
    pr.stop()
    pr.shutdown()
    exit()

wheel_radius = 0.03
robot_width = 0.2 
speed = 0.1
gain_p = 0.1
gain_d = 0.5
stop_thread = False


try:
    ir_rear_left = ProximitySensor('ir_rear_left')
    ir_front_left = ProximitySensor('ir_front_left')
    ir_rear_right = ProximitySensor('ir_rear_right')
    ir_front_right = ProximitySensor('ir_front_right')
    sonar = ProximitySensor('Proximity_sensor')

except Exception as e:
    print(f"Error initializing sensors: {e}")
    pr.stop()
    pr.shutdown()
    exit()





robot_pose = Dummy('robot_pose')
target_dummy = Dummy('ref_point')

def pr_step():
	pr.step()
	
def updateRobotPose():
    position = robot_pose.get_position()
    orientation = robot_pose.get_orientation()
    print(f"ROBOT POSE: Position = {position}, Orientation = {orientation}")
    pose = [position[0], position[1], orientation[2]]
    return pose


def getTrajectoryPoint():
    position = target_dummy.get_position()
    print(f"REF POSE: Position = {position}")
    orientation = target_dummy.get_orientation()
    print(f"REF OR: Orientation = {orientation}")
    linear_vel, angular_vel = target_dummy.get_velocity()
    print(f"LinearVel: {linear_vel}, AngularVel: {angular_vel}")
    ptraj = [0, 0, 0]
    if orientation[2] > 0:
        ptraj = [position[0], position[1], orientation[2] - math.pi / 2]
    else:
        ptraj = [position[0], position[1], math.pi / 2 - orientation[2]]
    vtraj = [linear_vel[0], linear_vel[1], angular_vel[2]]
    return ptraj, vtraj


def calculate_control():
    robot_position = np.array(robot_pose.get_position())[:2]
    target_position = np.array(target_dummy.get_position())[:2]


    error_position = target_position - robot_position
    distance_error = np.linalg.norm(error_position)
    desired_angle = math.atan2(error_position[1], error_position[0])

    robot_orientation = robot_pose.get_orientation()[2]
    angle_error = desired_angle - robot_orientation


    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi


    random_deviation = random.uniform(-math.pi / 6, math.pi / 6) 
    desired_angle += random_deviation 


    angle_error = desired_angle - robot_orientation


    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi


    linear_velocity = gain_p * distance_error
    angular_velocity = gain_d * angle_error


    left_velocity = (linear_velocity - (angular_velocity * robot_width / 2)) / wheel_radius * speed
    right_velocity = (linear_velocity + (angular_velocity * robot_width / 2)) / wheel_radius * speed


    print(f"Robot Position: {robot_position}, Target Position: {target_position}")
    print(f"Distance Error: {distance_error}, Angle Error: {angle_error}")
    print(f"Left Wheel Velocity: {left_velocity}, Right Wheel Velocity: {right_velocity}")
    
    return left_velocity, right_velocity




    
def update_target_position(threshold=0.05):

    robot_position = np.array(robot_pose.get_position())[:2]
    target_position = np.array(target_dummy.get_position())[:2]


    if np.linalg.norm(robot_position - target_position) < threshold:
        left_wheel.set_joint_target_velocity(0)
        right_wheel.set_joint_target_velocity(0)
        print(f"Reach the goal")


    return True


vref= 0.1
desired_distance = 0.1  # فاصله مطلوب از دیوار (مثلاً 20 سانتی‌متر)
k_distance = 0.3  # ضریب تناسبی برای کنترل فاصله
k_angle = 1.0  # ضریب تناسبی برای کنترل زاویه
v_base = 0.1  # سرعت پایه ربات

def update_robot():
	TTT = 0
	LT = False
	LASTT = 1

	left_wheel.set_joint_target_velocity(0.5)
	right_wheel.set_joint_target_velocity(0.5)
	oldleft = 1
	oldright = 1
	search_mode = False
	XT = 0
	R = 0
	CASE = "D"
	while not stop_thread:
		CASE = "D"
		sonarDistance = 0.1
		try:
			sonarDistance = sonar.read()
		except: pass
		if sonarDistance == -1.0:
			sonarDistance = 0.3
		if sonarDistance < 0.2:
			CASE = "S"
			print("Sonar:", sonarDistance)
		d_RL = ir_rear_left.read()
		d_FL = ir_front_left.read()
		if d_RL == -1.0:
			d_RL = 0.9
		if d_FL == -1.0:
			d_FL = 0.9
		print("DRL",d_RL,"DFL",d_FL)
		if d_RL < 0.15 and d_FL < 0.15:
			CASE = "R"
					
		if CASE == "R":
			print("Turning")
			oldleft = 0.03/ wheel_radius
			oldright = 0.0015/ wheel_radius
			left_wheel.set_joint_target_velocity(oldleft)
			right_wheel.set_joint_target_velocity(oldright)
			for _ in range(15): 
                           pr_step()
			#pr_step()
			CASE = "D"
		elif CASE == "S":
			CASE = "D"
			oldleft = 0.009/ wheel_radius
			oldright = 0.0015/ wheel_radius
			left_wheel.set_joint_target_velocity(oldleft)
			right_wheel.set_joint_target_velocity(oldright)
		elif CASE == "I":
			CASE = "D"
			b = 0.1
			phi = math.atan((d_RL - d_FL) / b)
			d_L = (d_FL + d_RL) / 2
			gamma = k_distance * (desired_distance - d_L)
			omega = -k_angle * phi + gamma
			v_left = v_base - b * omega
			v_right = v_base + b * omega
			left_wheel.set_joint_target_velocity(v_left / 0.1)
			right_wheel.set_joint_target_velocity(v_right / 0.1)
		elif CASE == "D":
			left_velocity, right_velocity = calculate_control()
			left_wheel.set_joint_target_velocity(left_velocity)
			right_wheel.set_joint_target_velocity(right_velocity)
			current_target = update_target_position()
			print(f"Robot Moving: Left Wheel Velocity = {left_velocity}, Right Wheel Velocity = {right_velocity}")
			#time.sleep(0.1)  
		pr_step()



def stop():
    left_wheel.set_joint_target_velocity(0)
    right_wheel.set_joint_target_velocity(0)
    global stop_thread
    stop_thread = True
    print("Robot Movement Stopped")
    pr.step()


def start_movement():
    global stop_thread
    stop_thread = False
    print("Robot Movement Started")
    message_label.config(text=f"Robot Moving to Target!")
    threading.Thread(target=update_robot, daemon=True).start()


root = tk.Tk()
root.title("DYRO Bot")
root.geometry("300x300")


start_button = tk.Button(root, text="Start Movement", command=start_movement)
start_button.pack(pady=10)

stop_button = tk.Button(root, text="Stop", command=stop)
stop_button.pack(pady=10)


message_label = tk.Label(root, text="")
message_label.pack(pady=10)


root.mainloop()


pr.stop()
pr.shutdown()

