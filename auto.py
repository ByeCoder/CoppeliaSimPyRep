import tkinter as tk
from tkinter import ttk
from pyrep import PyRep
import numpy as np
import math,threading,time
from pyrep.objects import ProximitySensor, Shape
from pyrep.objects.joint import Joint

print("Please Enter File name:\n")
SF = input()
SF = f"{SF}.ttt"

pr = PyRep()
pr.launch(SF,headless=False)
pr.start()
left_arm=Joint('leftjointarm')
right_arm=Joint('rightjointarm')
left_wheel=Joint('leftjointp')
right_wheel=Joint('rightjointp')
wheel_radius=0.03
max_speed=0.1
max_turn=0.3
speed=0
turn=0
b=0.0565
gripper_open=False
stop_thread = False
ir_rear_left = ProximitySensor('ir_rear_left')
ir_front_left = ProximitySensor('ir_front_left')
ir_rear_right = ProximitySensor('ir_rear_right')
ir_front_right = ProximitySensor('ir_front_right')
Wall = Shape('Shape')
sonar = ProximitySensor('Proximity_sensor')

def pr_step():
	pr.step()
	
	
def onSpeedChange(newValue):
    speed=newValue*max_speed/100
    move(speed,turn)

def onTurnChange(newValue):
    turn=newValue*max_turn/100
    move(speed,turn)

def onGripper():
    global gripper_open
    if gripper_open:
        closeGripper()
        gripper_open=False
    else:
        openGripper()
        gripper_open=True
    pr_step()
    

def openGripper():
    left_arm.set_joint_target_position(30*math.pi/180)
    right_arm.set_joint_target_position(-30*math.pi/180)


def closeGripper():
    left_arm.set_joint_target_position(-10*math.pi/180)
    right_arm.set_joint_target_position(10*math.pi/180)


def move(v,w):
    left_wheel.set_joint_target_velocity((v-b*w)/wheel_radius)
    right_wheel.set_joint_target_velocity((v+b*w)/wheel_radius)


def moveForward():
    left_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr_step()
def moveBackward():
    left_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr_step()

def turnLeft():
    left_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr_step()

def turnRight():
    left_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr_step()

def stop():
    left_wheel.set_joint_target_velocity(0)
    right_wheel.set_joint_target_velocity(0)
    global stop_thread
    stop_thread = True
    pr_step()

# Create the main application window
root = tk.Tk()
root.title("DYRO Bot")
root.geometry("300x600")
    
# Create a label to show the slider value
value_label = tk.Label(root, text="Robot Speed: 0")
value_label.pack(pady=10)
def update_label():
    value = slider2.get()
    value_label.config(text=f"Robot Speed: {value}")
    onSpeedChange(value)
    while not stop_thread:
        pr_step()
        
def update_label_async(event):
    global stop_thread
    stop_thread = False
    threading.Thread(target=update_label).start()


# def to update label when slider is released
# Create a slider with a release event to show value after releasing
slider2 = ttk.Scale(root, from_=-100, to=100, orient='horizontal', length=200)
slider2.pack(pady=10)
slider2.bind("<B1-Motion>", update_label_async)



value_label2 = tk.Label(root, text="Robot Turn: 0")
value_label2.pack(pady=10)

def update_label2():
    value = slider.get()
    value_label2.config(text=f"Robot Turn: {value}")
    onTurnChange(value)
    while not stop_thread:
        pr_step()
        
def update_label_async2(event):
    global stop_thread
    stop_thread = False
    threading.Thread(target=update_label2).start()


slider = ttk.Scale(root, from_=-100, to=100, orient='horizontal', length=200)
slider.pack(pady=10)
slider.bind("<B1-Motion>", update_label_async2)
# def to handle button click and display a message

def openclose():
    message_label.config(text=f"Open/Close!")
    onGripper()


def Forwardc():
    global stop_thread
    stop_thread = False
    message_label.config(text=f"Forward!")
    threading.Thread(target=moveForward).start()

def Backwardc():
    global stop_thread
    stop_thread = False
    message_label.config(text=f"Backward!")
    threading.Thread(target=moveBackward).start()
    
def Leftc():
    global stop_thread
    stop_thread = False
    message_label.config(text=f"Left!")
    threading.Thread(target=turnLeft).start()

def Rightc():
    global stop_thread
    stop_thread = False
    message_label.config(text=f"Right!")
    threading.Thread(target=turnRight).start()

def Stopc():
    message_label.config(text=f"Stop!")
    stop()
    
v = 0.5  # Speed
b = 0.1  # Space in wheels
e = 0.25  # Space from wall
k = 1.0  # Static K
R = 0.06  # R of wheel

v_max = v  # Max speed sonar
d_min = 0.18  # Min to stop in m
d_max = 0.3  # Max space to speed in m

def calculate_speed(distance):
    if distance < d_min:
        return 0
    elif distance >= d_min and distance <= d_max:
        return v_max * (distance / d_max)
    else:
        return v_max
        
def Auto():
	left_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
	right_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
	while not stop_thread:
		sonarDetect = sonar.is_detected(Wall)
		sonarDistance = 0.1
		if sonarDetect == True:
			sonarDistance = sonar.read()
		if sonarDistance == -1.0:
			sonarDistance = 0.3
		print(sonarDistance, sonarDetect)
		d_RL = ir_rear_left.read()
		d_FL = ir_front_left.read()
		d_RR = ir_rear_right.read()
		d_FR = ir_front_right.read()
		    # Phi and Walls
		phi_L = math.atan((d_RL - d_FL) / b)
		d_L = (d_FL + d_RL) / 2
		phi_R = math.atan((d_FR - d_RR) / b)
		d_R = (d_FR + d_RR) / 2
		   # Errors and Dist
		phi = (phi_L + phi_R) / 2
		gamma = k * (d_R - d_L)
		   # alpha
		alpha = phi + gamma
		   # Speed
		v_L = v * (math.cos(alpha) + (b / e) * math.sin(alpha))
		v_R = v * (math.cos(alpha) - (b / e) * math.sin(alpha))
		omega_L = v_L / R
		omega_R = v_R / R
		left_wheel.set_joint_target_velocity(omega_L)
		right_wheel.set_joint_target_velocity(omega_R)
		if float(sonarDistance) < 0.2 and sonarDetect == True:
			linear_speed = calculate_speed(sonarDistance)
			left_wheel.set_joint_target_velocity(linear_speed / wheel_radius)
			right_wheel.set_joint_target_velocity(linear_speed / wheel_radius)
		pr_step()
        
def AutoC():
    global stop_thread
    stop_thread = False
    message_label.config(text=f"Forward/Auto Control!")
    threading.Thread(target=Auto).start()

# Create a button
button = tk.Button(root, text="Open/Close", command=openclose)
button.pack(pady=10)

button = tk.Button(root, text="Forward", command=Forwardc)
button.pack(pady=10)

button = tk.Button(root, text="Backward", command=Backwardc)
button.pack(pady=10)

button = tk.Button(root, text="Left", command=Leftc)
button.pack(pady=10)

button = tk.Button(root, text="Right", command=Rightc)
button.pack(pady=10)

button = tk.Button(root, text="Forward|Auto Control", command=AutoC)
button.pack(pady=10)

button = tk.Button(root, text="||||||   Stop   ||||||", command=Stopc)
button.pack(pady=10)

# Create a label to display messages when the button is clicked
message_label = tk.Label(root, text="")
message_label.pack(pady=10)

# Run the Tkinter event loop
root.mainloop()


pr.stop()
pr.shutdown()
