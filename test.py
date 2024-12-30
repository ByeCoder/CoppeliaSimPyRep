import tkinter as tk
from tkinter import ttk
from pyrep import PyRep
from pyrep.objects.joint import Joint
import numpy as np
import time
import math
import threading

SF = "test.ttt"

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
    pr.step()
    

def openGripper():
    left_arm.set_joint_target_position(30*math.pi/180)
    right_arm.set_joint_target_position(-30*math.pi/180)

#    setJointTargetPosition(left_arm,30*math.pi/180)
 #   setJointTargetPosition(right_arm,-30*math.pi/180)

def closeGripper():
#    print(-10*math.pi/180)
    left_arm.set_joint_target_position(-10*math.pi/180)
    right_arm.set_joint_target_position(10*math.pi/180)

#setJointTargetPosition(left_arm,-10*math.pi/180)
 #   setJointTargetPosition(right_arm,10*math.pi/180)

def move(v,w):
#    setJointTargetVelocity(left_wheel,(v-b*w)/wheel_radius)
#    setJointTargetVelocity(right_wheel,(v+b*w)/wheel_radius)
    left_wheel.set_joint_target_velocity((v-b*w)/wheel_radius)
    right_wheel.set_joint_target_velocity((v+b*w)/wheel_radius)

#    right_arm.set_joint_target_position(10*math.pi/180)
def moveForward():
#    setJointTargetVelocity(left_wheel,0.5*max_speed/wheel_radius)
 #   setJointTargetVelocity(right_wheel,0.5*max_speed/wheel_radius)
    left_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr.step()
def moveBackward():
#    setJointTargetVelocity(left_wheel,-0.5*max_speed/wheel_radius)
 #   setJointTargetVelocity(right_wheel,-0.5*max_speed/wheel_radius)
    left_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr.step()

def turnLeft():
#    setJointTargetVelocity(left_wheel,-0.5*max_speed/wheel_radius)
 #   setJointTargetVelocity(right_wheel,0.5*max_speed/wheel_radius)
    left_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr.step()

def turnRight():
#    setJointTargetVelocity(left_wheel,0.5*max_speed/wheel_radius)
 #   setJointTargetVelocity(right_wheel,-0.5*max_speed/wheel_radius)
    left_wheel.set_joint_target_velocity(0.5*max_speed/wheel_radius)
    right_wheel.set_joint_target_velocity(-0.5*max_speed/wheel_radius)
    while not stop_thread:
        pr.step()

def stop():
 #   setJointTargetVelocity(left_wheel,0)
#    setJointTargetVelocity(right_wheel,0)
    left_wheel.set_joint_target_velocity(0)
    right_wheel.set_joint_target_velocity(0)
    global stop_thread
    stop_thread = True
    pr.step()

# Create the main application window
root = tk.Tk()
root.title("DYRO Bot")
root.geometry("300x900")
    
# Create a label to show the slider value
value_label = tk.Label(root, text="Robot Speed: 0")
value_label.pack(pady=10)
def update_label():
    value = slider2.get()
    value_label.config(text=f"Robot Speed: {value}")
    onSpeedChange(value)
    while not stop_thread:
        pr.step()
        
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
        pr.step()
        
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

button = tk.Button(root, text="Stop", command=Stopc)
button.pack(pady=10)

# Create a label to display messages when the button is clicked
message_label = tk.Label(root, text="")
message_label.pack(pady=10)
#pr.step()
# Run the Tkinter event loop
root.mainloop()


pr.stop()
pr.shutdown()
