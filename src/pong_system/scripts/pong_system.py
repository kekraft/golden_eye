#!/usr/bin/env python

import roslib; roslib.load_manifest('pong_system')
import rospy

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import copy
import sys

# from Tkinter import *
import Tkinter as tk
import Image as PilImage
import ImageTk # Had to install sudo apt-get install python-imaging-tk

# IS THERE A WAY TO FORCE TARGETTING?
# HOW DO WE STEP IN MANUALLY TO DO TARGETTING?


from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class Pong_System:
    def __init__(self, on_offense = False):
        self.state = Game_State.SETUP
        self.targeted = False
        self.manual = False

        ######## Do calibration routine here ###########
        self.init_calibration()

        # init motors
        self.motor_a = Motor("motor_a")
        self.motor_b = Motor("motor_b")
        self.motor_c = Motor("motor_c")

        # Subscribers to each subsystem state
        self.game_state_sub = rospy.Subscriber('/game/state', String, self.state_cb)
        self.launcher_state_sub = rospy.Subscriber('/launcher/state', String, self.state_cb)
        self.loader_state_sub = rospy.Subscriber('/loader/state', String, self.state_cb)
        self.vision_state_sub = rospy.Subscriber('/vision/state', String, self.state_cb)

        # subscribe to visions output
        self.cup_loc_sub = rospy.Subscriber('/vision/cup_location', Vector3, self.cup_loc_cb)

        # Publishers for each topic that the subsystems listen to
        # Loader cmd (True = Load, False = Do nothing)
        self.loader_pub = rospy.Publisher('/loader/load_cmd', Bool, queue_size=10)

        # launcher velocity and pid commands
        # Vector3.x = motor a speed, Vector3.y = motor b speed, Vector3.z = motor c speed
        self.launcher_motor_vel_pub = rospy.Publisher('/launcher/motor_vel', Vector3, queue_size=10)
        # Vector3.x = kp, Vector3.y = ki, Vector3.z = kd
        self.launcher_pid_pub = rospy.Publisher('/launcher/pid_val', Vector3, queue_size=10)

        # subscribe to game state
        # a true message sent to this topic means we are on offense and should shoot
        # a false message sent to this topic means we are on defense and should do targeting
        self.game_offensive_sub = rospy.Subscriber('/game/offense', Bool, self.game_side_cb)

        ######## Do inital targetting ##################

        ######### Set game state #######################
        if on_offense:
            self.state = Game_State.OFFENSE
        else:
            self.state = Game_State.DEFENSE


    def state_cb(self, msg):
        # print out the data
        print msg.data

        # log the data
        rospy.loginfo(msg)

    def cup_loc_cb(self, msg):
        ''' This is called whenever a new cup position is given by the vision system.
             What happens here is the pixel coords are translated to world coordinates
                and that is transformed to a motor velocity. Each motor velocity is set 
                appropriately
        '''
        self.cup_loc = (msg.x, msg.y, msg.x)
        X = msg.x
        y = msg.y
        z = msg.z

        # transform pixel cup location to world location


        # transform world location to motor velocity

        self.targeted = True

    def game_side_cb(self, msg):
        ''' Reading True means we are on offense.
                If on offense, load a ball
            Reading False means we are on defense.
                If on defense, go ahead and set the motors to 
                 spin at the targeted cup
        '''
        if (msg.data is True) and (self.state is not Game_State.SETUP):
            rospy.loginfo("On offense.")

            if self.targeted is False:
                # force to target the cups
                pass

            # motors' velocities should already be set to hit targeted cup
            # load ball
            self.load()

            self.targeted = False

        else:  

            # target and start motors
            if not self.targeted:
                # force targetting
                pass

            # command motors based on target
            cmd = Vector3()
            cmd.x = self.motor_a.vel
            cmd.y = self.motor_b.vel
            cmd.z = self.motor_c.vel
            self.launcher_motor_vel_pub.publish(cmd)

    def init_calibration(self):
        ''' Initial calibration consists of getting the vision 
            routine up and running.

            We could also input whether or not we are on offense or defense.
        '''
        # add the correct vision stuff to our path
        print "do calibration"

        print "are we on offense or defense?"

    def load(self):
        rospy.loginfo("Sending load cmd")
        cmd = Bool()
        cmd.data = True
        self.loader_pub.publish(cmd)

    def update_motor_speed(self, motor_a_speed, motor_b_speed, motor_c_speed):
        # rospy.loginfo("Updating motor speeds")
        msg = 'Updating Motor Speed values to {0}, {1}, {2}.'.format(motor_a_speed, motor_b_speed, motor_c_speed)
        rospy.loginfo(msg)
        cmd = Vector3()
        cmd.x = motor_a_speed
        cmd.y = motor_b_speed
        cmd.z = motor_c_speed
        self.launcher_motor_vel_pub.publish(cmd)

    def update_pid_values(self, kp, ki, kd):
        msg = 'Updating PID values to {0}, {1}, {2}.'.format(kp, ki, kd)
        rospy.loginfo(msg)
        cmd = Vector3()
        cmd.x = kp
        cmd.y = ki
        cmd.z = kd
        self.launcher_pid_pub.publish(cmd)



class Motor:
    name = "none"
    vel = 0.0
    p = 0.0
    i = 0.0
    d = 0.0

    def __init__(self, name, vel=0.0, p=0.0, i=0.0, d=0.0):
        self.name = name
        self.vel = vel
        self.p = p
        self.i = i
        self.d = d

class Game_State:
    SETUP = 0
    OFFENSE = 1
    DEFENSE = 2


class Application(tk.Frame):
    on_offense = False
    selected = False

    def set_defense(self):
        print "On defense..."
        self.selected = True
        self.on_offense = False
        # self.destroy()
        # self.master.destroy()
        # self.master.quit()

    def set_offense(self):
        print "On offense..."
        self.selected = True
        self.on_offense = True
        # self.destroy()
        # self.master.destroy()
        # self.master.quit()

    def createWidgets(self):
        # self.QUIT = Button(self)
        # self.QUIT["text"] = "QUIT"
        # self.QUIT["fg"]   = "red"
        # self.QUIT["command"] = self.quit
        # self.QUIT.pack({"side": "left"})

        self.offense = tk.Button(self)
        self.offense["text"] = "Offense",
        self.offense["command"] = self.set_offense
        self.offense.pack({"side": "left"})

        self.defense = tk.Button(self)
        self.defense["text"] = "Defense"
        self.defense["command"] = self.set_defense

        self.defense.pack({"side": "left"})

    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        master.minsize(width=250, height=250)
        self.pack()
        self.createWidgets()


class Select_Side():
    on_offense = False
    selected = False
    root = None

    def __init__(self):
        # Creates the window and waits for a selection,
        # returns selection after window is destoyed
        self.create_window()
        print "Selected = ", self.selected
        print "On Offense = ", self.on_offense
        
    def create_window(self):
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.app = Application(master=self.root)
        self.root.app.mainloop()

    def on_closing(self):
        print "destroying"
        self.selected = self.root.app.selected
        self.on_offense = self.root.app.on_offense
        self.root.destroy()

#import sift
class System_GUI():
  def __init__(self, img, pong_system):
    self.root = tk.Tk()
    # tl = Tkinter.Toplevel(root)    

    # cv2.namedWindow("Display Window", cv2.WINDOW_AUTOSIZE)
    # cv2.imshow("Display Window", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    self.pong_system = pong_system

    im = PilImage.fromarray(img)
    self.imgtk = ImageTk.PhotoImage(image=im)

    # insert image into panel
    self.panel = tk.Label(self.root, image = self.imgtk)
    self.panel.pack(side = "top", fill = "both", expand = "yes")

    # Game State Text....need method for updating this to offense and defense
    self.game_state_text = tk.Text(self.root, height=1, width = 8)
    self.game_state_text.config(bg='gray77')
    self.game_state_text.config(fg='black')
    self.game_state_text.pack()
    self.game_state_text.insert(tk.END, " SETUP")
    self.game_state_text.config(state=tk.DISABLED)

    # motor spin boxes section
    motor_text_frame = tk.Frame(self.root)
    motor_text_frame.pack()

    motor_a_text = tk.Text(motor_text_frame, height=1, width=8)
    motor_a_text.pack(side=tk.LEFT, ipadx=20, padx = 10)
    motor_a_text.insert(tk.END, "Motor A:")
    motor_a_text.config(state=tk.DISABLED)
    motor_a_text.configure(bg='gray77')

    motor_b_text = tk.Text(motor_text_frame, height=1, width=8)
    motor_b_text.pack(side=tk.LEFT, ipadx=20, padx=100)
    motor_b_text.insert(tk.END, "Motor B:")
    motor_b_text.config(state=tk.DISABLED)
    motor_b_text.configure(bg='gray77')

    motor_c_text = tk.Text(motor_text_frame, height=1, width=8)
    motor_c_text.pack(side=tk.LEFT, ipadx=20, padx = 10)
    motor_c_text.insert(tk.END, "Motor C:")
    motor_c_text.config(state=tk.DISABLED)
    motor_c_text.configure(bg='gray77')

    motor_box_frame = tk.Frame(self.root)
    motor_box_frame.pack()

    self.motor_a_velocity_box = tk.Spinbox(motor_box_frame, from_=128, to=255, increment=1)
    self.motor_a_velocity_box.pack(ipadx=5, padx=10, pady=10, side=tk.LEFT)

    self.motor_b_velocity_box = tk.Spinbox(motor_box_frame, from_=128, to=255, increment=1)
    self.motor_b_velocity_box.pack(ipadx=5, padx=10, pady=10, side=tk.LEFT)    

    self.motor_c_velocity_box = tk.Spinbox(motor_box_frame, from_=128, to=255, increment=1)
    self.motor_c_velocity_box.pack(ipadx= 5, padx=10, pady=10, side=tk.LEFT)

    # PID value boxes

    pid_text_frame = tk.Frame(self.root)
    pid_text_frame.pack()
    # pid_text_frame.grid(row=0,column=0)
    # text boxes for pid
    kp_text = tk.Text(pid_text_frame, height=1, width=8)
    kp_text.pack(side=tk.LEFT)
    kp_text.insert(tk.END, "P: ")
    kp_text.config(state=tk.DISABLED)
    kp_text.configure(bg='gray77')
    
    ki_text = tk.Text(pid_text_frame, height=1, width=8)
    ki_text.pack(side=tk.LEFT)
    ki_text.insert(tk.END, "I: ")
    ki_text.config(state=tk.DISABLED)
    ki_text.configure(bg='gray77')

    kd_text = tk.Text(pid_text_frame, height=1, width=8)
    kd_text.pack(side=tk.LEFT)
    kd_text.insert(tk.END, "D: ")
    kd_text.config(state=tk.DISABLED)
    kd_text.configure(bg='gray77')


    # frame for pid buttons
    pid_button_frame = tk.Frame(self.root)
    pid_button_frame.pack()
    self.ki_box = tk.Spinbox(pid_button_frame, from_=0, to=10, increment=.1)
    self.ki_box.pack(side=tk.LEFT)

    self.kp_box = tk.Spinbox(pid_button_frame, from_=0, to=10, increment=.1)
    self.kp_box.pack(side=tk.LEFT)

    self.kd_box = tk.Spinbox(pid_button_frame, from_=0, to=10, increment=.1)
    self.kd_box.pack(side=tk.LEFT)




    # What mode are we in, automatic or manual
    self.mode_val = tk.IntVar()
    self.manual_button = tk.Radiobutton(self.root, text="Manual", variable=self.mode_val, value=1, command=self.mode_change)
    self.manual_button.pack()
    self.automatic_button = tk.Radiobutton(self.root, text="Automatic", variable=self.mode_val, value=2, command=self.mode_change)
    self.automatic_button.pack()

    # insert buttons into panel
    # 4 buttons: Fire, update motors, update pid, and Quit
    self.fire_button = tk.Button(self.root, text="Fire")
    self.update_motor_speed_button = tk.Button(self.root, text="Update Motor Values")
    self.update_pid_button = tk.Button(self.root, text="Update PID")
    self.quit_button = tk.Button(self.root, text="Quit")

    self.fire_button.pack(fill=tk.X)
    self.update_motor_speed_button.pack()
    self.update_pid_button.pack()
    self.quit_button.pack(fill=tk.X)

    self.fire_button.bind('<Button-1>', self.fire)
    self.update_motor_speed_button.bind('<Button-1>', self.update_motor_speed)
    self.update_pid_button.bind('<Button-1>', self.update_pid_values)
    self.quit_button.bind('<Button-1>', self.quit)

  def start_gui(self):
    # start the GUI
    self.root.mainloop()

  def fire(self, arg):
    ''' To really tie this in, we need to be able to publish the motor commands'''
    print 'Fire'
    # motor_a_speed = float(self.motor_a_velocity_box.get())
    # print 'Motor a Speed', motor_a_speed

    self.pong_system.load()

  def update_pid_values(self, arg):
    # get values from spin boxes
    kp = float(self.kp_box.get())
    ki = float(self.ki_box.get())
    kd = float(self.kd_box.get())

    # send ros command to controller
    self.pong_system.update_pid_values(kp, ki, kd)

  def update_motor_speed(self, arg):
    # get values from spin boxes
    motor_a_speed = float(self.motor_a_velocity_box.get())
    motor_b_speed = float(self.motor_b_velocity_box.get())
    motor_c_speed = float(self.motor_c_velocity_box.get())

    # Send ros command to controller
    self.pong_system.update_motor_speed(motor_a_speed, motor_b_speed, motor_c_speed)


  def quit(self, arg):
    print 'Quit'

  def mode_change(self):
    print 'Selection changed to: ', self.mode_val.get()

  def update_img(self, open_cv_img):
    # convert image to be friendly with tkinter
    # rearrange color channel
    b,g,r = cv2.split(img)
    img = cv2.merge((r,g,b))

    im = PilImage.fromarray(img)
    imgtk = ImageTk.PhotoImage(image=im)


def main():

    # Have the user dictate whether or not they are on offense
    side = Select_Side()
    selected = side.selected
    assert(selected)
    on_offense = side.on_offense

    rospy.init_node('pong_system')

    pong = Pong_System(on_offense)

    start_system_gui(pong)
    
    # rospy.spin()

    
    

def start_system_gui(pong_system):
    img = cv2.imread("saved.jpg", 1)
    b,g,r = cv2.split(img)
    img = cv2.merge((r,g,b))
    
    gui = System_GUI(img, pong_system)
    gui.start_gui()


if __name__ == '__main__':
    
    main()