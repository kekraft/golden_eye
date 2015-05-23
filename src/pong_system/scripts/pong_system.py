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
import time

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

# import vision helper functions
import os
directory = os.path.realpath(__file__)
# print directory
vision_direct = os.path.join(directory, '../../../pong_vision/src')
vision_direct = os.path.abspath(vision_direct)
# print vision_direct
sys.path.append(vision_direct)

from vision_helper import *

# path debugging
# try:
#     user_paths = os.environ['PYTHONPATH'].split(os.pathsep)
#     for path in user_paths:
#         print path
#         print
# except KeyError:
#     user_paths = []

class Pong_System:
    def __init__(self, on_offense = False):
        self.state = Game_State.SETUP
        self.targeted = False
        self.manual = False

        self.img = None

        ######## Do calibration routine here ###########
        self.init_calibration()

        # init motors
        self.motor_a = Motor("motor_a")
        self.motor_b = Motor("motor_b")
        self.motor_c = Motor("motor_c")
        self.motor_a.pwm = 0
        self.motor_b.pwm = 0
        self.motor_c.pwm = 0
        self.motor_a.speed = 0
        self.motor_b.speed = 0
        self.motor_c.speed = 0

        # Subscribers to each subsystem state
        self.game_state_sub = rospy.Subscriber('/game/state', String, self.state_cb)
        self.launcher_state_sub = rospy.Subscriber('/launcher/state', String, self.state_cb)
        self.loader_state_sub = rospy.Subscriber('/loader/state', String, self.state_cb)
        self.vision_state_sub = rospy.Subscriber('/vision/state', String, self.state_cb)

        # subscribe to visions output and raw feed (for the gui)
        self.cup_loc_sub = rospy.Subscriber('/vision/cup_location', Vector3, self.cup_loc_cb)
        self.image_feed_sub = rospy.Subscriber('/image_raw', Image, self.image_cb)

        # Publishers for each topic that the subsystems listen to
        # Loader cmd (True = Load, False = Do nothing)
        self.loader_pub = rospy.Publisher('/loader/load_cmd', Bool, queue_size=10)

        # launcher velocity and pid commands
        # Vector3.x = motor a speed, Vector3.y = motor b speed, Vector3.z = motor c speed
        self.launcher_motor_vel_pub = rospy.Publisher('/launcher/motor_vel', Vector3, queue_size=10)
        # Vector3.x = kp, Vector3.y = ki, Vector3.z = kd
        self.launcher_pid_pub = rospy.Publisher('/launcher/pid_val', Vector3, queue_size=10)

        # subscribe to speed output of the launcher
        self.launcher_speed_sub = rospy.Subscriber('/launcher/speed', Vector3, self.speed_cb)

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

        ## Make sure mototrs start at correct speeds
        self.update_motor_speed(self.motor_a.pwm, self.motor_b.pwm, self.motor_c.pwm)


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

            Only updates if the system is in automatic mode
        '''
        if not self.manual:
            self.cup_loc = (msg.x, msg.y, msg.x)
            X = msg.x
            y = msg.y
            z = msg.z

            # transform pixel cup location to world location


            # transform world location to motor velocity

            # set this to be the pixel we target

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

    def speed_cb(self, msg):
        self.motor_a.speed = msg.x
        self.motor_b.speed = msg.y
        self.motor_c.speed = msg.z

    def update_pid_values(self, kp, ki, kd):
        msg = 'Updating PID values to {0}, {1}, {2}.'.format(kp, ki, kd)
        rospy.loginfo(msg)
        cmd = Vector3()
        cmd.x = kp
        cmd.y = ki
        cmd.z = kd
        self.launcher_pid_pub.publish(cmd)

    def image_cb(self, img_msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        # cv2.imshow('Live Feed: Raw Image', cv_image)
        # cv2.waitKey(1)
        self.img = cv_image

    def target_pixel_location(self, x, y):
        print "Set up method to target pixel (x,y)"

    def shutdown(self, msg):
        rospy.signal_shutdown(msg)



class Motor:
    name = "none"
    speed = 0.0
    pwm = 0
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
    self.manual_mode = True
    self.game_state = Game_State.SETUP

    # vision helper class
    self.vision_helper = Vision_Helper()

    self.root = tk.Tk()
    self.root.resizable(0,0)
    self.root.wm_title("GoldenEye v0.1")
    # tl = Tkinter.Toplevel(root)    

    # cv2.namedWindow("Display Window", cv2.WINDOW_AUTOSIZE)
    # cv2.imshow("Display Window", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    self.pong_system = pong_system

    im = PilImage.fromarray(img)
    self.imgtk = ImageTk.PhotoImage(image=im)

    # insert image into panel
    self.img_panel = tk.Label(self.root, image = self.imgtk)
    self.img_panel.pack(side = "top", fill = "both", expand = "no")
    self.img_panel.bind("<Button-1>", self.img_clicked_button)

    # output position relative to clicks
    self.click_panel = tk.Frame(self.root)
    self.click_panel.pack()

    self.pixel_label = tk.Label(self.click_panel)
    self.pixel_label.grid(row=0,column=0)
    self.pixel_label.configure(text = "Pixel")
    self.pixel_value_label = tk.Label(self.click_panel)
    self.pixel_value_label.grid(row=0,column=1)

    self.dist_label = tk.Label(self.click_panel)
    self.dist_label.grid(row=0,column=2)
    self.dist_label.configure(text = "Distance")
    self.dist_value_label = tk.Label(self.click_panel)
    self.dist_value_label.grid(row=0,column=3)

    self.lat_label = tk.Label(self.click_panel)
    self.lat_label.grid(row=0,column=4)
    self.lat_label.configure(text = "Lateral")
    self.lat_value_label = tk.Label(self.click_panel)
    self.lat_value_label.grid(row=0,column=5)

    # Game State Text....need method for updating this to offense and defense
    self.game_state_panel = tk.Frame(self.root)
    self.game_state_panel.pack(pady = 10)

    self.game_setup_text = tk.Text(self.game_state_panel, height=1, width = 8)
    self.game_setup_text.config(bg='green2')
    self.game_setup_text.config(fg='black')
    self.game_setup_text.grid(row=0,column=0)
    self.game_setup_text.insert(tk.END, " Setup")
    self.game_setup_text.config(state=tk.DISABLED)
    self.game_setup_text.bind('<Button-1>', self.setup_state)

    self.game_offense_text = tk.Text(self.game_state_panel, height=1, width = 10)
    self.game_offense_text.config(bg='gray77')
    self.game_offense_text.config(fg='black')
    self.game_offense_text.grid(row=0,column=1)
    self.game_offense_text.insert(tk.END, " Offense")
    self.game_offense_text.config(state=tk.DISABLED)
    self.game_offense_text.bind('<Button-1>', self.offense_state)

    self.game_defense_text = tk.Text(self.game_state_panel, height=1, width = 10)
    self.game_defense_text.config(bg='gray77')
    self.game_defense_text.config(fg='black')
    self.game_defense_text.grid(row=0, column=2)
    self.game_defense_text.insert(tk.END, " Defense")
    self.game_defense_text.config(state=tk.DISABLED)
    self.game_defense_text.bind('<Button-1>', self.defense_state)

    ####### Panel to control distance of frame and table ########
    self.vision_panel = tk.Frame(self.root)
    self.vision_panel.pack(pady=10)

    self.dist_at_top_label = tk.Label(self.vision_panel)
    self.dist_at_top_label.grid(row=0, column=0)
    self.dist_at_top_label.configure(text = "Top Distance")

    self.dist_at_bot_label = tk.Label(self.vision_panel)
    self.dist_at_bot_label.grid(row=0, column=1)
    self.dist_at_bot_label.configure(text = "Bottom Distance")

    self.table_width_label = tk.Label(self.vision_panel)
    self.table_width_label.grid(row=0, column=2)
    self.table_width_label.configure(text = "Table Width")

    self.dist_at_top_entry = tk.Spinbox(self.vision_panel, from_=0, to=10, increment=1)
    self.dist_at_top_entry.grid(row=1, column=0)
    self.dist_at_top_entry.config(width = 18)

    self.dist_at_bot_entry = tk.Spinbox(self.vision_panel, from_=0, to=10, increment=1)
    self.dist_at_bot_entry.grid(row=1, column=1)
    self.dist_at_bot_entry.config(width = 18)

    self.table_width_entry = tk.Spinbox(self.vision_panel, from_=0, to=10, increment=1)
    self.table_width_entry.grid(row=1, column=2)
    self.table_width_entry.config(width = 18)

    ##### Text entries for current motor speeds #######
    self.cur_speed_panel = tk.Frame(self.root)
    self.cur_speed_panel.pack(pady=10)

    self.cur_speed_a_text = tk.Text(self.cur_speed_panel, height=1, width = 8)
    self.cur_speed_a_text.grid(row=0, column=0)
    self.cur_speed_a_text.insert(tk.END, " A: Cur")
    self.cur_speed_a_text.config(state=tk.DISABLED)
    
    self.cur_speed_b_text = tk.Text(self.cur_speed_panel, height=1, width = 8)
    self.cur_speed_b_text.grid(row=0, column=1)
    self.cur_speed_b_text.insert(tk.END, " B: Cur")
    self.cur_speed_b_text.config(state=tk.DISABLED)
        
    self.cur_speed_c_text = tk.Text(self.cur_speed_panel, height=1, width = 8)
    self.cur_speed_c_text.grid(row=0, column=2)
    self.cur_speed_c_text.insert(tk.END, " C: Cur")
    self.cur_speed_c_text.config(state=tk.DISABLED)

    self.cur_speed_a_val = tk.Label(self.cur_speed_panel, height=1, width = 8)
    self.cur_speed_a_val.grid(row=1, column=0)
    # self.cur_speed_a_val.insert(tk.END, "")
    
    self.cur_speed_b_val = tk.Label(self.cur_speed_panel, height=1, width = 8)
    self.cur_speed_b_val.grid(row=1, column=1)
    # self.cur_speed_b_val.insert(tk.END, "")
        
    self.cur_speed_c_val = tk.Label(self.cur_speed_panel, height=1, width = 8)
    self.cur_speed_c_val.grid(row=1, column=2)
    # self.cur_speed_c_val.insert(tk.END, "")

    # motor spin boxes section
    motor_text_frame = tk.Frame(self.root)
    motor_text_frame.pack(pady=10)

    motor_a_text = tk.Text(motor_text_frame, height=1, width=22)
    # motor_a_text.pack(side=tk.LEFT, ipadx=20, padx = 10)
    motor_a_text.grid(row=0, column=0)
    motor_a_text.insert(tk.END, "Motor A")
    motor_a_text.config(state=tk.DISABLED)
    motor_a_text.configure(bg='gray77')
    # motor_a_text.configure(justify = tk.CENTER)

    motor_b_text = tk.Text(motor_text_frame, height=1, width=22)
    # motor_b_text.pack(side=tk.LEFT, ipadx=20, padx=100)
    motor_b_text.grid(row=0, column=1)
    motor_b_text.insert(tk.END, "Motor B")
    motor_b_text.config(state=tk.DISABLED)
    motor_b_text.configure(bg='gray77')
    # motor_b_text.configure(justify = tk.CENTER)

    motor_c_text = tk.Text(motor_text_frame, height=1, width=22)
    # motor_c_text.pack(side=tk.LEFT, ipadx=20, padx = 10)
    motor_c_text.grid(row=0, column=2)
    motor_c_text.insert(tk.END, "Motor C")
    motor_c_text.config(state=tk.DISABLED)
    motor_c_text.configure(bg='gray77')
    # motor_c_text.configure(justify = tk.CENTER)

    # motor_box_frame = tk.Frame(self.root)
    # motor_box_frame.pack()

    self.motor_a_velocity_box = tk.Spinbox(motor_text_frame, from_=0, to=255, increment=1)
    # self.motor_a_velocity_box.pack(ipadx=5, padx=10, pady=10, side=tk.LEFT)
    self.motor_a_velocity_box.grid(row=1, column=0)
    self.motor_a_velocity_box.configure(width=18)

    self.motor_b_velocity_box = tk.Spinbox(motor_text_frame, from_=0, to=255, increment=1)
    # self.motor_b_velocity_box.pack(ipadx=5, padx=10, pady=10, side=tk.LEFT)    
    self.motor_b_velocity_box.grid(row=1, column=1)
    self.motor_b_velocity_box.configure(width=18)

    self.motor_c_velocity_box = tk.Spinbox(motor_text_frame, from_=0, to=255, increment=1)
    # self.motor_c_velocity_box.pack(ipadx= 5, padx=10, pady=10, side=tk.LEFT)
    self.motor_c_velocity_box.grid(row=1, column=2)
    self.motor_c_velocity_box.configure(width=18)

    # Setup area to control
    # forward velocity, top and right spin
    relative_motor_control_frame = tk.Frame(self.root)
    relative_motor_control_frame.pack(pady=10)

    forward_velocity_text = tk.Text(relative_motor_control_frame, height=1, width=22)
    forward_velocity_text.grid(row=0, column=0)    
    forward_velocity_text.insert(tk.END, "Forward Velocity")
    forward_velocity_text.config(state=tk.DISABLED)
    forward_velocity_text.configure(bg='gray77')
    # forward_velocity_text.config(justify=tk.CENTER)

    right_spin_text = tk.Text(relative_motor_control_frame, height=1, width=22)
    right_spin_text.grid(row=0, column=1)    
    right_spin_text.insert(tk.END, "Right Spin")
    right_spin_text.config(state=tk.DISABLED)
    right_spin_text.configure(bg='gray77')
    # right_spin_text.config(justify=tk.CENTER)

    top_spin_text = tk.Text(relative_motor_control_frame, height=1, width=22)
    top_spin_text.insert(tk.END, "Top Spin")
    top_spin_text.grid(row=0, column=2)
    top_spin_text.config(state=tk.DISABLED)
    top_spin_text.configure(bg='gray77')
    # top_spin_text.config(justify=tk.CENTER)

    # forward velocity, top, and right spin boxes
    self.forward_velocity_spinbox = tk.Spinbox(relative_motor_control_frame, from_=0, to=255, increment=1)
    self.forward_velocity_spinbox.grid(row=1, column=0)
    self.forward_velocity_spinbox.configure(width=18)

    self.right_spin_spinbox = tk.Spinbox(relative_motor_control_frame, from_=0, to=255, increment=1)
    self.right_spin_spinbox.grid(row=1,column=1)
    self.right_spin_spinbox.configure(width=18)

    self.top_spin_spinbox = tk.Spinbox(relative_motor_control_frame, from_=0, to=255, increment=1)
    self.top_spin_spinbox.grid(row=1,column=2)
    self.top_spin_spinbox.configure(width=18)

    # PID value boxes
    pid_section_frame = tk.Frame(self.root)
    pid_section_frame.pack(pady= 20)

    pid_text_frame = tk.Frame(pid_section_frame)
    pid_text_frame.grid(row=0, column=0)
    # pid_text_frame.grid(row=0,column=0)
    # text boxes for pid
    kp_text = tk.Text(pid_text_frame, height=1, width=8)
    # kp_text.pack(side=tk.LEFT)
    kp_text.grid(row=0, column=0)
    kp_text.insert(tk.END, "P: ")
    kp_text.config(state=tk.DISABLED)
    kp_text.configure(bg='gray77')
    
    ki_text = tk.Text(pid_text_frame, height=1, width=8)
    # ki_text.pack(side=tk.LEFT)
    ki_text.grid(row=0, column=1)
    ki_text.insert(tk.END, "I: ")
    ki_text.config(state=tk.DISABLED)
    ki_text.configure(bg='gray77')

    kd_text = tk.Text(pid_text_frame, height=1, width=8)
    # kd_text.pack(side=tk.LEFT)
    kd_text.grid(row=0, column=2)
    kd_text.insert(tk.END, "D: ")
    kd_text.config(state=tk.DISABLED)
    kd_text.configure(bg='gray77')


    # frame for pid buttons
    pid_button_frame = tk.Frame(pid_section_frame)
    # pid_button_frame.pack()
    pid_button_frame.grid(row=1, column=0)
    self.kp_box = tk.Spinbox(pid_button_frame, from_=0, to=10, increment=.1)
    # self.kp_box.pack(side=tk.LEFT)
    self.kp_box.grid(row=0, column=0)
    self.kp_box.configure(width=5)

    self.ki_box = tk.Spinbox(pid_button_frame, from_=0, to=10, increment=.1)
    # self.kp_box.pack(side=tk.LEFT)
    self.ki_box.grid(row=0, column=1)
    self.ki_box.configure(width=5)

    self.kd_box = tk.Spinbox(pid_button_frame, from_=0, to=10, increment=.1)
    # self.kd_box.pack(side=tk.LEFT)
    self.kd_box.grid(row=0, column=2)
    self.kd_box.configure(width=5)


    # What mode are we in, automatic or manual
    self.mode_val = tk.IntVar()
    self.manual_button = tk.Radiobutton(self.root, text="Manual", variable=self.mode_val, value=1, command=self.mode_change)
    self.manual_button.pack()
    self.automatic_button = tk.Radiobutton(self.root, text="Automatic", variable=self.mode_val, value=2, command=self.mode_change)
    self.automatic_button.pack()

    # insert buttons into panel
    # 4 buttons: Fire, update motors, update pid, and Quit
    main_button_panel = tk.Frame(self.root)
    main_button_panel.pack(pady = 20)

    self.fire_button = tk.Button(main_button_panel, text="Fire", width = 20)
    self.update_motor_speed_button = tk.Button(main_button_panel, text="Update Motor Values", width = 20)
    self.update_pid_button = tk.Button(main_button_panel, text="Update PID", width = 20)
    self.quit_button = tk.Button(main_button_panel, text="Quit", width = 20)

    # self.fire_button.pack(fill=tk.X, side=tk.TOP)
    # self.update_motor_speed_button.pack(fill=tk.X, side=tk.BOTTOM)
    # self.update_pid_button.pack(fill=tk.X, side=tk.BOTTOM)
    # self.quit_button.pack(fill=tk.X, side=tk.TOP)
    self.fire_button.grid(row=1,column=1, padx =5, pady = 5)
    self.update_motor_speed_button.grid(row=0,column=1, padx=5, pady = 5)
    self.update_pid_button.grid(row=0,column=0, padx =5, pady = 5)
    self.quit_button.grid(row=1, column=0, padx =5, pady = 5)

    self.fire_button.bind('<Button-1>', self.fire)
    self.update_motor_speed_button.bind('<Button-1>', self.update_motor_speed)
    self.update_pid_button.bind('<Button-1>', self.update_pid_values)
    self.quit_button.bind('<Button-1>', self.quit)

    self.root.bind("<Return>", self.fire)
    self.root.bind("p", self.update_img)
    self.root.bind("c", self.calibrate)

    # # display updated picture occasionally
    self.root.after(100, self.update_img)

    # # display updated speed occasionally
    self.root.after(100, self.update_speeds_label)

    

  def calibrate(self, arg):
    # camera calibration
    # select four corners of image to do translation points
    corner_calibrate = Calibrate_Pixel_2_World()
    img = self.pong_system.img

    if img != None:
        corner_calibrate.start_calibration(img)

        self.vision_helper.top_cup_left_row = corner_calibrate.top_cup_left_row
        self.vision_helper.top_cup_left_col = corner_calibrate.top_cup_left_col

        self.vision_helper.top_cup_right_row = corner_calibrate.top_cup_right_row
        self.vision_helper.top_cup_right_col = corner_calibrate.top_cup_right_col

        self.vision_helper.bot_cup_left_row = corner_calibrate.bot_cup_left_row
        self.vision_helper.bot_cup_left_col = corner_calibrate.bot_cup_left_col

        self.vision_helper.bot_cup_right_row = corner_calibrate.bot_cup_right_row
        self.vision_helper.bot_cup_right_col = corner_calibrate.bot_cup_right_col

  def start_gui(self):
    # update image 
    self.update_img()

    # select manual mode
    self.manual_button.select()

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
    self.pong_system.update_motor_speed(0, 0, 0)
    print 'Quit'
    time.sleep(.5)
    self.pong_system.shutdown("Quit pressed in GUI")
    self.root.destroy()

  def mode_change(self):
    # print 'Selection changed to: ', self.mode_val.get()
    if self.mode_val.get() == 1:
        self.manual_mode = True
        self.pong_system.manual = True
    else:
        self.manual_mode = False
        self.pong_system.manual = False

  def update_img(self, arg=None):
    # get image from pong system class
    img = self.pong_system.img

    if img is not None:
        # convert image to be friendly with tkinter
        # rearrange color channel
        # print arg
        # self.root.after(4000, self.update_img)

        b,g,r = cv2.split(img)
        img = cv2.merge((r,g,b))

        im = PilImage.fromarray(img)
        self.imgtk = ImageTk.PhotoImage(image=im)

        self.img_panel.configure(image=self.imgtk)

  def update_speeds_label(self, arg=None):
    # get speeds from motors 
    motor_a_speed = self.pong_system.motor_a.speed
    motor_b_speed = self.pong_system.motor_b.speed
    motor_c_speed = self.pong_system.motor_c.speed

    motor_a_speed_str = '{0:.2f}'.format(motor_a_speed)
    motor_b_speed_str = '{0:.2f}'.format(motor_b_speed)
    motor_c_speed_str = '{0:.2f}'.format(motor_c_speed)

    self.cur_speed_a_val.configure(text = motor_a_speed_str)
    self.cur_speed_b_val.configure(text = motor_a_speed_str)
    self.cur_speed_c_val.configure(text = motor_a_speed_str)

    # # display updated speed occasionally
    self.root.after(100, self.update_speeds_label)


  def img_clicked_button(self, arg):
    ''' If the system is in manual mode, then take the click and 
        aim for the selected alrea. Otherwise, ignore the click.
    '''
    row = arg.y
    col = arg.x

    if self.manual_mode:
        # Turn the canvas click points into points that are relative to the image location
        # print "im size", self.imgtk.  
        canvas_width = self.img_panel.winfo_width()
        canvas_height = self.img_panel.winfo_height()

        # img_width, img_height = self.pong_system.img.shape
        img_width = self.imgtk.width()
        img_height = self.imgtk.height()

        # canvas width minus image width
        x_offset = (canvas_width - img_width) / 2
        y_offset = (canvas_height - img_height) / 2
        
        # click relative to img pixel
        x_img_pixel = arg.x - x_offset
        y_img_pixel = arg.y - y_offset
        
        # print "panel clicked"
        # print '({0},{1})'.format(arg.x, arg.y)
        # print "canvas width: ", canvas_width
        # print "canvas height: ", canvas_height
        # print "Image width: ", img_width
        # print "Image height: ", img_height
        # print "x offset: ", x_offset
        # print "y offset: ", y_offset
        # print "X pixel on image: ", x_img_pixel
        # print "Y pixel on image: ", y_img_pixel
        # print 
        # print 

        # feed the information to the pong system

    # output info to gui

    # pixel
    pixel_value_string = '({0}, {1})        '.format(row, col)
    self.pixel_value_label.configure(text = pixel_value_string)

    # get lateral and dist value
    dist, lateral = self.vision_helper.calc_position_from_pixel(row, col)
    self.lat_value_label.configure(text = str(lateral))
    self.dist_value_label.configure(text = str(dist))


  def setup_state(self, arg=None):
    # print " setup "
    self.game_setup_text.config(bg='green2')
    self.game_offense_text.config(bg='gray77')
    self.game_defense_text.config(bg='gray77')

    self.game_state = Game_State.SETUP

  def offense_state(self, arg=None):
    # print "offense "
    self.game_setup_text.config(bg='gray77')
    self.game_offense_text.config(bg='green2')
    self.game_defense_text.config(bg='gray77')

    self.game_state = Game_State.OFFENSE

  def defense_state(self, arg=None):
    # print "defense"
    self.game_setup_text.config(bg='gray77')
    self.game_offense_text.config(bg='gray77')
    self.game_defense_text.config(bg='green2')

    self.game_state = Game_State.DEFENSE

def main():

    # Have the user dictate whether or not they are on offense
    #side = Select_Side()
    #selected = side.selected
    #assert(selected)
    #on_offense = side.on_offense

    rospy.init_node('pong_system')

    # pong = Pong_System(on_offense)
    pong = Pong_System(on_offense=True)

    start_system_gui(pong)
    
    # not needed because Tkinter is already doing a loop called main loop
    # could loop here endlessly and call it root.update 
    # see http://stackoverflow.com/questions/459083/how-do-you-run-your-own-code-alongside-tkinters-event-loop
    # rospy.spin()

    
    

def start_system_gui(pong_system):
    img = cv2.imread("saved.jpg", 1)
    b,g,r = cv2.split(img)
    img = cv2.merge((r,g,b))
    
    gui = System_GUI(img, pong_system)
    gui.start_gui()


if __name__ == '__main__':
    
    main()
