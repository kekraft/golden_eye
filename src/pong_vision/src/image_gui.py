#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
import random

import Tkinter
import Image, ImageTk # Had to install sudo apt-get install python-imaging-tk

#import sift
class System_GUI():
  def __init__(self, img):
    root = Tkinter.Tk()
    im = Image.fromarray(img)
    imgtk = ImageTk.PhotoImage(image=im)

    # insert image into panel
    panel = Tkinter.Label(root, image = imgtk)
    panel.pack(side = "top", fill = "both", expand = "yes")

    # Game State Text....need method for updating this to offense and defense
    self.game_state_text = Tkinter.Text(root, height=1, width = 8)
    self.game_state_text.config(bg='gray77')
    self.game_state_text.config(fg='black')
    self.game_state_text.pack()
    self.game_state_text.insert(Tkinter.END, " SETUP")
    self.game_state_text.config(state=Tkinter.DISABLED)

    # motor spin boxes
    motor_a_text = Tkinter.Text(root, height=1, width=8)
    motor_a_text.pack()
    motor_a_text.insert(Tkinter.END, "Motor A:")
    motor_a_text.config(state=Tkinter.DISABLED)
    motor_a_text.configure(bg='gray77')
    self.motor_a_velocity_box = Tkinter.Spinbox(root, from_=0, to=10, increment=.1)
    self.motor_a_velocity_box.pack()

    motor_b_text = Tkinter.Text(root, height=1, width=8)
    motor_b_text.pack()
    motor_b_text.insert(Tkinter.END, "Motor B:")
    motor_b_text.config(state=Tkinter.DISABLED)
    motor_b_text.configure(bg='gray77')
    self.motor_b_velocity_box = Tkinter.Spinbox(root, from_=0, to=10, increment=.1)
    self.motor_b_velocity_box.pack()

    motor_c_text = Tkinter.Text(root, height=1, width=8)
    motor_c_text.pack()
    motor_c_text.insert(Tkinter.END, "Motor C:")
    motor_c_text.config(state=Tkinter.DISABLED)
    motor_c_text.configure(bg='gray77')
    self.motor_c_velocity_box = Tkinter.Spinbox(root, from_=0, to=10, increment=.1)
    self.motor_c_velocity_box.pack()

    # What mode are we in, automatic or manual
    self.mode_val = Tkinter.IntVar()
    self.manual_button = Tkinter.Radiobutton(root, text="Manual", variable=self.mode_val, value=1, command=self.mode_change)
    self.manual_button.pack()
    self.automatic_button = Tkinter.Radiobutton(root, text="Automatic", variable=self.mode_val, value=2, command=self.mode_change)
    self.automatic_button.pack()

    # insert buttons into panel
    # 2 buttons: Fire and Quit
    self.fire_button = Tkinter.Button(root, text="Fire")
    self.quit_button = Tkinter.Button(root, text="Quit")

    self.fire_button.pack()
    self.quit_button.pack()

    self.fire_button.bind('<Button-1>', self.fire)
    self.quit_button.bind('<Button-1>', self.quit)

    # start the GUI
    root.mainloop()

  def fire(self, arg):
    print 'Fire'
    motor_a_speed = float(self.motor_a_velocity_box.get())
    print 'Motor a Speed', motor_a_speed


  def quit(self, arg):
    print 'Quit'

  def mode_change(self):
    print 'Selection changed to: ', self.mode_val.get()
    


if __name__ == '__main__':

    # run pipeline with a given image 
    img = cv2.imread("../images/saved.jpg", 1)
    cv2.namedWindow("Display Window", cv2.WINDOW_AUTOSIZE)
    #cv2.imshow("Display Window", img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    # convert image to be friendly with tkinter
    # rearrange color channel
    b,g,r = cv2.split(img)
    img = cv2.merge((r,g,b))
    
    gui = System_GUI(img)

