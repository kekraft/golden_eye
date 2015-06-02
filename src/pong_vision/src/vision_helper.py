#!/usr/bin/env python 

import math
import numpy as np
import cv2

'''
Purpose: This class exists to handle the calculations of translating cup center pixel values 
     to x (lateral position relative to table) and z (distance from top of cups to robot along top of cup plane)
     position relative to the robot, for use in our Applied Robotics ping pong robot. 

     NOTE: Major assumptions are made
        1) The camera is laterally centered on the table
        2) Everything in the image is on a flat horizontal plane that is parallel with the cup tops and at the same height as the cups
    
    To use this class correctly, 4 points must be selected that indicate the top left of the cup on the table, the top right of the cup on the table
     the bottom left of the cup on the table, and the bottom right of the cup on the table

    Also, the table width and table length must be given.
    
    Even more importantly, is the dist to the top left and right corners from the robot (around 8')

    Also, the distance from the bottom of what is seen in the image to the robot (not sure what this will be)

    Lastly, the number of rows and cols must be specified

Author: Kory Kraft
Date: 5/22/2015
'''

class Vision_Helper:
    table_width = 2.5 # manual
    table_length = 6.0 # manual
    dist_at_top = 6.0 # manual
    dist_at_bottom = 2.0 # manual
    
    img_rows = 640.0 # manual
    img_cols = 480.0 # manual

    # variables to get lateral position of cups bsed on initial calibration with top of cups.. init values were just for testing
    top_cup_left_row = 63.0 # click 1
    top_cup_left_col = 164.0

    top_cup_right_row = 86.0 # click 2
    top_cup_right_col = 483.0

    bot_cup_left_row = 433.0 # click 3
    bot_cup_left_col = 4.0

    bot_cup_right_row = 475.0 # click 4
    bot_cup_right_col = 602.0

    def calc_position_from_pixel(self, row, col):
        # returns the distance from the robot to the pixel value
        # and the lateral distance of the pixel value from center of table
        distance = self.get_dist_from_row_pixel(row)

        
        self.calc_center_endpts()
        self.calc_center_slope()
        self.calc_LH()
        self.calc_RH()

        lateral = self.calc_lateral(row, col)
        return distance, lateral

    ###### Methods that deal with obtaining the distance of the object from the robot
    ######   alonge the plane formed by the top of the cups
    ######
    def calc_dist_slope(self):
        self.dist_m = (self.dist_at_bottom - self.dist_at_top) / (self.img_rows - 0.0)
        return self.dist_m

    def get_dist_from_row_pixel(self, row):
        # just doing simple point slope intercept math
        # distance is along the plane made by the top of the cups
        # I assume 
        #   the distance at the top of the image is known
        #   the distance at the bottom of the image is known
        # in this case, y = height and x = row
        self.dist_intercept = self.dist_at_top
        height = self.calc_dist_slope() * row + self.dist_intercept
        return height

    #### Methods that deal with obtaining the lateral position of a column pixel
    ####  Center of table = x = 0. Far right of table = +x. Far left of table = -x.
    ####
    ####  Relies on top_cup_left (click 1), top_cup_right (click 2), bot_cup_left (click 3), bot_cup_right (click 4)
    def calc_center_endpts(self):
        # don't have to take abs value of cols, becuase pts were click on left and right, and must be less than if pt is on left compared to right
        self.top_center_pt_col = self.top_cup_left_col + ((self.top_cup_right_col - self.top_cup_left_col) / 2.0)
        self.bot_center_pt_col = (self.bot_cup_left_col) + ((self.bot_cup_right_col - self.bot_cup_left_col)/2.0)
        
        closer_top_row = -999
        if self.top_cup_left_row < self.top_cup_right_row:
            closer_top_row = self.top_cup_left_row
        else:
            closer_top_row = self.top_cup_right_row

        self.top_center_pt_row = closer_top_row + abs(((self.top_cup_left_row - self.top_cup_right_row)/2.0))

        closer_bot_row = -999
        if self.bot_cup_left_row < self.bot_cup_right_row:
            closer_bot_row = self.bot_cup_left_row
        else:
            closer_bot_row = self.bot_cup_right_row
        
        self.bot_center_pt_row = closer_bot_row + abs(((self.bot_cup_left_row - self.bot_cup_right_row)/2.0))

    def calc_center_slope(self):
        # calculating slope of center line (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        self.center_m = (self.bot_center_pt_col - self.top_center_pt_col) / (self.bot_center_pt_row - self.top_center_pt_row)
        return self.center_m    

    def calc_LH(self):
        # calculating slope of center line (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # LH = the lefthand line that goes from click point 1 to click point 3 (top left to bottom left)
        self.lh_m = (self.bot_cup_left_col - self.top_cup_left_col) / (self.bot_cup_left_row - self.top_cup_left_row)
        return self.lh_m

    def calc_RH(self):
        # calculating slope of center line (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # RH = the righthand line that goes from click point 2 to click point 4 (top right to bottom right)
        self.rh_m = (self.bot_cup_right_col - self.top_cup_right_col) / (self.bot_cup_right_row - self.top_cup_right_row)
        return self.rh_m

    def calc_col_on_center(self, row):
        # note (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # calculate intercept for center line
        self.center_intercept = self.top_center_pt_col - (self.center_m * self.top_center_pt_row)
        
        col = self.center_m * row + self.center_intercept
        return col

    def calc_col_on_lh(self, row):
        # note (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # calculate intercept for lh line
        self.lh_intercept = self.top_cup_left_col - (self.lh_m * self.top_cup_left_row)
        
        col = self.lh_m * row + self.lh_intercept
        return col

    def calc_col_on_rh(self, row):
        # note (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # calculate intercept for rh line
        self.rh_intercept = self.top_cup_right_col - (self.lh_m * self.top_cup_right_row)
        
        col = self.rh_m * row + self.rh_intercept
        return col
    
    def calc_dist_to_center(self, row, col):
        # note (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # find matching point on center line
        # assume it ins on the same row since we are assuming we are facing the table square on.
        center_col = self.calc_col_on_center(row)       
        
        # take distance between pts
        dist_to_center = math.sqrt( (col - center_col) * (col - center_col) )

        return dist_to_center

    def calc_dist_to_lh(self, row, col):
        # note (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # find matching point on lh line
        # assume it ins on the same row since we are assuming we are facing the table square on.
        lh_col = self.calc_col_on_lh(row)
        
        dist_to_lh = math.sqrt( (col - lh_col) * (col - lh_col))

        return dist_to_lh

    def calc_dist_to_rh(self, row, col):
        # note (x is considered rows, y is considered columns..) (image is rotated right 90 degrees for this perspective)
        # find matching point on rh line
        # assume it ins on the same row since we are assuming we are facing the table square on.
        rh_col = self.calc_col_on_rh(row)

        dist_to_rh = math.sqrt( (col - rh_col) * (col - rh_col))
        
        return dist_to_rh

    def calc_lateral(self, row, col):
        # + = right of table center, - = left of table center

        # see if in rh or lh side
        rh_dist = self.calc_dist_to_rh(row, col)
        lh_dist = self.calc_dist_to_lh(row, col)

        if rh_dist < lh_dist:
            on_right = True
            dist_from_outer = rh_dist
        
            # see how far away rh is away from center (which in reality is half the width of the table)
            outer_col = self.calc_col_on_rh(row)            

        else:
            on_right = False
            dist_from_outer = lh_dist
            
            # see how far away hh is away from center (which in reality is half the width of the table)
            outer_col = self.calc_col_on_lh(row)
                    
        # get distance from outer col to center (to see what that section length is in pixels)  
        outer_to_center_dist = self.calc_dist_to_center(row, outer_col)
        
        # get percentage distance on outer side from center compared to section dist
        percent_dist = 1.0 - (dist_from_outer / outer_to_center_dist)
        
        # convert percentage distance of pixels to actual table width
        lateral_dist = percent_dist * (self.table_width / 2.0)

        if on_right:
            return lateral_dist + (self.table_width / 2.0)
        else:
            return -lateral_dist + (self.table_width / 2.0)

class Calibrate_Pixel_2_World():
    click_ctr = 0

    def select_corners(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            print "lbutton down"

            self.click_ctr = self.click_ctr + 1

            if self.click_ctr == 1:
                self.top_cup_left_row = y # click 1
                self.top_cup_left_col = x

            if self.click_ctr == 2:
                self.top_cup_right_row = y
                self.top_cup_right_col = x

            if self.click_ctr == 3:
                self.bot_cup_left_row = y
                self.bot_cup_left_col = x

            if self.click_ctr == 4:
                self.bot_cup_right_row = y
                self.bot_cup_right_col = x

                # cv2.destroyAllWindows()
        # print "X: ", x
        # print "Y: ", y
    

    def start_calibration(self, img = None):
        self.window = cv2.namedWindow("Select 4 Corners | Top Left, Top Right, Bottom Left, Bottom Right")
        cv2.setMouseCallback("Select 4 Corners | Top Left, Top Right, Bottom Left, Bottom Right", self.select_corners)

        assert(img != None)

        while True:
            # display the image and wait for a keypress
            # if img == None:
            #     img = cv2.imread()

            cv2.imshow("Select 4 Corners | Top Left, Top Right, Bottom Left, Bottom Right", img)
            key = cv2.waitKey(1) & 0xFF

            if self.click_ctr == 4:
                break
         
        cv2.destroyAllWindows()






        
        
    
        
