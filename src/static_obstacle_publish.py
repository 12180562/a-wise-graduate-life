#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import math
import rospy
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from udp_col_msg.msg import static_OB_info

class static_obstacle:
    def __init__(self):
        self.static_OB_pub = rospy.Publisher('/static_OB_info', static_OB_info, queue_size=10)

    def static_obstacle_data_make(self):
    #     static_obstacle = pd.read_csv('/home/hyoguen/catkin_ws/src/inha_modules/src/static_obstacle.csv',header = None)
    #     # header를 none으로 지정하면 칼럼도 숫자로 저장

    #     static_obstacle.columns = ["x","y"]

    #     x_list = []
    #     y_list = []

    #     for i in range(len(static_obstacle)):
    #         x_list.append(static_obstacle["x"][i])
    #         y_list.append(static_obstacle["y"][i])

        
    #     # make obstacle list
    #     obstacle_list_COLINE_x = list(map(float,x_list[1:5]))
    #     obstacle_list_COLINE_y = list(map(float,y_list[1:5]))

    #     # for i in range(len(obstacle_list_COLINE_x)):
    #     #     point = [obstacle_list_COLINE_x[i],obstacle_list_COLINE_y[i]]
    #     #     if self.find_point_near_OS(point,OS_pos) == False:
    #     #         obstacle_list_COLINE_x.remove(obstacle_list_COLINE_x[i])
    #     #         obstacle_list_COLINE_y.remove(obstacle_list_COLINE_y[i])
    #     #     else:
    #     #         pass

    #     obstacle_list_SUBSEA_x = list(map(float,x_list[6:10]))
    #     obstacle_list_SUBSEA_y = list(map(float,y_list[6:10]))

    #     # for i in range(len(obstacle_list_SUBSEA_x)):
    #     #     point = [obstacle_list_SUBSEA_x[i],obstacle_list_SUBSEA_y[i]]
    #     #     if self.find_point_near_OS(point,OS_pos) == False:
    #     #         obstacle_list_SUBSEA_x.remove(obstacle_list_SUBSEA_x[i])
    #     #         obstacle_list_SUBSEA_y.remove(obstacle_list_SUBSEA_y[i])
    #     #     else:
    #     #         pass

        
    #     # publish data
        static_OB = static_OB_info()
    #     static_OB.obstacle_list_COLINE_x = obstacle_list_COLINE_x
    #     static_OB.obstacle_list_COLINE_y = obstacle_list_COLINE_y
    #     static_OB.obstacle_list_SUBSEA_x = obstacle_list_SUBSEA_x
    #     static_OB.obstacle_list_SUBSEA_y = obstacle_list_SUBSEA_y

    #     self.static_OB_pub.publish(static_OB)

    #     return 0

    # # def find_point_near_OS(self, point, OS_pos):
    # #     OS_position = OS_pos
    # #     distance_from_OS = math.sqrt((OS_position[0]-point[0])**2+(OS_position[1]-point[1])**2)
    # #     if distance_from_OS < 50:
    # #         return True
    # #     else:
    # #         return False
        #static_OB.data = [25,125,65,125,65,125,65,100,65,100,25,100,25,100,25,125,135,125,165,125,165,125,165,105,165,105,135,105,135,105,135,125,65,90,135,90,135,90,135,60,135,60,65,60,65,60,65,90]
        #static_OB.data = [50,100,100,100,100,100,100,50,100,50,50,50,50,50,50,100,-50,-100,-100,-100,-100,-100,-100,-50,-100,-50,-50,-50,-50,-50,-50,-100]

        #원하는 좌표 그대로 넣으면 문제 없음
        new_static_OB = []
        static_OB.data = []
        static_OB.data = static_OB.data + [150,500,150,400,150,400,350,400,350,400,350,500]
        static_OB.data = static_OB.data + [0,250,150,250,150,250,150,0]
        static_OB.data = static_OB.data + [350,0,350,250,350,250,500,250]
        static_OB.data = static_OB.data + [0,0,0,500,0,500,500,500,500,500,500,0,500,0,0,0]

        static_OB.point = [0,25,10,50,-10,75,50,50,-50,50]
        # for i in static_OB.data:
        #     j=i
        #     new_static_OB.append(j)

        # static_OB.data = new_static_OB
        self.static_OB_pub.publish(static_OB)

def main():
    # make publish method.
    # get information of obstacle
    rospy.init_node("static_obstacle_publish", anonymous=False)    
    static_OB_class = static_obstacle()

    while not rospy.is_shutdown():
        static_obstacle_info = static_OB_class.static_obstacle_data_make()
    rospy.spin()

if __name__ == '__main__':
    main()

    # import obstacle point that type is array. and, they are have x coordinate set and y coordinate set.
    # ex) np.array([[x coordinate set],[y coordinate set]])
    # the first row is x_set, and second row is y_set.

    # give msg to topic. we have to make msg file, and the type of msg is float64[]. 
    # this msg type is numpy. Need i change the msg type to list?
    # add some code that makes pandas dataframe to numpy array.
    # also, we take the point imformation just continuous coordinate set.
    # we have to change this to other shape that is need for finding line cross coordinate

    # ex) [x1, y1, x2, y2, x3, y3] ----->> x_coordinate = [[x1,x2], [x2,x3]]
    #                              ----->> x_coordinate = [[y1,y2], [y2,y3]]

    # if it is so hard, we change the type of point, using in static obstacle avoidance.