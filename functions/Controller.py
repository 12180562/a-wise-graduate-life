#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
import rospy
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.PID import PID
from functions.MMG import KASS_MMG
from functions.mmg_non_dimension import MMG
from numpy import deg2rad
from math import pi

class Controller:
    """Speed, Steering, Heading 제어기 부분 """
    def __init__(self):
        mmg = MMG(ship_scale=rospy.get_param('OS_info/ship_scale'))
        self.max_delta = mmg.Model['Rudder_max']    # rad.
        self.n_plus = 0.0

    def heading_controller(self, target_head, current_head):
        '''
        `params` :
            `target_head` : [deg]
            `current_head` : [rad]

        Return :
            desired_Heading : [-pi, pi] 좌표계 // Rudder 제어 편의를 위하여, +면 우현선회, -면 좌현선회로 사용
        '''
        target_head = deg2rad(target_head) % (2*pi) # [0, 2*pi] 좌표계로 변환
        current_head = current_head %(2*pi)         # [0, 2*pi] 좌표계로 변환

        desired_heading = target_head - current_head

        if desired_heading >= pi:
            desired_heading = desired_heading - 2* pi
        elif desired_heading < - pi:
            desired_heading = desired_heading + 2 *pi
        else:
            desired_heading = desired_heading

        return desired_heading # [-pi, pi] 좌표계로 변환
    
    def steering_controller(self, target_head, delta, rudder_rate, dt):
        """ `Rudder rate`를 고려한 rudder 제어기

        Params: 
            target_head : `desired 타각`을 의미
            delta : 현재 rudder의 타각을 의미
        Return:
            delta : 타 제어기를 통해 계산된 현재 rudder의 타각을 의미
        """        
        ## PID 제어기
        # 본 과제에서는 제어기가 초점이 아닌관계로, error만큼을 Rudder의 값변화로 주었음
        Kp = rospy.get_param('Kp')
        Ki = rospy.get_param('Ki')
        Kd = rospy.get_param('Kd')
        pid = PID(Kp, Ki, Kd)  

        target_delta = pid.Update(target_head) 
        # print("target delta : ", rad2deg(target_delta))

        max_delta = self.max_delta      # 타가 취할 수 있는 최대 타각
        
        delta = -delta    
        # 과제에서 주어진 rudder 좌표계는 `타각은 위에서 볼 때 시계방향 회전이 (+)`로 주어져 있으르므로,
        # 즉, 우현으로 35도를 꺽고싶다면, 프로그래밍상 ‘-35도’를 주어야 함.
        # --> 자세한 좌표계 관련 내용은 `시험선조종모델&미계수(2020.11_KRISO)_R2_INHA.hwp`참조 바람!!

        if target_delta >= 0:
            max_delta = max_delta
            delta = delta + rudder_rate * dt
        elif target_delta < 0:
            max_delta = - max_delta
            delta = delta - rudder_rate * dt

        if abs(delta) >= abs(target_delta):
            delta = target_delta
            # 현재의 타각은 desired 타각 보다 클 수 없음
        
        if abs(delta) >= abs(max_delta):
            delta = max_delta
            # 현재의 타각은 최대 타각 보다 클 수 없음        

        delta = -delta # 좌표계 반영을 위한 음수값 처리

        return delta

    def speed_controller(self, target_rpm, n):
        """ 속도 제어기라기 보다는, `desired_speed`에 대응하는 모형선의 자항점을 추정하여, 저항에 따른 속도 감속에 대응

        Returns:
            n : 목표 속도를 내기위한 자항점
        """ 


        ## target speed를 도달하기 위해 추가적으로 필요한 `n`
        error_rpm = target_rpm - n
        # print("error : ", error_speed)          

        if abs(error_rpm) > abs(0.01 * target_rpm) :
            # speed 오차가 target speed의 0.1배 이내에 들어오도록 조정
            if error_rpm >= 0 :
                self.n_plus  = 0.1
            elif error_rpm < 0 :
                self.n_plus = -0.1
        else:
            self.n_plus = self.n_plus

        n = n + self.n_plus
        
        return n  