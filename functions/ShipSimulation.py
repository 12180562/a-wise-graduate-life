#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.MMG import KASS_MMG
from functions.Controller import Controller
from functions.mmg_non_dimension import MMG

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../mmgdynamics/src')))

from mmgdynamics.dynamics import mmg_dynamics
import mmgdynamics.calibrated_vessels as cvs
from mmgdynamics.structs import Vessel

from numpy import deg2rad, rad2deg
from math import cos, sin, sqrt, pi
import numpy as np 
import rospy

class ShipSimulation(Controller): # `Controller()`       # Speed, Steering, Heading 제어기를 포함하고 있어서 상속해줌
    """시뮬레이션을 위한 클래스"""
    def __init__(self, initial_X, initial_Y, initial_velocity, initial_u,initial_v, initial_Heading_deg, initial_delta_deg, LBP ,ship_scale, dt):
        '''시뮬레이션을 위한 클래스

        Params
            initial_X: 시뮬레이션을 시작하는 선박의 x방향 시작지점
            initial_Y: 시뮬레이션을 시작하는 선박의 y방향 시작지점  
            initial_velocity : 초기 속도
            initial_Heading : 초기 헤딩각
            initial_delta : 초기 타각
            dt: 시뮬레이션을 위한 time step          
        '''
        self.X_Earth  = initial_X
        self.Y_Earth  = initial_Y

        self.r_rad = deg2rad(0.0)    # rad./sec. 
        self.psi_rad = deg2rad(initial_Heading_deg) # rad.
        self.delta_rad = deg2rad(initial_delta_deg) # rad.
        
        self.uBody = initial_u   # x방향으로 진행중이므로, vBody속도는 없다고 가정하고 initial값을 줌    # m/sec.
        self.vBody = initial_v

        self.LBP = LBP
        self.ship_scale = ship_scale

        self.rps = 5

        # mmg = MMG(ship_scale=rospy.get_param('shipInfo_all/ship1_info/ship_scale'))
        self.max_delta = 0.6106   # rad.
        self.n_plus = 0.0
        
        self.Time = 0.0
        self.dt = dt     

    def NED_Coordinate(self, uBody, vBody, psi):
        """
            `선체고정좌표계`에서 `지면좌표계(NED)`로 변환
            NED 좌표계는 x는 북쪽 방향(North), y는 동쪽 방향(East), z는 아래 방향(Down) 를 의미함
            
            Units:
                - psi: radian
        """
        ' Velocity in the Earth coordinate system'
        uEarthX = uBody * cos(psi)          # Earth's x direction component of uBody
        uEarthY = uBody * sin(psi)          # Earth's y direction component of uBody
        vEarthX = - vBody * sin(psi)        # Earth's x direction component of vBody
        vEarthY = vBody * cos(psi)          # Earth's y direction component of vBody

        Xdot_Earth = uEarthX + vEarthX         # Earth's x direction component of V(ship speed)
        Ydot_Earth = uEarthY + vEarthY         # Earth's y direction component of V(ship speed)
        
        'Make it start from the last position'
        self.X_Earth = self.X_Earth + Xdot_Earth * self.dt
        self.Y_Earth = self.Y_Earth + Ydot_Earth * self.dt

        return self.X_Earth, self.Y_Earth

    def moving_ships(self, target_heading, target_spd):
        
        KASS_mmg = MMG(self.uBody,self.vBody,self.r_rad,self.psi_rad,self.LBP,self.ship_scale)           # MMG 조종운동방정식을 활용하여, 자선/타선의 동역학적 특성을 반영함
        ########## ship scale에 맞게 모형선의 제원 값을 수정 #######
        Rudder_rate_rad = KASS_mmg.Model['Rudder_rate'] *sqrt(self.ship_scale) / sqrt(KASS_mmg.Model['scale'])  # rad./sec.

        uBody = self.uBody 
        vBody = self.vBody 
        r_rad = self.r_rad  # rad./sec.
        current_heading_rad = self.psi_rad  # rad.

        # TDOO: Is the unit of the `target_heading` radian?
        desired_head_rad = self.heading_controller(target_heading, current_heading_rad)
        delta_rad = self.steering_controller(desired_head_rad, self.delta_rad, Rudder_rate_rad, self.dt) # Rudder 좌표계 정의로 인해, 우현 선회는 -35 deg 임

        U = sqrt(uBody**2+vBody**2)

        _,_,_,target_rps = KASS_mmg.resistance_test(target_spd)

        # NOTE: It changes the `self`
        n = self.speed_controller(target_rps, self.rps)
        # print(target_spd,target_rps,n)

        ##########  `t`에서의 자선의 속도와 타각을 바탕으로 `t+1`에서의 자선의 가속도 계산 ##########
        
        velocity_matrix = np.array([[uBody], [vBody], [r_rad]])
        acceleration_matrix = KASS_mmg.main(delta_rad,n)

# Nicpau part  
#############################################

        # velocity_matrix = np.array([uBody, vBody, r_rad]) # Pack the ship velocity components
        # vessel = Vessel(**cvs.kvlcc2_full)
        # acceleration_matrix = mmg_dynamics(velocity_matrix, vessel, target_heading, 
        #                                    delta_rad, target_rps, 
        #                                    0, 0, 0, 0)

#############################################

        velocity_matrix = velocity_matrix + acceleration_matrix * self.dt

        '`velocity_matrix` is  `np.array` !  so, we need to change as scalar'   
        uBody = velocity_matrix[0].item()  # "np.asscalar" 
        vBody = velocity_matrix[1].item()  # "np.asscalar"                        
        r_rad = velocity_matrix[2].item()   # rad./sec.

        Vel = sqrt(uBody**2 + vBody**2)
        # New ship position from the new ship velocity
        psi_rad = self.psi_rad + r_rad * self.dt 
        psi_rad = psi_rad % (2 * pi)                   # compensate the angle representation range from [0, inf] to [0, 2*pi]

        # NOTE: It chagnes the `self`
        X_Earth, Y_Earth = self.NED_Coordinate(uBody, vBody, psi_rad)   # `선체고정좌표계`에서 `지구고정좌표계`로 변환

        # NOTE: It chages the `self`
        self.uBody = uBody
        self.vBody = vBody
        self.r_rad = r_rad  # rad./sec.
        self.psi_rad = psi_rad
        self.delta_rad = delta_rad
        self.Time = self.Time + self.dt 
        self.rps = n

        shipState = dict()
        shipState['X'] = X_Earth
        shipState['Y'] = Y_Earth
        shipState['U'] = Vel
        shipState['u'] = uBody
        shipState['v'] = vBody
        shipState['psi_deg'] = rad2deg(psi_rad)
        shipState['delta_deg'] = rad2deg(delta_rad)
        shipState['r_deg'] = rad2deg(r_rad)

        return shipState