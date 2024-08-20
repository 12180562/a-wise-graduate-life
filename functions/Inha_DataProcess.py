import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.CRI import CRI
from numpy import deg2rad, rad2deg
from math import sin, cos, pi, sqrt, atan2

import numpy as np
import rospy

class Inha_dataProcess:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(
        self,
        ship_ID, 
        Pos_X, 
        Pos_Y,
        Vel_U, 
        Heading, 
        waypoint_dict, 
        ):

        self.ship_ID = ship_ID
        self.Pos_X = Pos_X
        self.Pos_Y = Pos_Y
        self.Vel_U = Vel_U
        self.Heading = Heading
        self.waypoint_dict = waypoint_dict

        self.ship_dic= {}
        self.SD_param = rospy.get_param('SD_param')

    def ship_list_container(self, OS_ID):
        ''' 
            Subscribe한 선박의 운항정보를 dictionary로 저장 
        
            Return : 
                ship_list_dic
                ship_ID
        '''

        for i in range(len(self.ship_ID)):
            index_ship = self.ship_ID[i]
            if index_ship == OS_ID:
                self.ship_dic[OS_ID]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Pos_X' : self.Pos_X[i],
                    'Pos_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    }

            else:
                self.ship_dic[index_ship]= {
                    'Ship_ID' : int(self.ship_ID[i]),
                    'Pos_X' : self.Pos_X[i],
                    'Pos_Y' : self.Pos_Y[i],
                    'Vel_U' : self.Vel_U[i],
                    'Heading' : self.Heading[i],
                    }

        # print(self.ship_dic)
        # print(self.ship_ID)
        # print("원래 X: {}, Y: {}".format(self.ship_dic[OS_ID]['Pos_X'], self.ship_dic[OS_ID]['Pos_Y']))

        return self.ship_dic, self.ship_ID

    def classify_OS_TS(self, ship_dic, ship_ID, OS_ID):
        # detecting_distance = rospy.get_param("detecting_distance")
        ''' 자선과 타선의 운항정보 분리
        
        Return :
            OS_list, TS_list // (dataframe)
        '''
        if len(ship_ID) == 1:
            OS_list = ship_dic[OS_ID]
            TS_list = None

        else:
            for i in range(len(ship_ID)):
                if ship_ID[i] == OS_ID:
                    OS_list = ship_dic[OS_ID]

            
            TS_list = ship_dic.copy()
            # FIXME: `OS_ID` does not exist in `TS_list` when processing TSs? The `OS_ID` is not the "OS"????
            del(TS_list[OS_ID])
            
        # print(TS_list)
        return OS_list, TS_list

    def CRI_cal(self, OS, TS):
        cri = CRI(
            rospy.get_param("shipInfo_all/ship1_info/ship_L"),
            rospy.get_param("shipInfo_all/ship1_info/ship_B"),
            OS['Pos_X'],
            OS['Pos_Y'],
            TS['Pos_X'],
            TS['Pos_Y'],
            deg2rad(OS['Heading']),
            deg2rad(TS['Heading']),
            OS['Vel_U'],
            TS['Vel_U'],
            rospy.get_param("shipInfo_all/ship1_info/ship_scale"),
        )

        RD = cri.RD()
        RC = cri.RC()
        TB = rad2deg(cri.TB())
        RB = rad2deg(cri.RB())

        Vox = cri.Vox()
        Voy = cri.Voy()
        Vtx = cri.Vtx()
        Vty = cri.Vty()

        DCPA = cri.dcpa()
        TCPA = cri.tcpa()

        K = cri.K()

        UDCPA = cri.UDCPA()
        UTCPA = cri.UTCPA()
        UD = cri.UD()
        UB = cri.UB()
        UK = cri.UK()

        enc = cri.encounter_classification()

        Rf = cri.Rf()
        Ra = cri.Ra()
        Rs = cri.Rs()
        Rp = cri.Rp()
        SD_dist = cri.SD_dist()
        # rb, lb = cri.SD_dist_new()

        cri_value = cri.CRI()

        # return RD, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value, rb,lb
        return RD, RC, TB, RB, K, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value

    def U_to_vector_V(self, U, deg):
        ''' Heading angle을 지구좌표계 기준의 속도벡터로 변환

        Return:
            Vx, Vy  [m/s]     
        '''
        psi = deg2rad(deg) ##강제로 xy좌표에서 NED좌표로 변환
        
        V_x = U * cos(psi)
        V_y = U * sin(psi)

        return V_x, V_y

    def waypoint_generator(self, OS, V_selected, dt):
        ''' `V_des` 방향 벡터를 기준으로 1초뒤 point를 waypoint 생성
        
        Return :
            wp_x, wp_y [m]
        '''

        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        wp = OS_X + V_selected * dt

        wp_x = wp[0]
        wp_y = wp[1]

        return wp_x, wp_y

    def eta_eda_assumption(self, WP, OS, target_U):
        ''' 목적지까지 도달 예상 시간 및 거리
        
        Return:
            eta [t], eda [m]
        '''
        OS_X = np.array([OS['Pos_X'], OS['Pos_Y']])

        distance = np.linalg.norm(WP - OS_X)

        eta = distance/ target_U
        eda = distance

        return eta, eda

    def desired_value_assumption(self, V_des):
        ''' 목적지까지 향하기 위한 Desired heading angle
        
        Return :
            desired_speed [m/s], desired_heading [deg]
        '''
        U_des = sqrt(V_des[0]**2 + V_des[1]**2)
        target_head = rad2deg(atan2(V_des[1], V_des[0])) ## NED 좌표계 기준으로 목적지를 바라보는 방향각

        desired_heading = target_head        
        desired_spd = U_des

        return desired_spd, desired_heading


    def TS_info_supplement(self, OS_list, TS_list):   
        """ TS에 대한 추가적인 information 생성 

        Returns:
            TS_info : pandas dataframe

        Note : 
            TS list =  {'Ship_ID': [], 'Pos_X' : [],  'Pos_Y' : [],   'Vel_U' : [],   'Heading' : [], 'V_x' : [], 'V_y' : [], 
                        'radius' : [], 'RD' : [], 'RB' : [],'RC' : [], 'local_rc' : [], 'status' : []}
        """

        if TS_list == None:
            TS_list = None
        else:
            TS_ID = TS_list.keys()
            for ts_ID in TS_ID:
                # RD, TB, RB, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value,rb,lb = self.CRI_cal(OS_list, TS_list[ts_ID])
                RD, RC, TB, RB, K, Vox, Voy, Vtx, Vty, DCPA, TCPA, UDCPA, UTCPA, UD, UB, UK, enc, Rf, Ra, Rs, Rp, SD_dist, cri_value = self.CRI_cal(OS_list, TS_list[ts_ID])

                TS_list[ts_ID]['RD'] = RD 
                TS_list[ts_ID]['RC'] = RC
                TS_list[ts_ID]['TB'] = TB  
                TS_list[ts_ID]['RB'] = RB
                TS_list[ts_ID]['K'] = K

                TS_list[ts_ID]['V_x'] = Vtx
                TS_list[ts_ID]['V_y'] = Vty

                TS_list[ts_ID]['DCPA'] = DCPA
                TS_list[ts_ID]['TCPA'] = TCPA

                TS_list[ts_ID]['UDCPA'] = UDCPA
                TS_list[ts_ID]['UTCPA'] = UTCPA
                TS_list[ts_ID]['UD'] = UD
                TS_list[ts_ID]['UB'] = UB
                TS_list[ts_ID]['UK'] = UK

                TS_list[ts_ID]['status'] = enc

                TS_list[ts_ID]['Rf'] = Rf
                TS_list[ts_ID]['Ra'] = Ra
                TS_list[ts_ID]['Rs'] = Rs
                TS_list[ts_ID]['Rp'] = Rp
                TS_list[ts_ID]['mapped_radius'] = SD_dist * self.SD_param
                # TS_list[ts_ID]["right_boundary"] = rb
                # TS_list[ts_ID]["left_boundary"] = lb

                TS_list[ts_ID]['CRI'] = cri_value

                # print(enc)

        return TS_list