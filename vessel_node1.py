#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.Inha_VelocityObstacle import VO_module
from functions.Inha_DataProcess import Inha_dataProcess
from functions.Ais_ukf import UKF

from udp_col_msg.msg import col, vis_info, cri_info, VO_info
from udp_msgs.msg import frm_info, group_wpts_info
from ukf_ais.msg import ShipInfo

from math import sqrt, atan2
from numpy import rad2deg

import csv
import numpy as np
import rospy
import time
import rospkg

import copy

class data_inNout:
    def __init__(self):
        rospy.Subscriber('/frm_info', frm_info, self.OP_callback)
        rospy.Subscriber('/waypoint_info', group_wpts_info, self.wp_callback)

        self.WP_pub = rospy.Publisher('/vessel1_info', col, queue_size=0)
        self.cri_pub = rospy.Publisher('/cri1_info', cri_info, queue_size=10)
        self.VO_pub = rospy.Publisher('/VO1_info', VO_info, queue_size=10)
        self.Vis_pub = rospy.Publisher('/vis1_info', vis_info, queue_size=10)

        self.ori_pub = rospy.Publisher('TS_list_ori', ShipInfo, queue_size=10)
        self.del_pub = rospy.Publisher('TS_list_d', ShipInfo, queue_size=10)
        self.pre_pub = rospy.Publisher('TS_list_predict', ShipInfo, queue_size=10)

        self.ship_ID = []
        self.waypoint_idx = 0
        self.len_waypoint_info = 0
        self.waypoint_dict = dict()
        self.ts_spd_dict = dict()

        self.TS_WP_index = []

        self.static_obstacle_info = []
        self.static_point_info = []

        self.target_heading_list = []

        self.start_time = time.time()
        self.ais_delay = rospy.get_param("ais_delay")

    def wp_callback(self, wp):
        self.len_waypoint_info = len(wp.group_wpts_info)
        wp_dic = dict()
        for i in range(self.len_waypoint_info):
            shipID = wp.group_wpts_info[i].shipID
            wp_dic['{}'.format(shipID)] = wp.group_wpts_info[i]

        self.waypoint_dict = wp_dic

    def OP_callback(self, operation):
        self.ship_ID = list(operation.m_nShipID)

        self.Pos_X  = operation.m_fltPos_X
        self.Pos_Y  = operation.m_fltPos_Y
        self.Vel_U  = operation.m_fltVel_U

        self.delta_deg = operation.m_fltRudderAngleFeedSTBD # deg.

        raw_psi = np.asanyarray(operation.m_fltHeading)
        self.Heading = raw_psi % 360

    def static_unavailable_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB.group_boundary_info)
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
            static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
            
        static_ob_info = []
        
        for k in range(len(static_ob_list_x)):
            for l in range(len(static_ob_list_x[k])):
                if l == 0:
                    pass
                else:
                    static_ob_info.append(static_ob_list_x[k][l-1])
                    static_ob_info.append(static_ob_list_y[k][l-1])
                    static_ob_info.append(static_ob_list_x[k][l])
                    static_ob_info.append(static_ob_list_y[k][l])
                    
        self.static_unavailable_info = static_ob_info
        
    def static_available_callback(self, static_OB):
        self.len_static_obstacle_info = len(static_OB.group_boundary_info)
        static_ob_list_x = []
        static_ob_list_y = []
        for i in range(self.len_static_obstacle_info):
            static_ob_list_x.append(list(static_OB.group_boundary_info[i].area_x))
            static_ob_list_y.append(list(static_OB.group_boundary_info[i].area_y))
        
        static_ob_info = []
        
        for k in range(len(static_ob_list_x)):
            for l in range(len(static_ob_list_x[k])):
                if l == 0:
                    pass
                else:
                    static_ob_info.append(static_ob_list_x[k][l-1])
                    static_ob_info.append(static_ob_list_y[k][l-1])
                    static_ob_info.append(static_ob_list_x[k][l])
                    static_ob_info.append(static_ob_list_y[k][l])
                    
        self.static_available_info = static_ob_info

    def path_out_publish(self, pub_list):
        inha = col()
        inha.nship_ID = pub_list[0]
        inha.modifyWayPoint = pub_list[1]
        inha.numOfWayPoint  = pub_list[2]
        inha.latOfWayPoint = pub_list[3]
        inha.longOfWayPoint = pub_list[4]
        inha.speedOfWayPoint = pub_list[5]
        inha.ETAOfWayPoint = round(pub_list[6], 3)
        inha.EDAOfWayPoint = round(pub_list[7], 3)
        inha.error = pub_list[8]
        inha.errorCode = pub_list[9]
        inha.targetSpeed = round(pub_list[10], 3)
        inha.targetCourse = round(pub_list[11], 3)
        
        self.WP_pub.publish(inha)

    def vis_out(self, pub_list):
        vis = vis_info()
        vis.nship_ID = pub_list[0]
        vis.collision_cone = pub_list[1]
        vis.v_opt = pub_list[2]
        vis.local_goal = pub_list[3]

        self.Vis_pub.publish(vis)

    def cri_out(self, pub_list):
        cri = cri_info()
        cri.DCPA = pub_list[0]
        cri.TCPA = pub_list[1]
        cri.UDCPA = pub_list[2]
        cri.UTCPA = pub_list[3]
        cri.UD = pub_list[4]
        cri.UB = pub_list[5]
        cri.UK = pub_list[6]
        cri.CRI = pub_list[7]
        cri.Rf = pub_list[8]
        cri.Ra = pub_list[9]
        cri.Rs = pub_list[10]
        cri.Rp = pub_list[11]
        cri.encounter_classification = pub_list[12]
        # print(cri.encounter_classification)

        self.cri_pub.publish(cri)

    def vo_out(self, pub_list):
        vo = VO_info()
        vo.V_opt = pub_list[0]
        vo.Collision_cone = pub_list[1]

        self.VO_pub.publish(vo)

    def ori_out(self, pub_list):
        for ship_ID, ship_data in pub_list.items():
            message = ShipInfo()
            message.Ship_ID = ship_data['Ship_ID']
            message.Pos_X = ship_data['Pos_X']
            message.Pos_Y = ship_data['Pos_Y']
            message.Vel_U = ship_data['Vel_U']
            message.Heading = ship_data['Heading']

            self.ori_pub.publish(message)

    def del_out(self, pub_list):
        for ship_ID, ship_data in pub_list.items():
            message = ShipInfo()
            message.Ship_ID = ship_data['Ship_ID']
            message.Pos_X = ship_data['Pos_X']
            message.Pos_Y = ship_data['Pos_Y']
            message.Vel_U = ship_data['Vel_U']
            message.Heading = ship_data['Heading']

            self.del_pub.publish(message)

    def pre_out(self, pub_list):
        for ship_ID, ship_data in pub_list.items():
            message = ShipInfo()
            message.Ship_ID = ship_data['Ship_ID']
            message.Pos_X = ship_data['Pos_X']
            message.Pos_Y = ship_data['Pos_Y']
            message.Vel_U = ship_data['Vel_U']
            message.Heading = ship_data['Heading']

            self.pre_pub.publish(message)

def main():  
    rospack = rospkg.RosPack()  
    package_path = rospack.get_path('kass_inha')
    VO_operate = rospy.get_param("shipInfo_all/ship1_info/include_inha_modules")

    update_rate = rospy.get_param("update_rate")
    dt =  rospy.get_param("mmg_dt")

    timestr = time.strftime("%Y%m%d-%H%M%S")
    # path = "/home/phl/문서/" + timestr + ".csv"
    # path = "/home/phlyoo/Documents/" + timestr + ".csv"
    # header = ['ShipID', 'Pos_X', 'Pos_Y', 'wp_x', 'wp_y', 'Vel_U', 'Vx', 'Vy', 'Heading', 'desired_heading', 'encounter', 'encounterMMSI']
    # header = ['RD','RC', 'K', 'DCPA','TCPA', 'UDCPA', 'UTCPA', 'UD', 'UB', 'UK', 'CRI', 'Rf', 'Ra', 'Rs', 'Rp', 'ENC', 'V_opt', 'pub_collision_cone', 'VO_operate']

    # file = open(path, 'a', newline='')
    # writer = csv.writer(file)
    # writer.writerow(header)

    node_Name = "vessel_node1"
    rospy.init_node("{}".format(node_Name), anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    OS_ID = rospy.get_param("shipInfo_all/ship1_info/ship_ID")
    TS_ID = []
    desired_spd_list = []
    pub_collision_cone = []
    V_opt = []

    # 자선의 정보
    OS_scale = rospy.get_param("shipInfo_all/ship1_info/ship_scale")
    target_speed = rospy.get_param("shipInfo_all/ship1_info/target_speed")  * 0.5144 / sqrt(OS_scale)
    ship_L = rospy.get_param("shipInfo_all/ship1_info/ship_L")
    
    data = data_inNout()
    
    t = 0
    waypointIndex = 0
    targetspdIndex = 0    

    encounter = None
    encounterMMSI = []

# UKF part
#####################################################################################################################
    
    ukf_dt = rospy.get_param('ukf_dt')

    last_publish_time = rospy.Time.now()  # 마지막으로 발행한 시간을 초기화
    delay = rospy.get_param('ais_delay')
    publish_interval = rospy.Duration(delay)  # 발행 주기를 5초로 설정
    
    ukf_instance = {}    
    TS_list_d={}
    TS_list={}
    predicted_state = []
    previous_input_list = {}

    first_loop = True
    first_publish = True
    heading_diff = 0.0

#####################################################################################################################

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()  # 현재 시간을 계속 추적
        Local_PP = VO_module()

        if len(data.ship_ID) == 0:
            print("========= Waiting for `/frm_info` topic subscription in {}=========".format(node_Name))
            rate.sleep()
            continue

        if data.len_waypoint_info == 0:
            print("========= Waiting for `/waypoint_info` topic subscription in {} =========".format(node_Name))
            rate.sleep()
            continue

        inha = Inha_dataProcess(
            data.ship_ID,
            data.Pos_X, 
            data.Pos_Y, 
            data.Vel_U, 
            data.Heading, 
            data.waypoint_dict
            )

        wpts_x_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_x)
        wpts_y_os = list(data.waypoint_dict['{}'.format(OS_ID)].wpts_y)
        Local_goal = [wpts_x_os[waypointIndex], wpts_y_os[waypointIndex]]

        ship_list, ship_ID = inha.ship_list_container(OS_ID)
        OS_list, TS_list_ori = inha.classify_OS_TS(ship_list, ship_ID, OS_ID)
        TS_ID = TS_list_ori.keys()
        # TODO : why do this?

# UKF part
#####################################################################################################################
        
        print("TS_list_ori: ", TS_list_ori)
        if first_loop:
            for ts_ID in TS_ID:
                ukf_instance[ts_ID] = UKF()

            first_loop = False

        for ts_ID in TS_ID:
            if(current_time - last_publish_time >= publish_interval) or first_publish:
                TS_list_d[ts_ID] = TS_list_ori[ts_ID]
                last_publish_time = current_time

            heading_diff = TS_list_d[ts_ID]['Heading'] - TS_list_ori[ts_ID]['Heading']

            if heading_diff < 0:
                heading_diff += 360
            elif heading_diff >= 360:
                heading_diff -= 360

            if abs(heading_diff) >= 15:
                TS_list_d[ts_ID] = TS_list_ori[ts_ID]

            else:
                TS_list_d[ts_ID] = TS_list_d[ts_ID]

        first_publish = False
        print("delay: ",TS_list_d)

        input_list = []
        TS_list = copy.deepcopy(TS_list_d)
        for ts_ID in TS_ID:
            input_list.append(TS_list_d[ts_ID]['Pos_X'])
            input_list.append(TS_list_d[ts_ID]['Pos_Y'])
            input_list.append(TS_list_d[ts_ID]['Vel_U'])
            input_list.append(TS_list_d[ts_ID]['Heading'])

            if ts_ID in previous_input_list and previous_input_list[ts_ID] == input_list:
                predicted_state, covariance = ukf_instance[ts_ID].predict(ukf_dt)

            else:
                predicted_state, covariance= ukf_instance[ts_ID].update(input_list, ukf_dt)

            previous_input_list[ts_ID] = input_list.copy()

            update_keys = ['Pos_X', 'Pos_Y', 'Vel_U', 'Heading']

            for key, value in zip(update_keys, predicted_state):
                if key in TS_list[ts_ID]:
                    TS_list[ts_ID][key] = value

        for ts_ID in TS_ID:
            X_diff = TS_list[ts_ID]["Pos_X"] - TS_list_ori[ts_ID]["Pos_X"]
            Y_diff = TS_list[ts_ID]["Pos_Y"] - TS_list_ori[ts_ID]["Pos_Y"]
            U_diff = TS_list[ts_ID]["Vel_U"] - TS_list_ori[ts_ID]["Vel_U"]
            H_diff = TS_list[ts_ID]["Heading"] - TS_list_ori[ts_ID]["Heading"]
        
        print("predict: ",TS_list)
        print("\n")
        print(X_diff)

#####################################################################################################################

        OS_Vx, OS_Vy = inha.U_to_vector_V(OS_list['Vel_U'], OS_list['Heading'])

        OS_list['V_x'] = OS_Vx
        OS_list['V_y'] = OS_Vy

        _, local_goal_EDA = inha.eta_eda_assumption(Local_goal, OS_list, target_speed)

        V_des = Local_PP.vectorV_to_goal(OS_list, Local_goal, target_speed)

        TS_list = inha.TS_info_supplement(
            OS_list, 
            TS_list,
            )
        
        TS_RD_temp = []
        TS_RC_temp = []
        TS_K_temp = []
        TS_DCPA_temp = []
        TS_TCPA_temp = []
        TS_UDCPA_temp = []
        TS_UTCPA_temp = []
        TS_UD_temp = []
        TS_UB_temp = []
        TS_UK_temp = []
        TS_CRI_temp = []
        TS_Rf_temp = []
        TS_Ra_temp = []
        TS_Rs_temp = []
        TS_Rp_temp = []
        TS_ENC_temp = []

        encounterMMSI = []
        TS_list_copy = {}

        for ts_ID in TS_ID:
            temp_RD = TS_list[ts_ID]['RD']
            TS_RD_temp.append(temp_RD)
            
            temp_RC = TS_list[ts_ID]['RC']
            TS_RC_temp.append(temp_RC)

            temp_K = TS_list[ts_ID]['K']
            TS_K_temp.append(temp_K)

            temp_DCPA = TS_list[ts_ID]['DCPA']
            TS_DCPA_temp.append(temp_DCPA)

            temp_TCPA = TS_list[ts_ID]['TCPA']
            TS_TCPA_temp.append(temp_TCPA)

            temp_UDCPA = TS_list[ts_ID]['UDCPA']
            TS_UDCPA_temp.append(temp_UDCPA)
            
            temp_UTCPA = TS_list[ts_ID]['UTCPA']
            TS_UTCPA_temp.append(temp_UTCPA)

            temp_UD = TS_list[ts_ID]['UD']
            TS_UD_temp.append(temp_UD)

            temp_UB = TS_list[ts_ID]['UB']
            TS_UB_temp.append(temp_UB)

            temp_UK = TS_list[ts_ID]['UK']
            TS_UK_temp.append(temp_UK)

            temp_cri = TS_list[ts_ID]['CRI']
            TS_CRI_temp.append(temp_cri)

            temp_Rf = TS_list[ts_ID]['Rf']
            TS_Rf_temp.append(temp_Rf)

            temp_Ra = TS_list[ts_ID]['Ra']
            TS_Ra_temp.append(temp_Ra)

            temp_Rs = TS_list[ts_ID]['Rs']
            TS_Rs_temp.append(temp_Rs)

            temp_Rp = TS_list[ts_ID]['Rp']
            TS_Rp_temp.append(temp_Rp)

            temp_enc = TS_list[ts_ID]['status']
            TS_ENC_temp.append(temp_enc)
            # print(temp_enc)

            distance = sqrt((OS_list["Pos_X"]-TS_list[ts_ID]["Pos_X"])**2+(OS_list["Pos_Y"]-TS_list[ts_ID]["Pos_Y"])**2)

            if distance <= rospy.get_param("detecting_distance"):
                if ts_ID not in TS_list_copy:
                    TS_list_copy[ts_ID] = TS_list[ts_ID]
                    encounterMMSI.append(ts_ID)
                    # print(f"TS was detected at around OS: {ts_ID}")
            else:
                if ts_ID in TS_list_copy:
                    del TS_list_copy[ts_ID]
                    encounterMMSI.remove(ts_ID)
                    # print(f"TS moved out of range: {ts_ID}")

        TS_ID = encounterMMSI
        TS_list = TS_list_copy

        # print("distance : ", distance)
        # print("DCPA: ", temp_DCPA)
        
        if len(encounterMMSI) == 0:
            encounter = False
        else:
            encounter = True

        # print("TS_ID:           ", TS_ID)
        # print("encounter:       ", encounter)
        # print("encounterMMSI:   ", encounterMMSI)

        V_selected, pub_collision_cone = Local_PP.VO_update(
            OS_list, 
            TS_list, 
            V_des, 
            data.static_obstacle_info,
            data.static_point_info
            )

        desired_spd_list = []
        desired_heading_list = []

        wp = inha.waypoint_generator(OS_list, V_selected, dt)
        wp_x = wp[0]
        wp_y = wp[1]

        if VO_operate:
            eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
            temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
            desired_spd_list.append(temp_spd)
            desired_heading_list.append(temp_heading_deg)
            desired_spd = desired_spd_list[0]
            desired_heading = desired_heading_list[0]
        
        else:
            V_selected = V_des
            eta, eda = inha.eta_eda_assumption(wp, OS_list, target_speed)            
            temp_spd, temp_heading_deg = inha.desired_value_assumption(V_selected)
            desired_spd_list = list(data.waypoint_dict['{}'.format(OS_ID)].target_spd)
            desired_heading_list.append(temp_heading_deg)
            desired_spd = desired_spd_list[targetspdIndex]
            desired_heading = desired_heading_list[0]

        if t%10 ==0:
            pass

        t += 1

        if len(data.target_heading_list) != rospy.get_param('filter_length'):
            data.target_heading_list.append(desired_heading)
        
        else:
            del data.target_heading_list[0]

        sum_of_heading = 0
        real_target_heading = 0
        for i in data.target_heading_list:
            sum_of_heading = sum_of_heading + i

        if len(data.target_heading_list) >= 2:
            if data.target_heading_list[len(data.target_heading_list)-1]*data.target_heading_list[len(data.target_heading_list)-2] < 0:
                data.target_heading_list = [data.target_heading_list[-1]]
                real_target_heading = desired_heading
            else:
                real_target_heading = sum_of_heading/len(data.target_heading_list)

        OS_pub_list = [
            int(OS_ID), 
            False,
            waypointIndex,
            # int(data.waypoint_idx), # 부경대 i_way
            # data.waypoint_idx, # kriso
            [wp_x], 
            [wp_y],  
            desired_spd_list, 
            eta, 
            eda, 
            False, 
            0, 
            desired_spd, 
            # desired_heading
            real_target_heading,
            ]

        vis_pub_list = [
            int(OS_ID), 
            pub_collision_cone,
            V_opt,
            Local_goal
        ]

        cri_pub_list = [
            TS_DCPA_temp,
            TS_TCPA_temp,
            TS_UDCPA_temp,
            TS_UTCPA_temp,
            TS_UD_temp,
            TS_UB_temp,
            TS_UK_temp,
            TS_CRI_temp,
            TS_Rf_temp,
            TS_Ra_temp,
            TS_Rs_temp,
            TS_Rp_temp,
            TS_ENC_temp,
        ]

        vo_pub_list = [
            V_selected,
            pub_collision_cone
        ]

        ship_dic2list = list(OS_list.values())

        # savedata_list = [
        #     TS_RD_temp,
        #     TS_RC_temp,
        #     TS_K_temp,
        #     TS_DCPA_temp,
        #     TS_TCPA_temp,
        #     TS_UDCPA_temp,
        #     TS_UTCPA_temp,
        #     TS_UD_temp,
        #     TS_UB_temp,
        #     TS_UK_temp,
        #     TS_CRI_temp,
        #     TS_Rf_temp,
        #     TS_Ra_temp,
        #     TS_Rs_temp,
        #     TS_Rp_temp,
        #     TS_ENC_temp,
        #     V_selected,
        #     pub_collision_cone,
        #     VO_operate
        # ]

        # savedata_list = [
        #     int(OS_ID),
        #     ship_dic2list[1],
        #     ship_dic2list[2],
        #     wp_x,
        #     wp_y,
        #     ship_dic2list[3],
        #     OS_Vx,
        #     OS_Vy,
        #     ship_dic2list[4],
        #     desired_heading,
        #     encounter,
        #     encounterMMSI
        # ]

        # writer.writerow(savedata_list)

        data.path_out_publish(OS_pub_list)
        data.vis_out(vis_pub_list)
        data.cri_out(cri_pub_list)
        data.vo_out(vo_pub_list)

        data.ori_out(TS_list_ori)
        data.del_out(TS_list_d)
        data.pre_out(TS_list)

        if local_goal_EDA < 5 * (ship_L/OS_scale) :
            waypointIndex = (waypointIndex + 1) % len(wpts_x_os)
            targetspdIndex = waypointIndex

        rate.sleep()
        
        # print("Loop end time: ", time.time() - startTime)
        # print("================ Node 1 loop end ================\n")

    # file.close()

    rospy.spin()

if __name__ == '__main__':
    main()