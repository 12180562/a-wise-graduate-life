#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from functions.InfoLoader import InfoLoader
from udp_col_msg.msg import path_output, col, vis_info, group_vis_info
import rospy

class data_inNout:
    """inha_module의 data 송신을 위해 필요한 함수들이 정의됨"""
    def __init__(self):
        
        shipsInfo = InfoLoader(rospy.get_param("shipInfo_all"))
        self.num_ships = shipsInfo.num_ships
        # print(self.num_ships)

        if self.num_ships == 1:
            rospy.Subscriber('/vessel1_info', col, self.ship1_callback) 
            self.ship1_info = []
        elif self.num_ships == 2:
            rospy.Subscriber('/vessel1_info', col, self.ship1_callback)
            rospy.Subscriber('/vessel2_info', col, self.ship2_callback)
            self.ship1_info = []
            self.ship2_info = []
        elif self.num_ships == 3:
            rospy.Subscriber('/vessel1_info', col, self.ship1_callback)
            rospy.Subscriber('/vessel2_info', col, self.ship2_callback)
            rospy.Subscriber('/vessel3_info', col, self.ship3_callback)
            self.ship1_info = []
            self.ship2_info = []
            self.ship3_info = []
        elif self.num_ships == 4:
            rospy.Subscriber('/vessel1_info', col, self.ship1_callback)
            rospy.Subscriber('/vessel2_info', col, self.ship2_callback)
            rospy.Subscriber('/vessel3_info', col, self.ship3_callback)
            rospy.Subscriber('/vessel4_info', col, self.ship4_callback)
            self.ship1_info = []
            self.ship2_info = []
            self.ship3_info = []
            self.ship4_info = []
        elif self.num_ships == 5:
            rospy.Subscriber('/vessel1_info', col, self.ship1_callback)
            rospy.Subscriber('/vessel2_info', col, self.ship2_callback)
            rospy.Subscriber('/vessel3_info', col, self.ship3_callback)
            rospy.Subscriber('/vessel4_info', col, self.ship4_callback)
            rospy.Subscriber('/vessel5_info', col, self.ship5_callback)
            self.ship1_info = []
            self.ship2_info = []
            self.ship3_info = []
            self.ship4_info = []
            self.ship5_info = []

        # TODO: Remove hardcoding
        rospy.Subscriber('/vis1_info', vis_info, self.vis1_callback) 
        rospy.Subscriber('/vis2_info', vis_info, self.vis2_callback)
        self.vis1_info = []
        self.vis2_info = []

        self.WP_pub = rospy.Publisher('/path_out_inha', path_output, queue_size=10)
        self.vis_pub = rospy.Publisher('/group_vis_info', group_vis_info, queue_size=10)


    def ship1_callback(self, ship_info):
        self.ship1_info = [ship_info]

    def ship2_callback(self, ship_info):
        self.ship2_info = [ship_info]   

    def ship3_callback(self, ship_info):
        self.ship3_info = [ship_info]   

    def ship4_callback(self, ship_info):
        self.ship4_info = [ship_info]   

    def ship5_callback(self, ship_info):
        self.ship5_info = [ship_info] 

    def vis1_callback(self, vis_info):
        self.vis1_info = [vis_info]

    def vis2_callback(self, vis_info):
        self.vis2_info = [vis_info]


    def path_out_publish(self, pub_list):
        ''' publish `/path_out_inha`
        
        Note :
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        '''
        topic = path_output()
        topic.header.stamp = rospy.Time.now() # http://wiki.ros.org/rospy/Overview/Time 
        topic.header.frame_id = "output of inha" ## 여기바꾸면 부경대에 알려줄 것! (KAIST/INHA code 분기시 사용함)
                
        topic.pathData = pub_list ## [OS, TS1, TS2, TS3 ....]
        self.WP_pub.publish(topic)
        # rospy.loginfo(topic)

    def group_vis_publish(self, pub_list):
        ''' publish `/path_out_inha`
        
        Note :
            pub_list = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desiredU, desiredHeading, isNeedCA, "Encounter status"]
        '''
        vis_topic = group_vis_info()
        vis_topic.visData = pub_list
        self.vis_pub.publish(vis_topic)

def main():  
    # with open('./params/KASS_Coefficient.yaml') as f:
    #     KASS_coefficient = yaml.safe_load(f)

    # with open('./params/main_parameter.yaml') as f:
    #     params = yaml.safe_load(f)

    # update_rate = params['update_rate']              # 혜리 수정
    update_rate = rospy.get_param("update_rate")
    rospy.init_node('Data_integration', anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    data = data_inNout()

    while not rospy.is_shutdown():
        # print(data.num_ships)
        if data.num_ships == 1:
            if len(data.ship1_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship1_info` topic subscription =========")
                rate.sleep()
                continue

        elif data.num_ships == 2:
            if len(data.ship1_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship1_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship2_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship2_info` topic subscription =========")
                rate.sleep()
                continue

        elif data.num_ships == 3:
            if len(data.ship1_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship1_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship2_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship2_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship3_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship3_info` topic subscription =========")
                rate.sleep()
                continue

        elif data.num_ships == 4:
            if len(data.ship1_info) == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship1_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship2_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship2_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship3_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship3_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship4_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship4_info` topic subscription =========")
                rate.sleep()
                continue

        elif data.num_ships == 5:
            if len(data.ship1_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship1_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship2_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship2_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship3_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship3_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship4_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship4_info` topic subscription =========")
                rate.sleep()
                continue

            if len(data.ship5_info) == 0:
                ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
                print("========= Waiting for `/ship5_info` topic subscription =========")
                rate.sleep()
                continue

        # print("\n=============== data_integratioin loop start =============\n")

        

        if data.num_ships == 1:
            ship1_info = data.ship1_info[0]
            pub_list_path_output = [ship1_info]
        elif data.num_ships == 2:
            ship1_info = data.ship1_info[0]
            ship2_info = data.ship2_info[0]
            pub_list_path_output = [ship1_info, ship2_info]
        elif data.num_ships == 3:
            ship1_info = data.ship1_info[0]
            ship2_info = data.ship2_info[0]
            ship3_info = data.ship3_info[0]
            pub_list_path_output = [ship1_info, ship2_info, ship3_info]
        elif data.num_ships == 4:
            ship1_info = data.ship1_info[0]
            ship2_info = data.ship2_info[0]
            ship3_info = data.ship3_info[0]
            ship4_info = data.ship4_info[0]
            pub_list_path_output = [ship1_info, ship2_info, ship3_info, ship4_info]
        elif data.num_ships == 5:
            ship1_info = data.ship1_info[0]
            ship2_info = data.ship2_info[0]
            ship3_info = data.ship3_info[0]
            ship4_info = data.ship4_info[0]
            ship5_info = data.ship5_info[0]
            pub_list_path_output = [
                ship1_info,
                ship2_info,
                ship3_info,
                ship4_info,
                ship5_info,
                ]
        
        # TODO: Remove hardcoding
        # vis1_info = data.vis1_info[0]
        # vis2_info = data.vis2_info[0]
        # vis3_info = data.vis3_info[0]
        # vis4_info = data.vis4_info[0]
        # vis5_info = data.vis5_info[0]

        # pub_list_vis_output = [vis1_info, vis2_info] 

        data.path_out_publish(pub_list_path_output)     
        # data.group_vis_publish(pub_list_vis_output)
        rate.sleep()

        # print("\n============= data_integration loop end ============\n")

    rospy.spin()


if __name__ == '__main__':
    main()
