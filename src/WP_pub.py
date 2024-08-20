#!/usr/bin/env python3
import rospy
import sys
import os
print(sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__)))))
from functions.InfoLoader import InfoLoader
from udp_msgs.msg import group_wpts_info, wpts_info

def main():
    update_rate = rospy.get_param("update_rate")
    pub = rospy.Publisher('/waypoint_info', group_wpts_info, queue_size=10)

    shipsInfo = InfoLoader(rospy.get_param("shipInfo_all"))

    wptsInfo = dict()
    for shipName in shipsInfo.shipName_all:
        wptsInfo[shipName] = wpts_info()

    rospy.init_node('WP_generator', anonymous=False)
    rate = rospy.Rate(update_rate) # 2초에 한번씩 데이터 줘라

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():   
        topic = group_wpts_info()
        topic.header.stamp = rospy.Time.now() - start_time  # http://wiki.ros.org/rospy/Overview/Time 
        topic.header.frame_id = "group waypoint list"

        for shipName in shipsInfo.shipName_all:
            wptsInfo[shipName].shipID = rospy.get_param("shipInfo_all/" + shipName + "_info/ship_ID")
            wptsInfo[shipName].wpts_x = rospy.get_param("waypoint_List/wpts_x_" + shipName)
            wptsInfo[shipName].wpts_y = rospy.get_param("waypoint_List/wpts_y_" + shipName)
            wptsInfo[shipName].target_spd = rospy.get_param("target_spd_List/target_speed_" + shipName)


        topic.group_wpts_info = [wptsInfo[shipName] for shipName in shipsInfo.shipName_all]

        pub.publish(topic)
        rospy.loginfo(topic)        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
