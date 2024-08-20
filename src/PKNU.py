#!/usr/bin/env python
from udp_col_msg.msg import path_output
import rospy
class PKNU:
    """ROS1의 한계로 node간의 쌍방향 topic이 안되는 문제점을 해결하기 위한 중계 node로 이해하면 됨
    
    Note:
        `/path_out_inha`를 그대로 KRISO node로 전달하며, 실제 제어기는 KRISO node에서 구현됨
    """
    def __init__(self):
        rospy.Subscriber('/path_out_inha', path_output, self.topic_callback)
        self.topic_pub = rospy.Publisher('/path_out_inha2', path_output, queue_size=10)
        self.len_ship = 0
        self.list = []
        self.topics = []

    def topic_callback(self, WP):
        self.len_ship = len(WP.pathData)
        topic_list = []
        for i in range(self.len_ship):
            topics = WP.pathData[i]  
            topic_list.append(topics)

        self.list = topic_list

    def topic_publish(self):
        inha = path_output()

        topic_list = []
        for i in range(self.len_ship):
            topics = self.list[i]
            topic_list.append(topics)

        inha.pathData = topic_list
        self.topic_pub.publish(inha)

def main():   
    update_rate = rospy.get_param("update_rate")
    rospy.init_node('PKNU', anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    pknu = PKNU()
    while not rospy.is_shutdown():
        if pknu.len_ship == 0:
            print("========= Waiting for `/path_out_inha` topic subscription =========")
            continue

        pknu.topic_publish()
        rate.sleep()
        
    rospy.spin()


if __name__ == '__main__':
    main()
