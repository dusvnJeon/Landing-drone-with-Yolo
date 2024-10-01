#!/usr/bin/env python

import rospy
import rosnode
from geometry_msgs.msg import PoseStamped

nodes_to_kill = []

def callback(msg):
    # 메시지 헤더에서 발행자 노드 이름을 추출
    node_name = msg.header.frame_id
    if node_name and node_name not in nodes_to_kill:
        nodes_to_kill.append(node_name)
        rospy.loginfo(f"Found node: {node_name}")

def kill_nodes(event):
    for node in nodes_to_kill:
        if node != rospy.get_name():  # 현재 노드를 제외하고 종료
            try:
                rosnode.kill_nodes([node])
                rospy.loginfo(f"Killed node: {node}")
            except rosnode.ROSNodeException as e:
                rospy.logerr(f"Failed to kill node: {node}. Error: {e}")

if __name__ == '__main__':
    rospy.init_node('node_killer')
    topic_name = '/mavros/setpoint_position/local'

    rospy.Subscriber(topic_name, PoseStamped, callback)

    rospy.loginfo(f"Listening to topic: {topic_name}")

    # 10초마다 kill_nodes 함수를 호출합니다.
    rospy.Timer(rospy.Duration(10), kill_nodes)

    rospy.spin()
