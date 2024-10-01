#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

def set_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode_service(custom_mode=mode)
        if response.mode_sent:
            rospy.loginfo(f"Mode set to {mode}")
        else:
            rospy.logwarn(f"Failed to set mode to {mode}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('drone_landing_node')
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU connected")

    # Wait until the drone is in AUTO.LOITER mode
    while not rospy.is_shutdown():
        rospy.loginfo("Waiting for AUTO.LOITER mode...")
        rate.sleep()

    rospy.loginfo("Drone is in AUTO.LOITER mode")

    # Set mode to AUTO.LAND
    set_mode("AUTO.LAND")

    rospy.spin()
