#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from std_msgs.msg import String

current_state = State()
current_offboard_override = False
loiter_enabled = False

def state_cb(state):
    global current_state
    current_state = state

def override_cb(msg):
    global current_offboard_override
    current_offboard_override = msg.data.lower() == 'true'

def set_mode_client(base_mode, custom_mode):
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode(base_mode, custom_mode)
        return response.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

def main():
    global current_offboard_override, loiter_enabled

    rospy.init_node('set_loiter_mode_node', anonymous=True)
    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/offboard_override', String, override_cb)

    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        if not loiter_enabled and current_state.mode != "AUTO.LOITER" and not current_offboard_override:
            success = set_mode_client(0, 'AUTO.LOITER')
            if success:
                rospy.loginfo("AUTO.LOITER mode set successfully")
                loiter_enabled = True
            else:
                rospy.logerr("Failed to set AUTO.LOITER mode")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
