#! /usr/bin/env python

import rospy
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    global_pos_pub = rospy.Publisher("mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    target = GlobalPositionTarget()
    target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT  # Use relative altitude frame
    target.type_mask = GlobalPositionTarget.IGNORE_VX + GlobalPositionTarget.IGNORE_VY + \
                       GlobalPositionTarget.IGNORE_VZ + GlobalPositionTarget.IGNORE_AFX + \
                       GlobalPositionTarget.IGNORE_AFY + GlobalPositionTarget.IGNORE_AFZ + \
                       GlobalPositionTarget.IGNORE_YAW + GlobalPositionTarget.IGNORE_YAW_RATE

    target.latitude = 47.397742
    target.longitude = 8.5455934
    target.altitude = 540.3069164476947

    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break
        global_pos_pub.publish(target)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        global_pos_pub.publish(target)
        rate.sleep()
