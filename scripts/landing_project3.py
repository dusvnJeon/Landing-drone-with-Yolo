#!/usr/bin/env python

import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
from object_detector.msg import States  # 타겟 검출 메시지 타입

# 드론 상태를 저장할 변수
current_pose = None
target_detected = None

def pose_callback(data):
    global current_pose
    current_pose = data

def target_callback(data):
    global target_detected
    target_detected = data

def set_land_mode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode_service(custom_mode='AUTO.LAND')
        if response.mode_sent:
            rospy.loginfo("AUTO.LAND mode set successfully")
        else:
            rospy.logwarn("Failed to set AUTO.LAND mode")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('landing_controller', anonymous=True)
    
    # 드론 위치 구독자 설정
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    
    # 타겟 검출 구독자 설정
    rospy.Subscriber('/states', States, target_callback)
    
    # 드론 제어 명령 발행자 설정
    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    
    rate = rospy.Rate(20)  # 20Hz

    while not rospy.is_shutdown():
        if current_pose is None:
            rate.sleep()
            continue
        
        control_msg = PositionTarget()
        control_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        control_msg.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE
        
        # z 위치가 0.5 미터일 때 착륙 모드 설정
        if current_pose.pose.position.z <= 0.5:
            control_msg.velocity.x = 0
            control_msg.velocity.y = 0
            control_msg.velocity.z = -0.1
        elif target_detected is not None:
            # 타겟의 중심점과 카메라 중심점의 오차 계산
            target_x = target_detected.Xc
            target_y = target_detected.Yc
            image_center_x = 640 / 2
            image_center_y = 480 / 2
            error_x = target_x - image_center_x
            error_y = target_y - image_center_y

            # 드론 속도 조정을 위한 제어 명령 생성
            control_msg.velocity.x = - (error_y * 0.01)  # y축 오차를 x축 속도로 변환
            control_msg.velocity.y = - (error_x * 0.01)  # x축 오차를 y축 속도로 변환
            control_msg.velocity.z = -0.5  # 고도 낮추기
        else:
            # 타겟이 검출되지 않으면 속도 유지
            control_msg.velocity.x = 0
            control_msg.velocity.y = 0
            control_msg.velocity.z = 0  # 속도 유지

        setpoint_pub.publish(control_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
