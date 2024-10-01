#!/usr/bin/env python

import rospy
import math
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from object_detector.msg import States  # 타겟 검출 메시지 타입

# 드론 상태를 저장할 변수
current_pose = None
target_detected = None
target_reached = False  # 타겟 중심에 도달했는지 여부
yaw_aligned = False  # Yaw가 정렬되었는지 여부

def pose_callback(data):
    global current_pose
    current_pose = data

def target_callback(data):
    global target_detected
    target_detected = data

def main():
    global target_reached, yaw_aligned
    
    rospy.init_node('landing_controller', anonymous=True)
    
    # 드론 위치 구독자 설정
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    
    # 타겟 검출 구독자 설정
    rospy.Subscriber('/states', States, target_callback)
    
    # 드론 제어 명령 발행자 설정
    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    
    rate = rospy.Rate(20)  # 20Hz

    while not rospy.is_shutdown():
        if current_pose is None or target_detected is None:
            rate.sleep()
            continue
        
        control_msg = PositionTarget()
        control_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        control_msg.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        
        if not target_reached:
            # 타겟의 중심점과 카메라 중심점의 오차 계산
            target_x = target_detected.Xc
            target_y = target_detected.Yc
            image_center_x = 640 / 2
            image_center_y = 480 / 2
            error_x = target_x - image_center_x
            error_y = target_y - image_center_y
            
            # 오차가 특정 값 이하일 때 타겟에 도달했다고 판단
            if abs(error_x) < 10 and abs(error_y) < 10:
                target_reached = True
                rospy.loginfo("Target reached. Beginning yaw alignment.")
            else:
                # 드론 위치 조정을 위한 제어 명령 생성
                control_msg.position.x = current_pose.pose.position.x - (error_y * 0.01)  # y축 오차를 x축 위치로 변환
                control_msg.position.y = current_pose.pose.position.y - (error_x * 0.01)  # x축 오차를 y축 위치로 변환
                control_msg.position.z = current_pose.pose.position.z  # 고도 유지
                
        elif target_reached and not yaw_aligned:
            # Theta를 사용하여 yaw 정렬
            theta_rad = math.radians(target_detected.Theta)
            if abs(theta_rad) < math.radians(1):  # Theta가 1도 이하일 때 정렬 완료
                yaw_aligned = True
                rospy.loginfo("Yaw aligned. Beginning descent.")
            else:
                control_msg.position.x = current_pose.pose.position.x
                control_msg.position.y = current_pose.pose.position.y
                control_msg.position.z = current_pose.pose.position.z  # 고도 유지
                control_msg.yaw = current_pose.pose.orientation.z - theta_rad * 0.1  # Theta를 yaw로 변환하여 제어 명령 생성
                control_msg.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        
        elif yaw_aligned:
            # z 위치가 2.5 미터일 때 하강 속도 설정
            if current_pose.pose.position.z > 2.5:
                control_msg.position.x = current_pose.pose.position.x - (error_y * 0.01)
                control_msg.position.y = current_pose.pose.position.y - (error_x * 0.01)
                control_msg.position.z = current_pose.pose.position.z - 0.3  # 하강 속도 설정
            else:
                control_msg.position.x = current_pose.pose.position.x
                control_msg.position.y = current_pose.pose.position.y
                control_msg.position.z = current_pose.pose.position.z - 0.5  # 하강 속도 설정

        setpoint_pub.publish(control_msg)
        
        # 착륙 완료 조건 체크 (고도 < 0.1미터)
        if current_pose.pose.position.z < 0.1:
            rospy.loginfo("Landing complete")
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
