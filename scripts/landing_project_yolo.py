#!/usr/bin/env python

import rospy
import math
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from object_detector.msg import States  # 타겟 검출 메시지 타입
from mavros_msgs.srv import SetMode, SetModeRequest
import time

# 드론 상태를 저장할 변수
current_pose = None
target_detected = None
target_reached = False  # 타겟 중심에 도달했는지 여부
yaw_aligned = False  # Yaw가 정렬되었는지 여부
final_adjustment_done = False  # 최종 조정이 완료되었는지 여부
target_lost_time = None  # 타겟을 놓친 시간

def pose_callback(data):
    global current_pose
    current_pose = data

def target_callback(data):
    global target_detected, target_lost_time
    target_detected = data
    # 타겟이 다시 감지되었을 때 시간을 초기화
    if target_detected.Xc != 0.0 or target_detected.Yc != 0.0 or target_detected.W != 0.0 or target_detected.H != 0.0 or target_detected.Theta != 0.0:
        target_lost_time = None

def main():
    global target_reached, yaw_aligned, final_adjustment_done, target_lost_time
    
    rospy.init_node('landing_controller', anonymous=True)
    
    # 드론 위치 구독자 설정
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    
    # 타겟 검출 구독자 설정
    rospy.Subscriber('/states', States, target_callback)
    
    # 드론 제어 명령 발행자 설정
    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    
    rate = rospy.Rate(20)  # 20Hz

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'AUTO.LOITER'

    while not rospy.is_shutdown():
        if current_pose is None or target_detected is None:
            rate.sleep()
            continue
        
        control_msg = PositionTarget()
        control_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        control_msg.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW
        
        # 타겟 검출 정보가 초기값이라면 fail-safe 동작 수행
        if (target_detected.Xc == 0.0 and target_detected.Yc == 0.0 and 
            target_detected.W == 0.0 and target_detected.H == 0.0 and 
            target_detected.Theta == 0.0):
                start_time = time.time()
                while not rospy.is_shutdown():
                    current_time = time.time()
                    if current_time - start_time > 2.0:
                        rospy.logwarn("Target lost! Executing fail-safe procedure.")
                        control_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                        control_msg.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW
                        control_msg.position.x = 0
                        control_msg.position.y = 0
                        control_msg.position.z = 3
                        setpoint_pub.publish(control_msg)
                target_reached = False
                yaw_aligned = False
                final_adjustment_done = False
        else:
            if not target_reached:
                # 타겟의 중심점과 카메라 중심점의 오차 계산
                target_x = target_detected.Xc
                target_y = target_detected.Yc
                image_center_x = 640 / 2
                image_center_y = 480 / 2
                error_x = target_x - image_center_x
                error_y = target_y - image_center_y
                
                # 오차가 특정 값 이하일 때 타겟에 도달했다고 판단
                if abs(error_x) < 20 and abs(error_y) < 20:
                    target_reached = True
                    rospy.loginfo("Target reached. Beginning yaw alignment.")
                else:
                    # 드론 속도 조정을 위한 제어 명령 생성
                    control_msg.velocity.x = - (error_y * 0.002)  # y축 오차를 x축 속도로 변환
                    control_msg.velocity.y = - (error_x * 0.002)  # x축 오차를 y축 속도로 변환
                    control_msg.velocity.z = -0.2  # 고도 유지
                    
            elif target_reached and not yaw_aligned:
                # Theta를 사용하여 yaw 정렬
                theta_rad = math.radians(target_detected.Theta+90)
                if abs(theta_rad) < math.radians(1):  # Theta가 1도 이하일 때 정렬 완료
                    yaw_aligned = True
                    rospy.loginfo("Yaw aligned. Beginning final adjustment.")
                else:
                    control_msg.yaw_rate = -theta_rad * 0.2  # Theta를 yaw rate로 변환하여 제어 명령 생성
                    control_msg.velocity.x = 0  # y축 오차를 x축 속도로 변환
                    control_msg.velocity.y = 0  # x축 오차를 y축 속도로 변환
                    control_msg.velocity.z = -0.2 # 고도 유지
                    control_msg.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW
            
            elif yaw_aligned and not final_adjustment_done:
                # 최종 조정을 통해 중심에 오도록 함
                target_x = target_detected.Xc
                target_y = target_detected.Yc
                image_center_x = 640 / 2
                image_center_y = 480 / 2
                error_x = target_x - image_center_x
                error_y = target_y - image_center_y

                if abs(error_x) < 20 and abs(error_y) < 20:
                    final_adjustment_done = True
                    rospy.loginfo("Final adjustment complete. Beginning descent.")
                else:
                    control_msg.velocity.x = - (error_y * 0.002)  # y축 오차를 x축 속도로 변환
                    control_msg.velocity.y = - (error_x * 0.002)  # x축 오차를 y축 속도로 변환
                    control_msg.velocity.z = -0.2  # 고도 유지

            elif final_adjustment_done:
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.sleep(2)  # 2초간 대기
                    offb_set_mode.custom_mode = 'AUTO.LAND'
                    set_mode_client.call(offb_set_mode)

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
