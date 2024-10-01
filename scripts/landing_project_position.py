#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from object_detector.msg import States  # 타겟 검출 메시지 타입
from mavros_msgs.srv import SetMode, SetModeRequest
from tf.transformations import quaternion_from_euler  # Quaternion 변환을 위한 모듈

# 드론 상태를 저장할 변수
current_pose = None
target_detected = None
target_reached = False  # 타겟 중심에 도달했는지 여부
yaw_aligned = False  # Yaw가 정렬되었는지 여부
final_adjustment_done = False  # 최종 조정이 완료되었는지 여부
maintain_z_position = 2 # 몇 미터의 위치에서 호버링 하면서 어레인지할지

# 콜백 함수 정의
def pose_callback(data):
    global current_pose
    current_pose = data

def target_callback(data):
    global target_detected
    target_detected = data

def main():
    global target_reached, yaw_aligned, final_adjustment_done
    
    rospy.init_node('landing_controller', anonymous=True)
    
    # 드론 위치 구독자 설정
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    
    # 타겟 검출 구독자 설정
    rospy.Subscriber('/states', States, target_callback)
    
    # 드론 제어 명령 발행자 설정
    setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(20)  # 20Hz

    # 서비스 클라이언트 설정
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'AUTO.LOITER'

    while not rospy.is_shutdown():
        if current_pose is None or target_detected is None:
            rate.sleep()
            continue
        
        control_msg = PoseStamped()
        control_msg.header.stamp = rospy.Time.now()
       
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
                control_msg.pose.position.x = current_pose.pose.position.x - (error_y * 0.002)
                control_msg.pose.position.y = current_pose.pose.position.y - (error_x * 0.002)
                control_msg.pose.position.z = maintain_z_position  # 고도 유지
                
        elif target_reached and not yaw_aligned:
            # Theta를 사용하여 yaw 정렬
            theta_rad = math.radians(target_detected.Theta + 90)

            if abs(theta_rad) < math.radians(1):  # Theta가 1도 이하일 때 정렬 완료
                yaw_aligned = True
                rospy.loginfo("Yaw aligned. Beginning final adjustment.")
            else:
                # Yaw 정렬을 위해 quaternion 계산
                quat = quaternion_from_euler(0, 0, theta_rad)
                print(theta_rad)
                control_msg.pose.orientation.x = quat[0]
                control_msg.pose.orientation.y = quat[1]
                control_msg.pose.orientation.z = quat[2]
                control_msg.pose.orientation.w = quat[3]

                # 위치 유지
                control_msg.pose.position.x = current_pose.pose.position.x
                control_msg.pose.position.y = current_pose.pose.position.y
                control_msg.pose.position.z = maintain_z_position  # 고도 유지
        
        elif yaw_aligned and not final_adjustment_done:
            # 최종 조정을 통해 중심에 오도록 함
            target_x = target_detected.Xc
            target_y = target_detected.Yc
            image_center_x = 640 / 2
            image_center_y = 480 / 2
            error_x = target_x - image_center_x
            error_y = target_y - image_center_y

            if abs(error_x) < 10 and abs(error_y) < 10:
                final_adjustment_done = True
                rospy.loginfo("Final adjustment complete. Beginning descent.")
            else:
                control_msg.pose.position.x = current_pose.pose.position.x - (error_y * 0.002)
                control_msg.pose.position.y = current_pose.pose.position.y - (error_x * 0.002)
                control_msg.pose.position.z = maintain_z_position  # 고도 유지

        elif final_adjustment_done:
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("AUTO.LOITER mode set successfully. Preparing for AUTO.LAND.")
                rospy.sleep(2)  # 2초간 대기
                offb_set_mode.custom_mode = 'AUTO.LAND'
                if set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("AUTO.LAND mode set successfully.")
                else:
                    rospy.logwarn("Failed to set AUTO.LAND mode.")
            else:
                rospy.logwarn("Failed to set AUTO.LOITER mode.")

        # 제어 명령 퍼블리시
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
