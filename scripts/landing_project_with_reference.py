#!/usr/bin/env python

import rospy
import math
from mavros_msgs.msg import PositionTarget
from object_detector.msg import States  # Custom msg type
from drone_controller.msg import Error  # Custom msg type
from mavros_msgs.srv import CommandTOL, CommandTOLRequest

FACTORZ = 0.01  # Descend Factor
MIN_Z = 0.3  # Minimum altitude to consider landing
TARGET_THETA = 90.0  # Target theta angle

class PID:
    def __init__(self, cmax, cmin, ckp, ckd, cki):
        self.max = cmax
        self.min = cmin
        self.kp = ckp
        self.kd = ckd
        self.ki = cki
        self.pre_error = 0
        self.integral = 0
        self.pre_integral = 0

    def calculate(self, setpoint, pv, cdt):
        # Calculate error
        error = setpoint - pv
        
        # Proportional term
        Pout = self.kp * error
        
        # Integral term
        self.integral = self.pre_integral + 0.5 * (self.pre_error + error) * 0.1  # cdt is equals to 0.1 to avoid overflow
        Iout = self.ki * self.integral  # Integral output
        
        # Derivative term
        derivative = (error - self.pre_error) / 0.1  # cdt is equals to 0.1 to avoid overflow
        Dout = self.kp * self.kd * derivative  # Derivative output
        
        # Calculate total output
        output = Pout + Iout + Dout

        # Save error to previous error and previous integral value
        self.pre_error = error
        self.pre_integral = self.integral

        # Limit the max and min output
        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min

        return output

class Controller:
    def __init__(self):
        # Node initialization
        rospy.init_node('controller_node', anonymous=True)
        
        # PID controllers objects
        self.pidx = PID(0.75, -0.75, 0.005, 0.0008, 0.00005)  # max, min, kp, kd, ki
        self.pidy = PID(0.75, -0.75, 0.006, 0.00085, 0.00006)
        self.pidth = PID(0.35, -0.35, 0.004, 0.0005, 0.00001)
        
        # Publisher type mavros_msgs::PositionTarget, it publishes in /mavros/setpoint_raw/local topic
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        # Publisher type drone_controller::Error, it publishes in /error topic
        self.pub1 = rospy.Publisher('/error', Error, queue_size=10)
        
        # Subscriber to /predicted_states topic from object_detector/Corners
        self.sub = rospy.Subscriber('/predicted_states', States, self.controller_callback)
        
        # Landing client
        self.land_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
        
        self.last_time = rospy.Time.now()  # ROS time initialization
        
        self.imageW = 640 / 2  # Setpoint in X
        self.imageH = 480 / 2  # Setpoint in Y
        self.zini = 2  # Initial altitude

    def controller_callback(self, msg):
        # Error Calculation between image and template's center
        ErX = self.imageW - msg.Xc  # Error in X of the image
        ErY = self.imageH - msg.Yc  # Error in Y of the image
        ErTheta = msg.Theta  # Error in Theta of the image
        ErZ = abs(msg.W - msg.H)  # Error in W and H of the images

        # Publish the error
        er = Error()
        er.errorX = ErX
        er.errorY = ErY
        er.errorT = ErTheta
        er.errorS = ErZ

        # Variables to be published (Initialized in 0)
        Vx = 0
        Vy = 0
        Vthe = 0
        zpos = 0

        # Time since last call
        time_between_markers = (rospy.Time.now() - self.last_time).to_sec()
        self.last_time = rospy.Time.now()

        # If the error between width and height is less than 4 pixels and height is greater than 0.3
        if ErZ < 4.0 and self.zini > MIN_Z:
            zpos = self.zini - FACTORZ  # Descend Z based on the factor
        elif self.zini <= MIN_Z:
            zpos = self.zini - (FACTORZ / 2)  # Continue descending slowly
        else:
            zpos = self.zini  # If there is more than 3 pixels of error, hold pos

        # Drone service for automatic landing when it reaches a specific altitude and centroid conditions
        if zpos <= MIN_Z and abs(self.imageW - msg.Xc) < 20 and abs(self.imageH - msg.Yc) < 20:
            land_cmd = CommandTOLRequest()  # Set all the descend parameters to Zero
            land_cmd.yaw = 0
            land_cmd.latitude = 0
            land_cmd.longitude = 0
            land_cmd.altitude = 0

            # When it lands, everything goes to zero
            if not self.land_client.call(land_cmd):
                # Publish the service of landing
                rospy.loginfo("Landing")
                # Print final Error
                rospy.loginfo("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f)", er.errorX, er.errorY, er.errorT, er.errorS)
                self.pub1.publish(er)
                rospy.signal_shutdown("Landing completed")  # Shutdown the node

        # Update vehicle's position
        self.zini = zpos

        # Compute controller output for each axis
        Vy = self.pidx.calculate(self.imageW, msg.Xc, time_between_markers)  # Setpoint, Process Variable, Sample time for Vx
        Vx = self.pidy.calculate(self.imageH, msg.Yc, time_between_markers)
        Vthe = self.pidth.calculate(TARGET_THETA, msg.Theta, time_between_markers)

        # Position target object to publish
        pos = PositionTarget()
        
        # FRAME_LOCAL_NED to move WRT to body_ned frame
        pos.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # pos.header.stamp = rospy.Time.now()  # Time header stamp
        # pos.header.frame_id = "base_link"  # "base_link" frame to compute odom
        pos.type_mask = 1987  # Mask for Vx, Vy, Z pos and Yaw rate
        pos.position.z = zpos
        pos.velocity.x = Vx
        pos.velocity.y = Vy
        pos.yaw_rate = Vthe

        rospy.loginfo("PID Vx, Vy and Zpos values at (%f,%f, %f)", Vx, Vy, zpos)
        self.pub.publish(pos)

        rospy.loginfo("Error at Vx, Vy, Theta and Z are (%f,%f,%f,%f)", ErX, ErY, ErTheta, ErZ)
        self.pub1.publish(er)


if __name__ == '__main__':
    try:
        Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
