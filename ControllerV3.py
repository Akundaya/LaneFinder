#!/usr/bin/env python

# This version does everything ControllerV2 does and
# Also turns 90 degree counter clockwise upon seeing red color up close

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from simple_pid import PID


class LineFollower(object):

    def __init__(self):

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
        self.imu_sub = rospy.Subscriber("imu_data", Imu, imu_callback)
        self.bridge_object = CvBridge()
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cv_pub = rospy.Publisher("/cv_image", Image, queue_size=10)

    def imu_callback(self):

        if Imu_msg == None:
            continue

        # print('Yaw:%.4f' % yaw)




    def camera_callback(self, data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        except CvBridgeError as e:
            print(e)

        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.

        (roll, pitch, yaw) = euler_from_quaternion(Imu_msg)

        newImage = cv_image.copy()

        height, width, channels = cv_image.shape
        descentre = 100
        rows_to_watch = 60
        crop_img = cv_image[(height) / 2 + descentre:(height) / 2 + (descentre + rows_to_watch)][1:width]

        ###########Color Detector for Stopping############

        newImage = cv_image.copy()
        hsv2 = cv2.cvtColor(newImage, cv2.COLOR_BGR2HSV)
        upper_red = np.array([5, 255, 255])
        lower_red = np.array([0, 50, 20])
        mask2 = cv2.inRange(hsv2, lower_red, upper_red)
        kernel1 = np.ones((5, 5), np.uint8)
        img_dilation = cv2.blur(mask2, (5, 5))

        cnts = cv2.findContours(img_dilation, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        x = max(cnts, key=cv2.contourArea)
        print(cv2.contourArea(x))
        cv2.drawContours(newImage, [x], -1, (255, 255, 255), -1)

        #################################

        rgb = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        blur = cv2.GaussianBlur(rgb, (5, 5), cv2.BORDER_DEFAULT)
        gray = cv2.cvtColor(blur, cv2.COLOR_RGB2GRAY)
        _, thresh = cv2.threshold(gray, np.mean(gray), 255, cv2.THRESH_BINARY)

        # Detect color
        upper_green = np.array([86, 255, 255])
        lower_green = np.array([25, 0, 0])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        bitwise = cv2.bitwise_and(crop_img, crop_img, mask=mask)
        blur1 = cv2.GaussianBlur(bitwise, (5, 5), cv2.BORDER_DEFAULT)

        ######### Perform morpholgical operations ###########
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations=1)
        close[np.where(mask != 0)] = 0

        mask0 = mask + close

        kernel = np.ones((3, 3), np.uint8)
        img_erosion = cv2.erode(close, kernel, iterations=2)
        edges = cv2.Canny(img_erosion, 50, 150)

        # Compute the centroid of the blob
        m = cv2.moments(img_erosion, False)
        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        except ZeroDivisionError:
            cy, cx = height / 2, width / 2

        cv2.circle(img_erosion, (int(cx), int(cy)), 5, (10, 10, 255), 2)

        # Controlling the speed of the robot
        pid = PID(1.0, 0.1, 0.1, setpoint=width / 2);
        control_signal = pid(cx);
        # error_x = cx - width / 2;
        speed_cmd = Twist();

        speed_cmd.angular.z = control_signal / 60;


        #IF Red Box is close enough
        if cv2.contourArea(x) > 3000:
            #Run Turning algorithm-------------------------------------------------------Turning algorithm
            speed.linear.x = 0.6
            speed.angular.z = 1.6
            speed_pub.publish(speed)

            old_yaw = yaw
            while True:
                (roll, pitch, yaw) = euler_from_quaternion(Imu_msg)
                print("--------")
                print("Difference in yaw:%.4f" % abs(yaw - old_yaw))
                if abs(yaw - old_yaw) > 2.7:
                    speed.linear.x = 0
                    speed.angular.z = 0
                    speed_pub.publish(speed)
                    break
                time.sleep(0.5)

        self.speed_pub.publish(speed_cmd)
        cv_ros = self.bridge_object.cv2_to_imgmsg(newImage, encoding="passthrough")
        self.cv_pub.publish(cv_ros)


def main():
    rospy.init_node('line_following_node')
    line_follower_object = LineFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
