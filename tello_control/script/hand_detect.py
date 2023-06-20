#!/usr/bin/env python
import rospy
import cv2
import mediapipe as mp

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tello_control.msg import hand_info

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

image_pub = rospy.Publisher('hand_detect/image', Image, queue_size=10)
hand_info_pub = rospy.Publisher('hand_detect/hand_info', hand_info, queue_size=10)

with mp_hands.Hands(min_detection_confidence=0.5,min_tracking_confidence=0.5) as hands:
    def img_callback(msg_image):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg_image, desired_encoding="rgb8")
        results = hands.process(image)

        if results.multi_hand_landmarks:
            i = 0
            for hand_landmarks in results.multi_hand_landmarks:
                msg = hand_info()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "hand_%s" % i
#                for j in range(21)
                msg.landmarks = hand_landmarks.landmark

                hand_info_pub.publish(msg)
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                i = i+1

        image_pub.publish(bridge.cv2_to_imgmsg(image, encoding="rgb8"))

    if __name__ == '__main__':
        rospy.init_node('hand_detect', anonymous=True)
        rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
        rospy.spin()
