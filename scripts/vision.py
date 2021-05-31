#!/usr/bin/env python3

import rospy
import os
import rospy,cv_bridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from scipy.spatial import distance


class Detector:

    def __init__(self):
        rospy.init_node('detector')

        self.no_image = True

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()    
        rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)

    def detect_shape(self):

        if self.no_image:
            return

        cv2.imwrite('temp.jpg', self.image)
        img = self.image
    
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        
        image_center = np.asarray(gray.shape) / 2
        image_center = image_center[:2]
        image_center = tuple(image_center.astype('int32'))
        
        print(gray.shape)
        print(image_center)

        shapes = []
        for i, contour in enumerate(contours):        
            if i == 0:
                continue

            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            
            print(len(approx))
            

            cv2.drawContours(img, [contour], 0, (0, 50, 255), 2)
            
        
            # finding center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])
                contour_center = (x,y)

            print(contour_center)
            cv2.circle(img, contour_center, 3, (100, 255, 0), 2)
            distance_to_center = distance.euclidean(image_center, contour_center)
            shapes.append({'contour': contour, 'center': contour_center, 'distance_to_center': distance_to_center})

        sorted_shapes = sorted(shapes, key=lambda i: i['distance_to_center'])
        cv2.circle(img, image_center, 3, (255, 255, 255), 2)
        # find contour of closest building to center and draw it (blue)
        center_building_contour = sorted_shapes[0]['contour']
        cv2.drawContours(img, [center_building_contour], 0, (255, 0, 0), 2)
        cv2.imwrite('processed.jpg', img)
        
            # putting shape name at center of each shape
            # if len(approx) == 3:
            #     print('Triangle')

            # elif len(approx) == 4:
            #     print('Quad')
        
            # elif len(approx) == 5:
            #     print('pentagon')
        
            # elif len(approx) == 6:
            #     print('hexa')
        
            # else:
            #     print('circle')
        
        # displaying the image after drawing contours
        #cv2.imshow('shapes', img)
        
       # cv2.waitKey(0)
       # cv2.destroyAllWindows()

    #set the camera feed
    def image_callback(self, msg):
        self.no_image = False
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
        

    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.detect_shape()
            r.sleep()       

if __name__ == '__main__':
    detector = Detector()

    detector.run()