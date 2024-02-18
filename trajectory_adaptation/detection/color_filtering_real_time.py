#!/usr/bin/env python3
#Author: ALESSANDRO COSTA 02/24

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.mgs import Float32MultiArray


#start a roscore before

def color_filtering(frame):
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,100,100])
    upper_red = np.array([7,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)

    return mask, frame

def detection_object(mask, frame):
    cx = 0
    cy = 0
  
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea, default = None)
    if largest_contour is not None:
        leftmost = tuple(largest_contour[largest_contour[:,:,0].argmin()][0])
        rightmost = tuple(largest_contour[largest_contour[:,:,0].argmax()][0])
        topmost = tuple(largest_contour[largest_contour[:,:,1].argmin()][0])
        bottommost = tuple(largest_contour[largest_contour[:,:,1].argmax()][0])

        cx=int((leftmost[0]+rightmost[0])/2)
        cy=int((topmost[1]+bottommost[1])/2)
        #print(cx,cy)
        #cv2.circle(frame,(cx,cy),5,(0,255,255),-1)
        
    print(cx,cy)
    cv2.imshow('Red object detection', cv2.resize(frame, (960, 540)))
    cv2.waitKey(1)

    return cx, cy

def publish_centroid(cx, cy):

    centroid_msg = Point()
    centroid_msg.x = cx
    centroid_msg.y = cy
    centroid_msg.z = 0.0
    
    pub_cen.publish(centroid_msg)
    rate.sleep()

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point


#start a roscore before

def color_filtering(frame):
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,100,100])
    upper_red = np.array([7,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)

    return mask, frame

def detection_object(mask, frame):
    cx = 0
    cy = 0
  
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea, default = None)
    if largest_contour is not None:
        leftmost = tuple(largest_contour[largest_contour[:,:,0].argmin()][0])
        rightmost = tuple(largest_contour[largest_contour[:,:,0].argmax()][0])
        topmost = tuple(largest_contour[largest_contour[:,:,1].argmin()][0])
        bottommost = tuple(largest_contour[largest_contour[:,:,1].argmax()][0])

        cx=int((leftmost[0]+rightmost[0])/2)
        cy=int((topmost[1]+bottommost[1])/2)
        #print(cx,cy)
        #cv2.circle(frame,(cx,cy),5,(0,255,255),-1)
        
    print(cx,cy)
    cv2.imshow('Red object detection', cv2.resize(frame, (960, 540)))
    cv2.waitKey(1)

    return cx, cy

def publish_centroid(cx, cy):

    centroid_msg = Point()
    centroid_msg.x = cx
    centroid_msg.y = cy
    centroid_msg.z = 0.0
    
    pub_cen.publish(centroid_msg)
    rate.sleep()

def publish_ten_centroids(centroids):
    # Create a Float32MultiArray message to hold the centroids
    centroids_msg = Float32MultiArray()
    centroids_msg.data = centroids

    # Publish the centroids
    pub_cen.publish(centroids_msg)
    rate.sleep()


def main():
    
    # cx = 0.0
    # cy = 0.0
    last_centroids = []
    cap = cv2.VideoCapture(5)
    while not rospy.is_shutdown():
        while True:
            ret, frame = cap.read()

            if not ret:
                break

            mask, frame = color_filtering(frame)

            cx, cy = detection_object(mask,frame)
            print(cx,cy)

            # for publishing the last 10 centroids ---- 
            last_centroids.append((cx, cy))

            if len(last_centroids) > 10:
                last_centroids.pop(0)
                
            publish_ten_centroids(last_centroids)
            # -----

            #for publishing just the last one (uncomment the x and y initialization at the beginning)
            #publish_centroid(cx,cy)

            if cv2.waitKey(1) & 0xFF == ord('Q'):
                    break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('publish_centroid')
    pub_cen = rospy.Publisher('/centroid', Point, queue_size=100)
    rate = rospy.Rate(60) #Hz limited by camera frequency
    main()


def main():
    
    cx = 0.0
    cy = 0.0
    cap = cv2.VideoCapture(5)
    while not rospy.is_shutdown():
        while True:
            ret, frame = cap.read()

            if not ret:
                break

            mask, frame = color_filtering(frame)

            cx, cy = detection_object(mask,frame)
            print(cx,cy)
            publish_centroid(cx, cy)


            if cv2.waitKey(1) & 0xFF == ord('Q'):
                    break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('publish_centroid')
    pub_cen = rospy.Publisher('/centroid', Point, queue_size=100)
    rate = rospy.Rate(60) #Hz limited by camera frequency
    main()