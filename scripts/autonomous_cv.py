#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import sys
import argparse
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np

class autonomous_cv:
    def __init__(self, file_arg):
        self.bridge = CvBridge()
        self.model = YOLO(file_arg) 
        self.img = rospy.Subscriber("/camera/image_raw", Image, self.decisionCallback)
        # Publishers for each joint controller
        self.right_pub = rospy.Publisher('/pir_b_001/right_wheel1j_controller/command', Float64, queue_size=10)
        self.left_pub = rospy.Publisher('/pir_b_001/left_wheel1j_controller/command', Float64, queue_size=10)
        self.front_pub = rospy.Publisher('/pir_b_001/front_omni_controller/command', Float64MultiArray, queue_size=10)
        self.rear_pub = rospy.Publisher('/pir_b_001/rear_omni_controller/command', Float64MultiArray, queue_size=10)
        self.max_speed = 3.5
        self.driving_effort = 0.0
        self.rolling_effort = 0.0
          
    def decisionCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model.track(cv_image, persist= True, classes = [1,2,3,4,5,6,7], conf= 0.25)
            
        except CvBridgeError as e:
            print(e)
        
        annotated_image = results[0].plot()
        for result in results:
            for detection in result.boxes.data:
                if len(detection) == 7:
                    path = int(detection[6])  # Class ID is in the 6th position (index 6)
                    if path == 0: # end
                        pass
                    elif path == 1: # exit 
                        x1, _, x2, _ = detection[:4]
                        x = x2 - x1
                        if x > 150:              
                            self.stop_all_motors() 
                    elif path == 2: # simple junction 
                        self.move_forward()
                        self.stop_rolling_motors() 
                    elif path == 3: # simple_junction_left_oriented
                        self.move_forward()
                        self.roll_left()  
                    elif path == 4: # simple_junction_right_oriented
                        self.move_forward()
                        self.roll_right() 
                    elif path == 5: # t_junction
                        self.move_forward()
                        self.stop_rolling_motors() 
                    elif path == 6: # t_junction_left_oriented
                        self.move_forward()
                        self.roll_left()  
                    elif path == 7: # t_junction_right_oriented
                        self.move_forward()
                        self.roll_right()
                    
        self.publish_effort()             
        cv2.imshow("pir_stream", annotated_image)
        cv2.waitKey(3)
        
    def publish_effort(self):
        self.front_pub.publish(Float64MultiArray(data=[-1 * self.driving_effort, -1 * self.driving_effort]))
        self.rear_pub.publish(Float64MultiArray(data=[-1 * self.driving_effort, -1 * self.driving_effort]))
        self.right_pub.publish(Float64(data=self.rolling_effort))
        self.left_pub.publish(Float64(data= -1 * self.rolling_effort))
    
    def move_forward(self):
        self.driving_effort = self.max_speed
        rospy.loginfo("moving forward") 
    
    def move_backward(self):
        self.driving_effort = -1 * self.max_speed
        rospy.loginfo("moving backwards") 
    
    def roll_right(self):
        self.rolling_effort = -1 * self.max_speed
        rospy.loginfo("rolling right")
    
    def roll_left(self):
        self.rolling_effort = self.max_speed
        rospy.loginfo("rolling left")
        
    def stop_all_motors(self): 
        self.driving_effort = 0.0
        self.rolling_effort = 0.0
        rospy.loginfo("Stopping all motors")
    
    def stop_rolling_motors(self): 
        self.rolling_effort = 0.0
        rospy.loginfo("Stopping rolling motors")
#-----------------------------------------------------------------------------------------------------------------          

def main(args):
    
    parser = argparse.ArgumentParser(description='path to weed detector model')
    parser.add_argument('file_path', type=str, help='Path to the input file')
    parsed_args = parser.parse_args(args)
    
    try: 
        rospy.init_node('autonomous_cv', anonymous= True)
        pir = autonomous_cv(parsed_args.file_path)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv[1:])
            