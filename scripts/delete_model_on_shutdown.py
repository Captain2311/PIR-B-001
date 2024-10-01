#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel
import signal
import sys

def delete_model():
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model_service('pir')  
        rospy.loginfo("Model 'pir' deleted from Gazebo.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def signal_handler(sig, frame):
    rospy.loginfo('Ctrl + C detected. Deleting model from Gazebo...')
    delete_model()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('delete_model', anonymous=True)
    
    # Register the signal handler to capture Ctrl + C (SIGINT)
    signal.signal(signal.SIGINT, signal_handler)

    rospy.loginfo('Press Ctrl + C to delete the model and exit...')
    
    # Keep the node alive until Ctrl + C is pressed
    rospy.spin()
