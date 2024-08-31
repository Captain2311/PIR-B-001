#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import sys, select, termios, tty

# Joint efforts 
front_effort = 0.0
rear_effort = 0.0
right_effort = 0.0
left_effort = 0.0
    
effort_step = 0.01  # Amount by which to increase/decrease effort

# Key mappings for joint selection
joint_bindings = {
    'w': 'front_omni_controller',
    'x': 'rear_omni_controller',
    'd': 'right_wheel1j_controller',
    'a': 'left_wheel1j_controller',
    's': 'stop_controllers',
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Check for escape sequence
            key += sys.stdin.read(2)  # Read the next two characters
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_effort(selected_joint):
    global front_effort, rear_effort, right_effort, left_effort
    if selected_joint == 'front_omni_controller':
        front_pub.publish(Float64MultiArray(data=[front_effort, front_effort]))
    elif selected_joint == 'rear_omni_controller':
        rear_pub.publish(Float64MultiArray(data=[rear_effort, rear_effort]))
    elif selected_joint == 'right_wheel1j_controller':
        right_pub.publish(Float64(data=right_effort))
    elif selected_joint == 'left_wheel1j_controller':
        left_pub.publish(Float64(data=left_effort))
    elif selected_joint == 'stop_controllers':
        # Stop all controllers
        front_effort = 0.0 
        rear_effort = 0.0
        right_effort = 0.0
        left_effort = 0.0
        front_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        rear_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        right_pub.publish(Float64(data=0.0))
        left_pub.publish(Float64(data=0.0))
        
def increase_effort(selected_joint):
    global front_effort, rear_effort, right_effort, left_effort
    if selected_joint == 'front_omni_controller':
        front_effort += effort_step
        rospy.loginfo(f"Increasing effort: {front_effort} for {selected_joint}")
    elif selected_joint == 'rear_omni_controller':
        rear_effort += effort_step
        rospy.loginfo(f"Increasing effort: {rear_effort} for {selected_joint}")
    elif selected_joint == 'right_wheel1j_controller':
        right_effort += effort_step
        rospy.loginfo(f"Increasing effort: {right_effort} for {selected_joint}")
    elif selected_joint == 'left_wheel1j_controller':
        left_effort += effort_step
        rospy.loginfo(f"Increasing effort: {left_effort} for {selected_joint}")
        
def decrease_effort(selected_joint):
    global front_effort, rear_effort, right_effort, left_effort
    if selected_joint == 'front_omni_controller':
        front_effort -= effort_step
        rospy.loginfo(f"Decreasing effort: {front_effort} for {selected_joint}")
    elif selected_joint == 'rear_omni_controller':
        rear_effort -= effort_step
        rospy.loginfo(f"Decreasing effort: {rear_effort} for {selected_joint}")
    elif selected_joint == 'right_wheel1j_controller':
        right_effort -= effort_step
        rospy.loginfo(f"Decreasing effort: {right_effort} for {selected_joint}")
    elif selected_joint == 'left_wheel1j_controller':
        left_effort -= effort_step
        rospy.loginfo(f"Decreasing effort: {left_effort} for {selected_joint}")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_joint_controller')
    
    # Publishers for each joint controller
    right_pub = rospy.Publisher('/pir_b_001/right_wheel1j_controller/command', Float64, queue_size=10)
    left_pub = rospy.Publisher('/pir_b_001/left_wheel1j_controller/command', Float64, queue_size=10)
    front_pub = rospy.Publisher('/pir_b_001/front_omni_controller/command', Float64MultiArray, queue_size=10)
    rear_pub = rospy.Publisher('/pir_b_001/rear_omni_controller/command', Float64MultiArray, queue_size=10)
    
    selected_joint = 'front_omni_controller'
    
    print("Use 'w', 'x', 'd', 'a', 's' to select the joint to control.")
    print("Use the up and down arrow keys to increase/decrease effort.")
    print("Press 'Ctrl+C' to quit.")
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key in joint_bindings.keys():
                selected_joint = joint_bindings[key]
                print(f"Selected joint: {selected_joint}")
            elif key == '\x1b[A':  # Up arrow key
                increase_effort(selected_joint)
            elif key == '\x1b[B':  # Down arrow key
                decrease_effort(selected_joint)
            elif key == '\x03':  # Ctrl+C to quit
                break

            # Publish effort based on the selected joint
            publish_effort(selected_joint)
            
    except Exception as e:
        rospy.logerr(f"Error: {e}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.loginfo("Teleop node shut down gracefully.")
