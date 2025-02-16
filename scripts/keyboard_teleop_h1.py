#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float64MultiArray
import sys, select, termios, tty

# Joint efforts 
driving_effort = 0.0
rolling_effort = 0.0
    
effort_step = 0.01  # Amount by which to increase/decrease effort

# Key mappings for joint selection
movement = {
    'w': 'Moving forward',
    'x': 'Moving backward',
    'd': 'Rolling in clockwise',
    'a': 'Rolling in counter clockwise',
    's': 'Stoping all motors ',
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

def publish_effort():
    global driving_effort, rolling_effort
    front_pub.publish(Float64MultiArray(data=[-1*driving_effort, -1*driving_effort]))
    rear_pub.publish(Float64MultiArray(data=[-1*driving_effort, -1*driving_effort]))
    right_pub.publish(Float64(data=rolling_effort))
    left_pub.publish(Float64(data=-1*rolling_effort))
        
def increase_drive_effort():
    global driving_effort
    driving_effort += effort_step
    rospy.loginfo(f"Increasing effort: {driving_effort}")
        
def reverse_drive_effort():
    global driving_effort
    driving_effort -= effort_step
    rospy.loginfo(f"Reversing effort: {driving_effort}")
    
def increase_roll_effort():
    global rolling_effort
    rolling_effort += effort_step
    rospy.loginfo(f"Increasing effort: {rolling_effort}")

def reverse_roll_effort():
    global rolling_effort
    rolling_effort -= effort_step
    rospy.loginfo(f"Increasing effort: {rolling_effort}")

def stop_all_motors(): 
    global driving_effort, rolling_effort
    driving_effort = 0.0
    rolling_effort = 0.0
    rospy.loginfo(f"Stopping all motors")
    
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop_h1')
    
    # Publishers for each joint controller
    right_pub = rospy.Publisher('/pir_b_001/right_wheel1j_controller/command', Float64, queue_size=10)
    left_pub = rospy.Publisher('/pir_b_001/left_wheel1j_controller/command', Float64, queue_size=10)
    front_pub = rospy.Publisher('/pir_b_001/front_omni_controller/command', Float64MultiArray, queue_size=10)
    rear_pub = rospy.Publisher('/pir_b_001/rear_omni_controller/command', Float64MultiArray, queue_size=10)
    
    msg = """
    Teleop pir-b-001 !
    ---------------------------
    Robot Movement:
        w               w/x: drive forward/ backward 
    a   s   d           d/a: roll cw/ ccw 
        x                 s: stop all motors

    CTRL-C to quit
    """
    print(msg)
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key in movement.keys():
                action = movement[key]
                print(f"Action: {action}")
                if key == 'w': # move forward
                    increase_drive_effort()
                elif key == 'x':  # move backward / reverse 
                    reverse_drive_effort()
                elif key == 'a':  # roll cw 
                    increase_roll_effort()
                elif key == 'd':  # roll ccw
                    reverse_roll_effort()
                elif key == 's':  # stop
                    stop_all_motors()
            elif key == '\x03':  # Ctrl+C to quit
                break
            
            publish_effort()
            
    except Exception as e:
        rospy.logerr(f"Error: {e}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.loginfo("Teleop node shut down gracefully.")
