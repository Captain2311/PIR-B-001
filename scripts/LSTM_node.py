#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import Imu
import keras
import numpy as np
import joblib

model = keras.models.load_model('LSTM_control.keras')
joint_scaler = joblib.load('joint_scaler.pkl')
imu_scaler = joblib.load('imu_scaler.pkl')  
window_size = 30  # LSTM window size

# Buffers for storing recent joint and IMU data
joint_data = {
    "right_wheel1j": [0.0] * window_size,
    "left_wheel1j": [0.0] * window_size,
    "front_omni": [0.0] * window_size,
    "rear_omni": [0.0] * window_size
}
imu_data = []  

def imu_callback(msg):
    global imu_data
    imu_data.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    if len(imu_data) > window_size:
        imu_data = imu_data[-window_size:]

def right_wheel_callback(msg):
    joint_data["right_wheel1j"].append(msg.data)
    if len(joint_data["right_wheel1j"]) > window_size:
        joint_data["right_wheel1j"] = joint_data["right_wheel1j"][-window_size:]

def left_wheel_callback(msg):
    joint_data["left_wheel1j"].append(msg.data)
    if len(joint_data["left_wheel1j"]) > window_size:
        joint_data["left_wheel1j"] = joint_data["left_wheel1j"][-window_size:]

def front_omni_callback(msg):
    joint_data["front_omni"].append(msg.data[0])
    if len(joint_data["front_omni"]) > window_size:
        joint_data["front_omni"] = joint_data["front_omni"][-window_size:]

def rear_omni_callback(msg):
    joint_data["rear_omni"].append(msg.data[0])
    if len(joint_data["rear_omni"]) > window_size:
        joint_data["rear_omni"] = joint_data["rear_omni"][-window_size:]

def predict_efforts():
    if len(joint_data["right_wheel1j"]) < window_size or len(imu_data) < window_size:
        return None

    joint_input = np.array([
        joint_data["right_wheel1j"],
        joint_data["left_wheel1j"],
        joint_data["front_omni"],
        joint_data["rear_omni"]
    ]).T  # Shape: (window_size, 4)

    joint_normalized = joint_scaler.transform(joint_input)
    imu_normalized = imu_scaler.transform(imu_data)
    
    combined_input = np.hstack((imu_normalized, joint_normalized))  # Shape: (window_size, 8)

    combined_input = combined_input.reshape((1, window_size, 8))

    predicted_efforts = model.predict(combined_input)

    # Inverse transform the prediction
    predicted_efforts_original = joint_scaler.inverse_transform(predicted_efforts)

    return predicted_efforts_original[0]

def main():
    rospy.init_node('efforts_prediction_node')

    # Subscribers
    rospy.Subscriber('/imu', Imu, imu_callback)  
    rospy.Subscriber('/pir_b_001/right_wheel1j_controller/command', Float64, right_wheel_callback)
    rospy.Subscriber('/pir_b_001/left_wheel1j_controller/command', Float64, left_wheel_callback)
    rospy.Subscriber('/pir_b_001/front_omni_controller/command', Float64MultiArray, front_omni_callback)
    rospy.Subscriber('/pir_b_001/rear_omni_controller/command', Float64MultiArray, rear_omni_callback)

    # Publishers
    right_pub = rospy.Publisher('/pir_b_001/right_wheel1j_controller/command', Float64, queue_size=10)
    left_pub = rospy.Publisher('/pir_b_001/left_wheel1j_controller/command', Float64, queue_size=10)
    front_pub = rospy.Publisher('/pir_b_001/front_omni_controller/command', Float64MultiArray, queue_size=10)
    rear_pub = rospy.Publisher('/pir_b_001/rear_omni_controller/command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        predicted_efforts = predict_efforts()
        if predicted_efforts is not None:
            # Publish the predicted velocities
            right_pub.publish(Float64(predicted_efforts[0]))
            left_pub.publish(Float64(predicted_efforts[1]))

            front_msg = Float64MultiArray()
            rear_msg = Float64MultiArray()
            front_msg.data = [predicted_efforts[2], predicted_efforts[2]]
            rear_msg.data = [predicted_efforts[3], predicted_efforts[3]]

            front_pub.publish(front_msg)
            rear_pub.publish(rear_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
