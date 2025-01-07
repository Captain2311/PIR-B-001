# Pipe Inspection Robot
## Robot Simulation and Configuration
This section outlines the steps required to set up and configure the robot simulation in a Gazebo environment.

### Launching the Gazebo Simulation
To start the simulation of the robot in Gazebo, you can choose between the following world configurations:

Complex Simulation World: For a more complex pipe environment, launch the complex world:

    $ roslaunch pir-b-001 complex_world_gazebo.launch
Simple Simulation World: For a basic simulation setup, launch the simple world:

    $ roslaunch pir-b-001 simple_world_gazebo.launch
It is recommended to launch the Gazebo world first and then spawn the robot in a separate process to facilitate settings configuation. Follow the procedure below:

Launch the Gazebo world:

    $ roslaunch pir-b-001 pipe_world.launch
Spawn the robot in a separate process: Open a second terminal and run:

    $ roslaunch pir-b-001 pir_spawner.launch
### Tuning the Torsional Springs
If you wish to experiment with the robot’s torsional spring parameters, you can do so by modifying the *spring_param.yaml* file located in the config folder. This file allows you to adjust various spring properties to better simulate the robot's behavior in different environments.

To gain a deeper understanding of the robot's physical structure (links and joints), you can visualize the robot by launching the *display.launch* file. This will help you inspect the robot’s configuration and joint constraints.

    $ roslaunch pir-b-001 display.launch
    
## Keyboard Teleoperation Control
For manual control of the robot using a keyboard, execute the following command:

    $ rosrun pir-b-001 keyboard_teleop.py
This node provides basic teleoperation functionality for robot movement.

If the robot is operating inside a pipe, it is recommended to use a specialized teleoperation script designed for that environment. The following command runs a node optimized for controlling the robot within piping networks:

    $ rosrun pir-b-001 keyboard_teleop_h1.py
This will provide a more convinient control of the robot's movements when navigating pipes.

## Autonomous Navigation
For autonomous navigation, ensure that the Gazebo simulation is running and the robot has been successfully spawned. Once the setup is complete, navigate to the directory containing the necessary machine learning models' weights:

    $ cd ~/catkin_ws/src/pir-b-001/scripts
### Running the Classification-Based Model (YOLOv8)
To enable the robot to perform autonomous navigation with a classification approach using YOLOv8, execute the following command, replacing best[x].pt with the appropriate pre-trained model weight file:

    $ rosrun pir-b-001 autonomous_cv.py best[x].pt
The best[x].pt files (e.g., best1.pt, best2.pt, best3.pt) are pre-trained models for object detection. If you are using models trained with cross-validation, the corresponding weight files are named cv1.pt, cv2.pt, ..., cv5.pt. Note that autonomous_cv.py accepts only a single parameter: the model weight file.

### Running the Regression-Based Model (LSTM)
To enable autonomous navigation based on the robot's IMU data and joint positions using an LSTM model, use the following command to run the regression model:

    $ rosrun pir-b-001 LSTM_node.py
This node will utilize the LSTM model to process sensor data and generate navigation commands for the robot.

