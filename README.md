# ZEDbot robot integration
Here I am explaining the second step of the "Autonomous charging system" project concerning my master thesis in Mechatronics Engineering made in collaboration with the California State Universiy in Los Angeles. 

This subsection follows the [Simulation stage](https://github.com/LucaRoma97/ZEDbot), where I have explained the algorithm design and development, the implementation of the robot body and functionalities on Gazebo and the translation of the algorithm in C++ nodes communicating in ROS framework.

After the Simulation step, I wanted to test the algorithm on an actual robot. Actually, it was expected to test the algorithm on the robot present in the Electrical and Computer Science Department of the California State University. Unfourtanetely, the department has been closed for the entire duration of my experience in Los Angeles.
So, I decided to build one simple three-wheels robot. This robot is composed of the following main elements:

- **Jetson Nano** to run the ROS functions.
- **ZED 2 stereocamera** from Stereolabs to extract the camera data.
- **Encoders sensors** to keep track of the robot movements.
- **Arduino / motor driver** to provide the speed to the motors and to transmit the encoders data.

ZED 2 & Encoders           |  Jetson Nano              |  Arduino / motor driver
:-------------------------:|:-------------------------:|:-------------------------:
<img src="images/zed_encoders.png" alt="alt text" width="300" height="250">  |  <img src="images/jetson_nano.png" alt="alt text" width="300" height="250"> |  <img src="images/motor_scheme.png" alt="alt text" width="300" height="250">


Here I will briefly refer on the Hardware Set Up focusing on the communication between the various elements. Then 

## Hardware Set up
The core of the robot is the NVIDIA Jetson Nano embedded platform. It gets the camera data from the ZED 2 stereocamera and the encoders data as input, elaborates them and provides as output the speeds for the right and left wheels. 

<img src="images/hardware_set_up.png" alt="alt text" width="550" height="350">

Communication bridges:
- **ZED -> Jetson** : Camera Data
- **Encoders -> Arduino -> Jetson Nano** : Encoders data
- **Jetson -> Arduino -> motor driver** : Speed data transmission

## Control system development


## Software integration
