# 2018FaultTolerant
Dissertation, the Fault Tolerant Control of Autonomous System on Localisation

The fault tolerant control is a popular solution for improving the availability of autonomous systems. It would be beneficial for robots if it can automatically detect and identify the fault and provide corresponding actions to minimize the expense. This project mainly implements the fault tolerant control in the non-holonomic robot Puzzle Bot which is made by the Autonomous Research Group in the University of Manchester.

In this project fault tolerant control is applied to the tasks of Puzzle Bots on localisation and mapping. Puzzle Bots are non-holonomic robots made by the Autonomous Research Group in the University of Manchester. The pose of the robot is calculated by the speeds of the two drive wheels. Encoders and current sensors are implemented in the motor circuits. Encoders are as transducer to calculate speeds; whilst current sensors calculate currents of motor inputs to estimate motor speeds. If the readings from encoders are distinct from the estimations speeds, faults will be reported and encoder measurements will be replaced by estimated speeds to finish localisation tasks.
For this project, ROS operating system will be used as the platform of communication among different procedures such as detecting speeds and currents, calculating estimations, motion controlling on motors, localisation, fault detecting and fault tolerant control. Those procedures work at the same time through the ROS environment and are communicated by messages.

The aim of this project is to implement the current sensor, add fault detection model, develop the motion control and implement the localisation task with fault tolerant control on the non-holonomic robot Puzzle Bot. To achieve such aim following objectives are given:

 Implement a current sensor in the single motor circuit and test the characteristics. Check the normal working condition of the current sensor in the circuit.
 Implement the mathematical motor model and tune it according to behaviours of the real motor. Figure out the property of every parameter in this model.
 Add currents from the current sensor to the motor model. Estimate motor speeds with measured currents.
 Make fault condition. If the encoder is broken, there will be a distinct difference between measured speeds from the encoder and the estimated speeds from currents.
 Add PID speed control for the motors so that the motor could rotate at a reference speed.
 Build the Puzzle Bot and implement current sensors.
 Build the localisation model. When receiving speeds, the model outputs the poses of the robot.
 Add the fault tolerant control in the localisation model. When a fault occurs, the measured speeds are replaced by estimated speeds to calculate poses of the robot.

The whole system mainly uses C++ as the programming language. C# is used in the Arduino implementation file. 
The system is made up of five nodes, including Arduino, Model, PID, Fault and Localisation. 
The node Arduino is stored in the Arduino board; 
the node PID is stored in Raspberry Pi. 
The nodes Model, Fault and Localisation are stored in computer. 

Each node has one main task in the program. All nodes work concurrently and communicate with each other by several types of messages through the topics. They are started at the same time by using a launch file. 
 
1motor_sensor.ino is used to control the left wheel motor directly on Arduino (you will need another similar file for the right wheel)


