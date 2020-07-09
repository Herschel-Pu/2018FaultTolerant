# 2018FaultTolerant
Dissertation, the Fault Tolerant Control of Autonomous System on Localisation

The whole system mainly uses C++ as the programming language. C# is used in the Arduino implementation file. 
The system is made up of five nodes, including Arduino, Model, PID, Fault and Localisation. 
The node Arduino is stored in the Arduino board; 
the node PID is stored in Raspberry Pi. 
The nodes Model, Fault and Localisation are stored in computer. 

Each node has one main task in the program. All nodes work concurrently and communicate with each other by several types of messages through the topics. They are started at the same time by using a launch file. Figure 1 shows the process of nodes passing messages.
 
1motor_sensor.ino is used to control the left wheel motor directly on Arduino (you will need another similar file for the right wheel)


