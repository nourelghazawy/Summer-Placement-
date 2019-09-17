# Summer-Placement-
Wireless Communication Implementation on a Line Following Robot

## System Overview

The Line Following Robot had 1 STM32F401 Board which reads data from ultrasonic sensor and drive two rear wheels on the robot in order to follow the line. This was done by implementing open loop control system. This system’s PID constants needs to be tuned. Currently the robot’s control system is tuned by hard coding the PID constants in the code. Which is time consuming to reach the ideal constants since for every trial on the track the code needs to be changed. Therefore, wireless communication was implemented so that these constants can be tuned without changing the code on the robot. To implement wireless communication NRF52840 Bluetooth 5 board was added on the robot.

In addition to this, to be able to close the loop for the robot’s control system an additional servo motor controller was added. This was done using encoders on the wheels that are being read using a dedicated microcontroller (PIC18). The PIC18 microcontroller was also added to the robot. The PCB board for the microcontroller was designed on Altium Designer and it’s attached in the project files. The purpose for using dedicated microcontroller is to allow the main board on the robot (Arm STM32) to focus on the critical tasks and only reads the encoder values using UART communication when needed. 

To be able to see the behaviour of the servo motor controller, the encoder data where transmitted over Bluetooth, then displayed on a LabVIEW VI. This data was used to analyse the response of the system to different PID constants. The PID constants were also transmitted over Bluetooth to the robot. 

## Final System Configuration 

The robot had 3 Microcontrollers. STM32, NRF52840 and PIC18. To communicate with the robot over Bluetooth. you can either use an additional NRF52840 board on the PC side or you can use a mobile app. During tuning the PID constants I used another NRF52840 so that I can display the data on LabVIEW in Real-Time. 

To control the robot without following the line, I used the nRF Toolbox app since you can add an instruction for every button added as shown in the figure below. 
