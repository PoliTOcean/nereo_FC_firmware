# Nereo Navigator
This Repository contains the firmware used in Nereo's FC board.
The board packs an STM32F469VIT6 microcontroller, the project is compiled with STM32CubeIDE Toolchain.
## micro-ROS
micro-ROS framework is used to communicate with the microcontroller, in fact it should be connected with serial to a microROS Agent. Using micro-ROS allows for unmatched modularity and ease of configuration regarding the communication with all the other deviced on the ROV's network.
## Using this project
This project has been designed to run on an STM32F469VIT6 MCU, yet it could in theory run (with proper adjustments) on any STM32 MCU.