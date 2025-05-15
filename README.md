# Nereo Navigator
This Repository contains the firmware used in Nereo's FC board.
The board packs an STM32F469VIT6 microcontroller, the project is compiled with STM32CubeIDE Toolchain.
## Aim
This firmware aims to, given the odonetry of the ROV and the command velocity, calculate and send the best pwm values to the 8 thrusters to move the ROV in the right direction with the desired speed.
## micro-ROS
micro-ROS framework is used to communicate with the microcontroller, in fact it should be connected with serial to a microROS Agent. Using micro-ROS allows for unmatched modularity and ease of configuration regarding the communication with all the other deviced on the ROV's network.
## Using this project
This project has been designed to run on an STM32F469VIT6 MCU, yet it could in theory run (with proper adjustments) on any STM32 MCU. We provide the precompiled elf file, but please take not that it was specifically built to work on a precise MCU and GPIO pinout, so it any hardware change is introduced please edit the project to adjust it to your needs
# Docs 
## [Project Structure.](docs/project_structure.md)
## [Interfaces.](docs/interfaces.md)
