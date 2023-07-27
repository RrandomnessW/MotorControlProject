# MotorControlProject
This repository documents what I learned in my first coop work term. It includes a STM32 project and corresponding files. The most significant part of this project is the PID (Proportional, Integral, Derivative) controller which smoothens out the acceleration of the motor and maintains a certain speed even when there's a heavy load. 

What does this project do?

This project will make an industrial brushed DC motor spin at a varying velocity pattern that matches the sine wave function. 


How does it work?

This project requires an STM32 Nucleo64 Board, a motor driver chip, a motor with an encoder, a DC power supply, a breadboard, jumper wires and female header wires. 

In my project, I am using a NUCLEO-F302R8 board from ST and the L298 Motor driver shield and an industrial motor rated for 12V. I am also using an external USART device connected to the Nucleo Board, but the default USART peripheral works too. 

The L298 motor driver is required to vary a DC motor’s speed. A voltage of 12V will be supplied from the power supply to the Motor driver. The outputs of the driver will be connected to the motor. 

What is the goal of this project?

The goal of this project is to reflect on what I’ve learned from my co-op work term and as a reference for anyone that may want to know a bit more about using a Microcontroller to rotate a motor. 
