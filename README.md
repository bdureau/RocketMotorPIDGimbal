# RocketMotorPIDGimbal (firmware)
This is yet again another attempt to fly a model rocket without fins using long burn motors. The following repository is the firmware for the board.
<img src="/photos/motor_gimbal1.png" width="49%">

<img src="/photos/motor_gimbal2.jpg" width="49%">

I am using the Arduino PID library an STM32F103C board (you can use an arduino Nano) and a 3D printer to print the 24mm motor gimbal
It uses a bluetooth module (HC-05, HC-06 etc ...)to communicate with an Android device

<img src="/photos/gimbal board 2.jpg" width="49%">

<img src="/photos/gimbal board 1.jpg" width="49%">

Use it with my Gimbal console application
https://github.com/bdureau/MotorGimbalConsole

I also have another version with the Bosch BNO055 sensor which is a lot less complex
https://github.com/bdureau/RocketMotorPIDGimbal_bno055/
