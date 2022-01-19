# Path Planning of Three Wheeled Omni-Directional Robot Using Bezier Curve Tracing Technique and PID control Algorithm
I have carried out path planning of a 3-wheeled Omni-drive through a technique used primarily in computer graphics design, called Bezier Curve. I have used a [Gyro 6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) for maintaining the orientation of the drive, 2 [Baumer Encoders](https://www.baumerdistributors.com/baumer-eil580-sc10-5le-01024-a-incremental-encoder.html) for measuring the relative position and distance covered by the drive, and [STM32F407VG-Discovery board](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) for the microcontroller.

# Bezier Curve:
Bezier Curve uses a set of control points for deciding its ‘curve’. In this example, I have used 4 control points. The curve was implemented using De Casteljau’s algorithm. [More information about Bezier curve and implementing De Casteljau’s algorithm.](https://javascript.info/bezier-curve)

# Specifications:
- The Motor controller used for this project was [Roboclaw](https://www.basicmicro.com/RoboClaw-2x30A-Motor-Controller_p_9.html). The header file  stm32f4xx_roboclaw.h is a custom made file for controlling the roboclaw using the STM board.
- The drive maintains its stability using PID  control algorithm. Specifically : PD control algorithm(as I have not used Integral ).The Gyro sensor is connected to an Atmega328p chip and sends its data to the Atmega328p, which in turn, uses UART to send the data to the STM.The 'yaw' has a range of 0~200 degrees.  
- The encoder used have a PPR of 1024 and are placed perpendicular to each other,in the center of the drive. This ensures that wheel-slippage doesn't affect the estimated position and gives the absolute x and y of the drive in the world frame.
- The drive traces out a semi circle of 2 metre diameter using 4-control point algorithm.

