# Path Planning of Three Wheeled Omni-Directional Robot Using Bezier Curve Tracing Technique and PID control Algorithm
I have carried out path planning of a 3-wheeled Omni-drive through a technique used primarily in computer graphics design, called Bezier Curve. I have used a [Gyro 6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) for maintaining the orientation of the drive, 2 [Baumer Encoders](https://www.baumerdistributors.com/baumer-eil580-sc10-5le-01024-a-incremental-encoder.html) for measuring the relative position and distance covered by the drive, and [STM32F407VG-Discovery board](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) for the microcontroller.

# Bezier Curve:
Bezier Curve uses a set of control points for deciding its ‘curve’. In this example, I have used 4 control points. The curve was implemented using De Casteljau’s algorithm. [More information about Bezier curve and implementing De Casteljau’s algorithm.](https://javascript.info/bezier-curve)

# Specifications:
- The Motor controller used for this project was [Roboclaw](https://www.basicmicro.com/RoboClaw-2x30A-Motor-Controller_p_9.html). The header file  stm32f4xx_roboclaw.h is a custom made file for controlling the roboclaw using the STM board.
- The drive maintains its stability using PID  control algorithm. Specifically : PD control algorithm(as I have not used Integral ).The Gyro sensor is connected to an Atmega328p chip and sends its data to the Atmega328p, which in turn, uses UART to send the data to the STM.The 'yaw' has a range of 0~200 degrees.  
- The encoder used have a PPR of 1024 and are placed perpendicular to each other,in the center of the drive. This ensures that wheel-slippage doesn't affect the estimated position and gives the absolute x and y of the drive in the world frame.
- The drive traces out a semi circle of 2 metre diameter using 4-control point algorithm.
- A wireless controller using Zigbee/Bluetooth(works with both) is also used for switching between Curve tracing and Joystick Control 

# Future-work:
- Two 1-D [Lidars](http://en.benewake.com/product/detail/5c345cc2e5b3a844c472329a) are mounted on the X and Y axis of the drive. Currently, they only act as a proximity sensor for obstacle avoidance(if the drive gets too close to an obstacle,it stops moving).
- These lidars can also be used to obtain the absolute position of the drive in an environment,combining these with the relative positional data obtained from the encoders and IMU, SLAM can also be implemented.
