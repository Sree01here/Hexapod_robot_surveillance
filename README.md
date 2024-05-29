HEXAPOD ROBOT

This arduino code is for the movement of hexapod robot for surveillance. This robot has 18 servo motors in it and these servo are connect to PCA9685 16-channel servo driver.
DHT 11 and MQ6 sensor is also connected to this robot for monitoring changes in the environment. Further we can develop this robot by intergrating camera and LIDAR for spacial mapping.
This Hexapod robot is used for surveillance and also for fire and rescue operation.

The Hexapod_movement.ino file just contains the forward, backward, left, right, clockwise and anti-clockwise movement commands.
The Hexapod_robot_surveillance.ino file is the integration of DHT11 and MQ6 sensor with the hexapod movemnt code.

This project final goal is to create a computer vision that can analyse the terrain and can adjust its movement to it and also to have useful end-effectors in the ends of its leg to perform some task such as:
soldering, pick and place, etc.
