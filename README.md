
# Project Description  
#### By: Piotr Peksa, Cole Deck, Muhammad Bilal Aijaz
Our team worked on designing an automated smart robot assistant capable of completing various household tasks. The chassis of the robot was premade by the IIT Robotics Club, and was used to add on hardware features such as cameras, a microphone, and a claw as the manipulator. The types of tasks performed by the robot include: opening/closing a lever-style door handle/knob, getting a drink from the fridge, and picking up a glass cup/bottle. To get the robot to perform these tasks, the software implemented were voice to text API, path planning, object detection, and object pose estimation. To achieve this goal, we used a Jetson Orin Nano to process all of the software, multiple cameras installed on the robot for better depth perception, and a robotic claw for completing certain tasks.


## Calibrate.py
* Takes a 9x6 checkerboard, and finds the corners in the image(s)
* Calculates the distortion coefficients, camera matrices, and rotational/translational vectors for left and right camera
* Computes disparity map to estimate depth of objects
## Object_Detection_node.py
* Uses custom model to detect specific objects
* Converts detected object to CV image
* Estimates pose(depth) of detected object
* Publishes the pose of the object as a topic
* Initiates voice command for finding a door or glass cup
## Stereo_Cam_Node.py
* Access video frames of left and right camera
* Converts CV images to ROS images
* Publishes the images as a topic
