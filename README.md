# Diffbot_results
This tracked robot has a two stage wooden top and plenty of room for electronics.
Main brain is Raspberry Pi model 4 with 4 Gig memory, running Ubuntu Mate (20.04)
Teensy 3.2 handles hardware interface, now doing motor control and encoder ticks.
L298N handles motor power. (One from China that costs less than $5US.)
YDLidar is for mapping and Realsense D435 camera for video and depth views.
UPDATE 
We had to drop the L298N and go with Sabertooth 2x12 motor controller. Found that
wires to motors were too light, needed to be heavier gauge. Also changed from tracked
chassis to one with four motors and four wheels. This tracked chassis had the habit
of "skipping" cogs when the going got tough. This throws off the odometry. Also
added the Jetson TX1 to underside of upper platforn, and now using the ZED camera.
So many changes! I will post new photos soon. Trying to stream video back to Oculus Quest 2
VR headset from the ZED. Tilting and turning the head will control camera pan & tilt.
Will use Unity, ROS, and ZED.
