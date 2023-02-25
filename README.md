# Teleop_2_wheel_robot_No_PID
The Arduino code has plenty of comments to understand it.
The only change is that when you connect the ROS serial server you need to change the IP address in the code for your laptop's IP everytime.
Like my laptop's IP while uploading the code was 192.168.110.114, so look into it every single time.


# Instructions to run using Keyboard
Connect the laptop's Wifi to the same wifi as that of ESP
Run the following command in another terminal<br />
`roslaunch rosserial_server socket.launch`
<br />
This above command will start the socket connection between the rosserial_sever and the nodes.
<br />
As soon as the server connection is established the subscriber node in the Arduino file is initialised and hence it subscribes to the topic cmd_vel.
To check if the rosserial_server is properly established type the following command <br />
`rostopic list `
<br />
If cmd_vel is running that means the server is properly established.
<br />
In a new terminal run the following code--
<br />
`rosrun teleop_twist keyboard teleop_twist_keyboard.py`
Now you can teleop the bot with the keyboard.


# Instructions to run using Joystick
Everything same as keyboard teleop except before running twist keyboard node you need to run the following command
<br />
`sudo apt install ros-noetic-joy`
<br />
This will install the joy ros package. Joy is the type of msg published by the joystick.
Now in another terminal run--
`rosrun joy joy_node`
This starts the node for joy message
Check the topics through rostopic list command.
If joy message is present that means ros is able to publish the messages from the keyboard through joy node
<br />
Now in a new terminal run the following command--
<br />
`rosrun teleop_twist_joy teleop_node `
<br />
Now the bot can be controlled with the controller.
But the button A should be keep pressed while changing the axes because according to the package the value is published when button 0 is pressed and in our case the button zero is A
(different for every controller visit the ros website for more information)
