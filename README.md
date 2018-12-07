# Documentation computer_vision_pkg - Package
This is the computer vision repository of project ebabil


## Convinient hacks

To connect the Raspberry Pi to your computer directly via ethernet follow step by step:

https://stackoverflow.com/questions/16040128/hook-up-raspberry-pi-via-ethernet-to-laptop-without-router
  
 
To edit files with atom over ssh follow:

https://atom.io/packages/remote-atom

And do `ssh -R 52698:localhost:52698 ubuntu@10.42.0.211` to connect properly. Do `ratom sample.py` to open the file. Dont forget to start server in the Atom editor.

## ROS Package

To install the package in your ROS environment, clone the package `name_of_package` in the `../catkin_ws/src/` directory and run `catkin_make` in `../catkin_ws/`.

## Usage

With active roscore, run `rosrun rosserial_python serial_node.py /dev/ttyACM0`, where `/dev/ttyACM0` is the mount directory of your Arduino. You need to get this information from the Arduino IDE, in Tools > Port. This will start the Arduino communication as a ROS Node.

With `rostopic pub servo std_msgs/UInt16  <angle>` you can now move the servo replacing `<angle>` with the angle in degree as integer.

To run the cv_node run `rosrun computer_vision cv_node` and enjoy the show!

Contact te.ozkaya@gmail.com for support.
