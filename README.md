# crustcrawler_ax12

Description, controller, MoveIt configuration, and demo files for the Crustcrawler AX-12 robotic arm.

## Hardware

The arm, with seven AX12 servos included:
[http://www.crustcrawler.com/products/AX12A%20Smart%20Robotic%20Arm/](http://www.crustcrawler.com/products/AX12A%20Smart%20Robotic%20Arm/)

The USB2AX controller, used to interface between the motors and a usb port:
[http://www.crustcrawler.com/products/AX12A%20Smart%20Robotic%20Arm/](http://www.crustcrawler.com/products/AX12A%20Smart%20Robotic%20Arm/)

Note, before you do anything with it, you have to grant non-root users read write permissions for the USB2AX. I do this with this command:
    sudo chmod 777 /dev/ttyACM0

## Usage

To just see a basic representation of the robot, type (in a separate tab from where you have roscore running):
    roslaunch ax12_description ax12_rviz.launch

Once you have the arm hooked up to your computer, to see a demo of the arm executing some MoveIt trajectories, first download
[MoveIt Commander from GitHub](https://github.com/ros-planning/moveit_commander) and put it in your catkin workspace. Then run:
