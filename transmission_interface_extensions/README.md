# transmission_interface_extensions
Plugins that enhance ros_control's transmission_interface

## Additional joint interface support
With transmission_interface_extensions installed, joint interfaces below are additionally supported in <transmittion> tag in URDF files.
* hardware_interface/PosVelJointInterface
* [hardware_interface_extensions](../hardware_interface_extensions)/PosVelEffJointInterface

## Slider-crank transmission
Also slider-crank transmittions are supported. See the figure below & [urdf/slider_crank_example.urdf](urdf/slider_crank_example.urdf) to know how to use it.

![](https://raw.githubusercontent.com/yoshito-n-students/ros_control_extensions/images/images/slider_crank.png)