# TeCoBot

Code repository for **TeCoBot**, a tensegrity-based continuum modular robot with compliant body deformation, claw-based docking, locomotion, manipulation, loco-manipulation, and autonomous visual docking experiments.

This repository contains the control, sensing, and experiment code used for the manuscript:

**Tensegrity Continuum Robots Enable Task-Adaptive Morphologies for Cooperative Behaviors**  
Accepted in **Nature Machine Intelligence**

## Repository Contents

### Autonomous Docking w Camera

This folder contains the autonomous docking code for TeCoBot. The code uses onboard camera feedback to detect visual markers, align the robot with the docking target, approach the docking station, and actuate the front claw for docking.

### ESP32 Receiver

This folder contains the ESP32 receiver code used on the robot. The receiver communicates with the external transmitter/controller and sends motion commands to the TeCoBot body and claw mechanisms.

### ESP32 Transmitter

This folder contains the ESP32 transmitter/controller code. It is used as the external controller to send locomotion, bending, claw, and docking commands to the ESP32 receiver on the robot.

### CC Validation

This folder contains the Constant Curvature model validation code. The validation setup uses two IMUs and one Time-of-Flight sensor to estimate the robot backbone shape. The IMUs are used to calculate the bending angle and bending plane, while the ToF sensor is used to estimate the effective backbone length. These measurements are used to compare the physical robot deformation with the constant curvature model.

## Citation

Citation information will be added after publication.

## License

This repository is released under the MIT License.
