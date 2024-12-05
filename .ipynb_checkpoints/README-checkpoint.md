# Basics of Mobile Robotics - PROJECT

**Students :  Elias Da Ros, Rim El Qabli, Mehdi Peci, Luis Oscar Rodriguez Jimenez**

## Project Description

This project was divided in four steps :
- **Create an environement:** An environment has to be created for the Thymio. This environment must contain a set of obstacles that the Thymio would avoid by using the global navigation which are detected only by the camera during the initialization step, without using the sensors.

- **Find the best path:** The Thymio and the goal have to be placed at a random position in the map (environement). From this start position, the Thymio has to find the best path to reach the goal. The setup can be changed multiple times to show the robustness of the program, as long as the robot is not moving the obstacles, the goal and the starting position of the robot can be changed at your convinience. This process is demonstated below:
![change of setup](./img/chgt_setup.gif)


- **Motion Control & Position Estimation:** The Thymio has to be controlled to help it follow correctly the found path. A filter will then make it possible to estimate the position. On the gifs below the position estimated by the camera is shown in red and the position estimated by the encoders is shown in yellow. At the begining, the encoders start at position (0,0) and then converge to the position of the robot. The final position of the robot is estimated using a Kalman filter with the positions of the camera and the encoders and is shown in green. 


>|Two estimations                                       |Final estimation |
|:-----------------------------------------------:|:-------------------------------------------------------------------------------:|
| ![two_estim](./img/base_configuration.gif) | ![final_estim](./img/base_vert.gif) |

during this steps the robot can be kidnapped and placed somewhere else, or the tracking with the camera can be deactivated and the position is estimated only with the encoders. Those two cases are shown below:

>|Kidnapping                                       |Position estimation using only the encoders |
|:-----------------------------------------------:|:-------------------------------------------------------------------------------:|
| ![Kidnapping](./img/kidnapping.gif) | ![only encoders](./img/encodeurs.gif) |

- **Obstacles Avoidance:** Once the robot is following the pre-calculated path, some physical obstacles can be placed on front of him which he will have to avoid. This is shown below:

![Obstacles avoidance](./img/local_obstacle.gif)