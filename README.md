# Basics of Mobile Robotics - PROJECT

**Students:** Rim El Qabli, Elias Da Ros, Mehdi Peci, Luis Oscar Rodriguez Jimenez

---

## Project Description

This project was divided into four steps:

---

### 1. Create an Environment

An environment has to be created for the Thymio. This environment must contain a set of obstacles that the Thymio would avoid by using the global navigation, which are detected only by the camera during the initialization step, without using the sensors.

---

### 2. Find the Best Path

The Thymio and the goal have to be placed at random positions in the map (environment). From this start position, the Thymio has to find the best path to reach the goal. The setup can be changed multiple times to show the robustness of the program. As long as the robot is not moving the obstacles, the goal, and the starting position of the robot can be changed at your convenience.

#### Example:

![Change of setup](./img/chgt_setup.gif)

---

### 3. Motion Control & Position Estimation

The Thymio has to be controlled to help it follow the calculated path correctly. A filter will then make it possible to estimate the position. In the GIFs below:
- The position estimated by the camera is shown in **red**.
- The position estimated by the encoders is shown in **yellow**. 

At the beginning, the encoders start at position (0,0) and then converge to the position of the robot. The final position of the robot is estimated using a **Kalman filter**, combining the positions of the camera and the encoders, and is shown in **green**.

#### Two Estimations vs. Final Estimation:

| Two Estimations                                      | Final Estimation                                   |
| :--------------------------------------------------:| :-----------------------------------------------:|
| ![Two Estimations](./img/base_configuration.gif)    | ![Final Estimation](./img/base_vert.gif)          |

---

#### Special Cases:

- **Kidnapping:** The robot can be moved to a different location.  
- **Position Estimation Using Only Encoders:** The camera tracking can be deactivated, and the position is estimated solely using the encoders.

| Kidnapping                                          | Position Estimation Using Only Encoders          |
| :-------------------------------------------------:| :-----------------------------------------------:|
| ![Kidnapping](./img/kidnapping.gif)                | ![Only Encoders](./img/encodeurs.gif)            |

---

### 4. Obstacle Avoidance

Once the robot is following the pre-calculated path, physical obstacles can be placed in its way. The robot will detect and avoid these obstacles.

#### Example:

![Obstacle Avoidance](./img/local_obstacle.gif)

---
