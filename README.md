# Hybrid Legged-Wheel Self-Balancing Robot
**Department of Mechatronics Engineering - Control Systems I**

---

##  Project Overview
This project involves the development of a hybrid locomotion system that combines the stability of an inverted pendulum with the adaptability of bipedal kinematics. By integrating two legs with wheeled feet, the robot can perform height adjustments, X-Y planar navigation, and obstacle avoidance while maintaining vertical equilibrium. The dynamic modeling is based heavily on a four-link geometric-dynamic coupling approach.

---

##  System Architecture
The project is divided into four main engineering pillars:

*   **Mechanical Design:** A 2-DOF per leg assembly designed in SolidWorks, optimized for a low Center of Mass (CoM) and minimal inertia. Actuation is handled by Stepper motors (for wheel drive/X-Y motion) and Servo motors (for leg articulation).
*   **Mathematical Modeling:** Derivation of the system's equations of motion using Lagrangian Dynamics, resulting in a non-linear state-space model based on a dual-wheel inverted pendulum and four-link kinematics.
*   **Control Engineering:** Implementation of a nested loop control architecture:
    *   **Inner Loop:** leg height adjustment, and virtual model control (VMC).
    *   **Outer Loop:** LQR (Linear Quadratic Regulator) for optimal tilt stabilization and disturbance rejection.
*   **Embedded Systems:** Built around the ESP32 microcontroller, featuring real-time sensor fusion (Kalman/Complementary filter) using an MPU6050 IMU, high-frequency motor control, and an integrated Ultrasonic sensor for environmental awareness.

---

## Control Strategy
To design the LQR controller, the system was linearized around the upright equilibrium point ($\theta = 0$).

**State Vector:**
$$x = \begin{bmatrix} \theta & \dot{\theta} & X & \dot{X} \end{bmatrix}^T$$

Where $\theta$ is the pitch angle and $v$ is the linear displacement. 

**Objective:**
Minimize the cost function $J$ to find the optimal gain matrix $K$:
$$J = \int_{0}^{\infty} (x^T Q x + u^T R u) dt$$

---

## 📁 Repository Navigation
*   **/hardware:** SolidWorks .SLDASM files, 3D printing STL files.
*   **/simulation:** .slx Simulink models and MATLAB scripts .
*   **/firmware:** Source code for the ESP32 (written in C++), featuring I2C MPU6050 drivers, Ultrasonic reading logic, and the control loop.
*   **/docs:** Reference papers, the final project report, and the mathematical derivation of the plant.
*   **/media:** Videos , pictures.

---

##  Key Technical Features
*   **Dynamic Balancing:** Active rejection of external forces via LQR.
*   **Variable Height:** Leg kinematics allow the robot to maintain balance while changing its total height.
*   **Obstacle Avoidance:** Integration of ultrasonic sensing allows the robot to detect and navigate around impediments.
*   **Omnidirectional Planar Motion:** Differential drive logic allows for precise X, Y positioning and steering.
*   **Sim-to-Real Verification:** High correlation between the Simulink model and the physical hardware performance.

---

## 👥 Team Members
*   **Doaa Essam , Amr Mohamed , Alyaa Mohammed , Abdelrahman Hatem , Esraa Mohamed , Mina Sameh** 
## Under supervision of : 
*   **Dr. Shuaiby Mohammed** 

