# Parallel-Robot-Kinematics
Inverse and Forward Kinematics of a 6 DOF Parallel Robot

## Introduction
This document describes the kinematics of a 6-DOF (degrees of freedom) parallel robot, designed for MRI-guided interventions, which uses pneumatic actuators to drive its movement. The parallel robot is based on the Stewart-Gough platform, a popular configuration for high-precision applications requiring multiple degrees of freedom.

## Robot Design

### Structure
The robot's structure is composed of three main components:
1. A fixed base.
2. A moving platform.
3. Six identical prismatic limbs.

Each limb is connected to the base via a **universal joint (U)** and to the moving platform via a **spherical joint (S)**, resulting in a 6-UPS configuration. By altering the length of the limbs, the position and orientation of the moving platform are controlled.

### Inverse Kinematics
Inverse kinematics for this robot involve calculating the required lengths of the six limbs to position the moving platform at a desired pose. The fixed base and moving platform are assigned coordinate frames \( B \) and \( P \) respectively, with vectors defined for the joint attachment points on both frames.

The position vector of the platform in the base frame is:

\[
\mathbf{B}_{d} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
\]

The orientation of the platform is described by three Euler angles: Roll (\( \phi \)), Pitch (\( \theta \)), and Yaw (\( \psi \)). The rotation matrix relating the base frame to the moving platform is expressed as:

\[
\mathbf{B R}_{P} = R_z(\psi) R_y(\theta) R_x(\phi)
\]

where \( R_z(\psi) \), \( R_y(\theta) \), and \( R_x(\phi) \) are rotation matrices for each Euler angle.

The length of each limb \( l_i \) is calculated as:

\[
l_i = \| \mathbf{B}_{d} + \mathbf{B R}_{P} \cdot \mathbf{P}_{pi} - \mathbf{B}_{bi} \|
\]

where:
- \( \mathbf{B}_{d} \) is the position vector,
- \( \mathbf{B R}_{P} \) is the rotation matrix,
- \( \mathbf{P}_{pi} \) is the position of the passive joint on the platform,
- \( \mathbf{B}_{bi} \) is the position of the passive joint on the base.

### Jacobian Matrix
The Jacobian matrix \( J \) relates the limb velocities to the twist of the robot's end-effector:

\[
\dot{L} = J \dot{X}
\]

Where:
- \( \dot{L} = \begin{bmatrix} \dot{l}_1 & \dot{l}_2 & \cdots & \dot{l}_6 \end{bmatrix}^T \) is the vector of limb velocities,
- \( \dot{X} = \begin{bmatrix} \dot{x} & \dot{y} & \dot{z} & \dot{\phi} & \dot{\theta} & \dot{\psi} \end{bmatrix}^T \) is the twist of the end-effector.

The Jacobian matrix is a 6x6 matrix derived from the velocity loop closure equations for each limb, capturing the relationships between the robot's limb movements and its platform motion.

### Forward Kinematics
Forward kinematics determine the platform's pose (position and orientation) given the current lengths of the six limbs. However, the forward kinematics problem for this parallel robot is non-trivial, with no closed-form solutions due to the highly non-linear equations involved.

The **Newton-Raphson** iterative method is employed to solve the forward kinematics, where an initial guess for the platform pose is iteratively updated until the limb length errors converge.

## Conclusion
The parallel robot described here features a robust kinematic design, capable of achieving accurate 6-DOF motion within the MRI environment. Its Stewart-Gough platform structure allows for precise control of the platform's position and orientation through the manipulation of six prismatic actuators, guided by inverse kinematics and Jacobian-based control methods.
