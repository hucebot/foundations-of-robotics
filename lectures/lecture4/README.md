# Inverse Kinematics for a 7-DoF arm
The script ```panda_ik.py``` implements a simple IK with two priority levels for a Franka Panda robot.

The file ```ik.py``` contains a class ```inverse_kinematics``` that implements a *Null-Space*-based IK with *Damped Pseudo-inverse*:

$$\mathbf{\dot{q}}_r = \mathbf{J}^{\sharp}\mathbf{v}_r + \left(\mathbf{I} - \mathbf{J}^{\sharp}\mathbf{J} \right)\mathbf{\dot{q}}_2$$

with:

$$\mathbf{J}^{\sharp} \in \mathbb{R}^{7 \times 6}$$,

the Damped Pseudo Inverse of $\mathbf{J} \in \mathbb{R}^{6 \times 7}$, with eigenvalues $\sigma_i^{\sharp} = \frac{\sigma_i}{\sigma_i^2 + \epsilon}$, 

$$\mathbf{v}_r = \lambda \mathbf{e}$$,

the error in Cartesian space computed using quaternions for the orientation part, and expressed in world frame,

$$\mathbf{\dot{q}}_2 = \lambda_q \left(\mathbf{q}_r - \mathbf{q} \right)$$,

a *postural* task defined in joint-space.

The computed joint velocities are integrated using a simple Euler integration scheme:

$$\mathbf{q}_{i+1} = \mathbf{q}_{i} + dt \mathbf{\dot{q}}_r$$

## Run the script
In the terminal run:

```python3 panda_ik.py```

which will open a ```viser``` client, follow the instructions on the terminal.

## GUI
![gui](resources/panda_ik_viser.png?raw=true)

The GUI displays the actual posture of the robot. 

The *Cartesian Marker* permits to set desired poses of the end effector in ral-time. In the same way the sliders in the top right permits to set a reference posture for the secondary task. To rest the sliders at their original configuration just press the *Reset* button. 

The *Reset IK* button re-initialize the IK at its original configuration (that can be changed at [this line](https://github.com/EnricoMingo/introduction_to_robotics/blob/ec1afb1c8252c8c666b7f695891f3706b693b9f0/lecture4/panda_ik.py#L19C5-L19C53)).

The *IK regularization* slider instead permits to set the value of the $\epsilon$ used to regularize the Pseudo-Inverse.
