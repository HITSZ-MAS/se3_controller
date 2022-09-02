# SE(3) Controler for Quadrotor



## 1 Usage

```
cd catkin_ws/src
git clone https://github.com/HITSZ-MAS/se3_controller.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
source ./devel/setup.zsh
roslaunch px4 mavros_posix_sitl.launch
rosrun se3_controller se3_controller_example_node
```

see se3_example.cpp for more details.

![se3_example](attachments/se3_example.gif)

## 2 Theory

### 2.1 Differential Flatness

Calculate $\boldsymbol{q}_{d}$ and $\boldsymbol{\omega}_{d}$ through $\boldsymbol{a}_{cmd}$ , $\boldsymbol{j}_{cmd}$ , $\psi$ and $\dot{\psi}$ .

$$
\begin{aligned}
\boldsymbol{\alpha} &= \boldsymbol{a}_{cmd}+g \boldsymbol{z}_{\mathcal{w}} \\
\boldsymbol{x}_{\mathcal{c}} &=[\cos \psi, \sin \psi, 0]^T \\
\boldsymbol{y}_{\mathcal{C}} &=[-\sin \psi, \cos \psi, 0]^T \\
\boldsymbol{x}_{\mathcal{B}} &=\frac{\boldsymbol{y}_{\mathcal{c}} \times \boldsymbol{\alpha}}{\left\|\boldsymbol{y}_{\mathcal{c}} \times \boldsymbol{\alpha}\right\|} \\
\boldsymbol{y}_{\mathcal{B}} &=\frac{\boldsymbol{\alpha} \times \boldsymbol{x}_{\mathcal{B}}}{\left\|\boldsymbol{\alpha} \times \boldsymbol{x}_{\mathcal{B}}\right\|} \\
\boldsymbol{z}_{\mathcal{B}} &=\boldsymbol{x}_{\mathcal{B}} \times \boldsymbol{y}_{\mathcal{B}} \\
\boldsymbol{q}_d &= Quaternion([\boldsymbol{x}_{\mathcal{B}}, \boldsymbol{y}_{\mathcal{B}}, \boldsymbol{z}_{\mathcal{B}}]) \\
a_z &=\boldsymbol{z}_{\mathcal{B}}^T\left( \boldsymbol{a}+g \boldsymbol{z}_{\mathcal{W}}\right) \\
\boldsymbol{\omega}_d &=
\left[\begin{array}{ccc}
0 & a_z & 0 \\
a_z & 0 & 0 \\
0 & -\boldsymbol{y}_{\mathcal{C}}^T \boldsymbol{z}_{\mathcal{B}} & \left\|\boldsymbol{y}_C \times \boldsymbol{z}_{\mathcal{B}}\right\|
\end{array}\right]^{-1} 
\left[\begin{array}{c}
 \boldsymbol{x}_{\mathcal{B}}^T \boldsymbol{j}_{cmd} \\
-\boldsymbol{y}_{\mathcal{B}}^T \boldsymbol{j}_{cmd} \\
\dot{\psi} \boldsymbol{x}_{\mathcal{C}}^T \boldsymbol{x}_B
\end{array}\right]
\end{aligned}
$$

see "Differential Flatness  of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of  High-Speed Trajectories" or "Control of Quadrotors Using the Hopf Fibration on SO(3)" for more details.

### 2.2 Control Law-PD control

$$
\begin{aligned}
&e_p=p-p_d \\
&e_v=v-v_d \\
&e_a=-k_{p,p} e_p-k_{p,v} e_v-k_{d,p} \dot{e}_p-k_{p,d} \dot{e}_v \\
&e_q=\left(\boldsymbol{q} \otimes \boldsymbol{q}_{d}^{-1}\right)_{x, y, z} \\
&e_{\omega}=\omega-R^T R_d \omega_d
\end{aligned}
$$

Where, subscript d indicates that it's the desired value.

$$
\begin{aligned}
a_{cmd}&=a_d-k_{p,p} e_p-k_{p,v} e_v-k_{d,p} \dot{e}_p-k_{p,d} \dot{e}_v + g e_3 \\
a_{z,cmd}&= a_{cmd}^TRe_3 \\
j_{cmd}&= j_d - k_{p,a}e_a - k_{d,a}\dot{e}_a \\
\omega_{cmd}&=\omega_d-k_{p,q} e_q-k_{p,\omega} e_{\omega}-k_{d,q} \dot{e}_q-k_{d,\omega} \dot{e}_{\omega} \\
\end{aligned}
$$

### 2.3 Thrust Normalization

Assume that

$$
t_{cmd}=\frac{a_{z,cmd}}{T_a}
$$

where $T_a$ is the normalization constant, which is determined by the physical characteristics of the quadrotor, and can be estimated by Kalman filtering

$$
\begin{aligned}
x_k&=T_{a,k} \\
z_k&=a_z=t_{cmd}*T_{a,k}
\end{aligned}
$$

Then

$$
\begin{aligned}
\breve{P}_k&=1/\rho \\
K_k&=\frac{\breve{P}_k\cdot t_{cmd}}{t_{cmd}\times \breve{P}_k\cdot t_{cmd}+\rho} \\
\hat{T}_{a,k}&=\breve{T}_{a,k}+K_k(a_{z,imu}-t_{cmd} \breve{T}_{a,k}) \\
P_k&=(1-K_k\cdot t_{cmd})\cdot \breve{P}_k \\
\end{aligned}
$$

## Reference

M. Faessler, A. Franchi, and D. Scaramuzza, "Differential Flatness  of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of  High-Speed Trajectories," IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 620–626, 2018.

M. Watterson, and V. Kumar, "Control of Quadrotors Using the Hopf  Fibration on SO(3),"  Robotics Research., pp. 199–215, 2020.

T. Lee, M. Leok, and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3)," IEEE Conference on Decision and Control, pp. 5420–5425, 2010. 

https://github.com/ZJU-FAST-Lab/Fast-Drone-250/blob/master/src/realflight_modules/px4ctrl/src/controller.cpp
