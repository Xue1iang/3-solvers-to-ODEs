# 3-solvers-to-ODEs

In this task, there are three ODE solvers for mathmatical model of MallARD. The first solver is named 'ode-45', which is a built-in solver in MATLAB. The rests are built in addition using Runge-Kutta method and euler's method. These models plot some figures out, including surge velocity(u), sway velocity(v) and yaw rate(r). The result is compared with the Figure.6 in IROS_2020 paper using dataset "XYPSI_1".

All of the three models are tested using the datasets form Marin and Keir. Dataset "X" is predominantly excited in X_b by control inputs in F_u. Dataset "Y" and "PSI" excitation is focused on Y_b and by F_v and F_r, respectively.  In the datasets "XTPSI", MallARD was excited in all degrees of freedom and was driven in squares, circles and figures of eight, which are motions that could be expected in practice. 

## Clone the repo

    git clone https://github.com/Xue1iang/3-solvers-to-ODEs.git

This repo contains a package of datasets, and three MATLAB codes of ODE solver.

## Plots

### OED45
1. Plot F_r and r_simulation r_experiment for a dataset where excitation of predominantly in yaw
![untitled](https://user-images.githubusercontent.com/77399327/114142353-70407e00-9945-11eb-9677-908e65c1ff68.jpg)

2. Quiverplot
![quiver](https://user-images.githubusercontent.com/77399327/114143163-60756980-9946-11eb-86bd-22f4f4420c27.jpg)

### Runge-Kutta Method
The accuracy of this solver can be changed via changing the step.
![RK](https://user-images.githubusercontent.com/77399327/114151689-f5309500-994f-11eb-8f6f-a8540390d09c.jpg)

### Euler's Method
![euler](https://user-images.githubusercontent.com/77399327/114153763-40e43e00-9952-11eb-870b-cee163689a25.jpg)
