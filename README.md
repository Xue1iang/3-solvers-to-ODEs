# 3-solvers-to-ODEs

In this task, there are three ODE solvers for mathmatical model of MallARD. The first solver is named 'ode-45', which is a built-in solver in MATLAB. The rests are built in addition using Runge-Kutta method and euler's method. These models plot some figures out, including surge velocity(u), sway velocity(v) and yaw rate(r). The result is compared with the Figure.6 in IROS_2020 paper using dataset "XYPSI_1".

All of the three models are tested using the datasets form Marin and Keir. Dataset "X" is predominantly excited in X_b by control inputs in F_u. Dataset "Y" and "PSI" excitation is focused on Y_b and by F_v and F_r, respectively.  In the datasets "XTPSI", MallARD was excited in all degrees of freedom and was driven in squares, circles and figures of eight, which are motions that could be expected in practice. 

## Clone the repo

    git clone https://github.com/Xue1iang/3-solvers-to-ODEs.git

This repo contains a package of datasets, and three MATLAB codes of ODE solver.

## 
