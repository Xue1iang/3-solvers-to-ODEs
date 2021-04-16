# 3-solvers-to-ODEs

In this task, there are three ODE solvers for mathematical model of MallARD. The first solver is named 'ode-45', which is a built-in solver in MATLAB. The rests are built in addition using Runge-Kutta method and euler's method. These models plot some figures out, including surge velocity(u), sway velocity(v) and yaw rate(r). The result is compared with the Figure.8 in 'Model Identification of a Small Fully-Actuated Aquatic Surface Vehicle Using a Long Short-Term Memory Neural Network'.

All of the three models are tested using the datasets form Marin and Keir. Dataset "X" is predominantly excited in X_b by control inputs in F_u. Dataset "Y" and "PSI" excitation is focused on Y_b and by F_v and F_r, respectively.  In the datasets "XTPSI", MallARD was excited in all degrees of freedom and was driven in squares, circles and figures of eight, which are motions that could be expected in practice. 

## Clone the repository

    git clone https://github.com/Xue1iang/3-solvers-to-ODEs.git

This repo contains a package of datasets, and three MATLAB codes of ODE solver.

## Explanation of the code
There are three MATLAB files. The firt one is a built-in solver in matlab, namd ODE45. Second one is Runge-kutta and third on is Euler's solver. Three files that all do he same thing using different methods.

First, pre-processing the data to setup the time. And then, using the solver to solve the ordinary differential equation. Calculate the yaw angle via integrating the yaw rate. Mutiply the rotation matrix to transfer to global frame.

The code uses experimental data from MallARD. Joypad input and MallARD position are recorded in the experiments. The mathematical model predicts velocities and position using the joypad inputs and identified model coefficients. Model and coefficients were taken from 'Model Identification of a Small Omnidirectional Aquatic Surface Vehicle: a Practical Implementation'.

## To run the MallARD simulation
Before your running, do not forget to change the address in xlsread('') for choosing the different datasets. After that, run the code and a 6-pack plot and a quiverplot will come out.

## Explanation about plots

### OED45
1. Plots show the measured velocity v in Yb taken from dataset '\XYPSI\XYPSI_1\' against the estimated velocity in Yb for the classical parametric model. Fv is the control input. Yb is the y axis in body-fixed frame.

![OED_45](https://user-images.githubusercontent.com/77399327/114861698-2f9ea400-9e20-11eb-96ad-68707e82ad66.jpg)


### Runge-Kutta Method
This method is mainly applied when the derivative and initial value information of the equation are known and the computer simulation is used to save the complicated process of solving differential equations. The accuracy of this solver can be changed via changing the step.
![Runge-kutta](https://user-images.githubusercontent.com/77399327/114861777-4ba24580-9e20-11eb-8223-89e2758a762b.jpg)


### Euler's Method
Euler's method is an important type of numerical solution. This type of method avoids solving the function expression, but seeks its approximate value on a series of discrete nodes. The distance between two adjacent nodes is called the step. The accuracy of this solver can be changed via changing the step.
![Euler's method](https://user-images.githubusercontent.com/77399327/114861788-4f35cc80-9e20-11eb-9fee-735ae23f09ba.jpg)

