clear
close all

data_joystick = xlsread("C:\Users\lenovo\Desktop\joystick.csv");
data_pos = xlsread("C:\Users\lenovo\Desktop\pose_slam.xlsx");
%% Parameters struct

p =struct('x_uc', -0.20, ...       % linear drag coefficient
          'x_uuc', -1.74, ...      % nonlinear drag coefficient
          'x_fc', 0.14, ...        % x_fc = x_f / m, x_f is linear force coefficient
          'y_vc', -0.23, ...       % linear drag coefficient
          'y_vvc', -0.80, ...      % nonlinear drag coefficient
          'y_fc', 0.20, ...        % y_fc = y_f / m, y_f is linear force coefficient
          'n_rc', -0.48, ...       % n_rc = n_r / I_z, n_r is linear drag coefficient, I_z is the rotational inertia about the Zb axis
          'n_rrc', -0.20, ...      % nonlinear drag coefficient
          'n_fc', 1.3,  ...        % n_fc = n_f / I_z, n_rr, n_f is the linear force coefficient
          'n_udc', 0.04, ...       % n_udc = m * y_g / I_z, y_g is the coordinate of the centre of gravity in the Yb axis
          'n_uuc',0.63);           % n_uuc = n_uu / I_z, n_uu is the nonlinear drag codfficient
%% Data input

% setup

f_u = data_joystick(:, 2);
f_v = data_joystick(:, 3);
f_r = data_joystick(:, 7);

n = length(data_pos);

% Time set
ts = (data_pos(:, 1) - data_pos(1, 1))/1000000000;
ts = ts(1:end, :);
tf = (data_joystick(:, 1) - data_pos(1, 1))/1000000000;

% remove the NaN occuring due to mismatches
f_u = interp1(tf, f_u, ts);
f_u(1) = f_u(3);
f_u(2) = f_u(3);
f_u(n) = f_u(n-1);

f_v = interp1(tf, f_v, ts);
f_v(1) = f_v(3);
f_v(2) = f_v(3);
f_v(n) = f_v(n-1);

f_r = interp1(tf, f_r, ts);
f_r(1) = f_r(3);
f_r(2) = f_r(3);
f_r(n) = f_r(n-1);
%% Solve the equations

fs = 15;
x0 = [0, 0, 0];          % initialization of u, v, r
tspan = [0:1/fs:n];
opts = odeset('RelTol',1e-18,'AbsTol',1e-18);    % set the threshold of error

[t, x] = ode45(@(t, x) f(t, x, f_u, f_v, f_r, ts, p), tspan, x0);  % using ode45 solver to solve the ODEs
%% Smoth angular velocity

% % using moving average filter to elimate the spikes
% 
% yawVelocity = x(:,3);          % yaw rate
% 
% % moving window smooth filter 
% N = 20;  % window size
% yawVelocity_smooth = movmean(yawVelocity, N);
%% Calculate the yaw angle

% intergrate the yaw rate to get the yaw angle. In this case, cumtrapz()
% is used to intergrate.

n = length(x(:, 3)); 
yawAngle = zeros(n ,1);                          % initializaion of yaw

% integrate the yaw rate to get yaw angle
yawAngle = cumtrapz(t, x(:, 3)); 
%% Calculate the trajectory

% To get the position (x, y) of the robot, it needs to be transfered from
% body-fixed frame to global frame via multiplying with rotation matrix.
% And then, intergrating the velocity to get the displacement

x_ = cos(yawAngle) .* x(:, 1) - sin(yawAngle) .* x(:, 2);    % x_velocity in earth-fixed frame
y_ = sin(yawAngle) .* x(:, 1) + cos(yawAngle) .* x(:, 2);    % y_velocity in earth-fixed frame

% integrate the velocity to get the displacement
positionGlobalX = zeros(n, 1);

positionGlobalX = cumtrapz(t, x_);

positionGlobalY = zeros(n, 1);

positionGlobalY = cumtrapz(t, y_);


 
 h1 = plot(positionGlobalX,positionGlobalY,'b--o');
 legend(h1, 'Motion trajectory')
 title('Motion trajectory (global frame)')
 xlabel('X')
 ylabel('Y','rotation',0)
 axis equal
%% Plot body-fixed frame

figure


plot(x(:, 1))
title('u (surge velocity)')
xlabel('Time points');
xlim([1100 1610])
ylabel('u(m/s)','rotation',0);
ylim([-0.4 0.4])


plot(x(:, 2))
title('v (sway velocity)')
xlabel('Time points');
xlim([1100 1610])
ylabel('v(m/s)','rotation',0);
ylim([-0.4 0.4])


plot(x(:, 3))
title('yaw Velocity')
xlabel('Time points');
xlim([1000 2000])
ylabel('r(rad/s)','rotation',0);
ylim([-1 1])

% subplot(2, 3, 1)
% plot(x(:, 1))
% title('surge velocity (body-fixed frame)')
% xlabel('Time(s)');
% ylabel('u(m/s)','rotation',0);
% 
% subplot(2, 3, 2)
% plot(x(:, 2))
% title('sway velocity (body-fixed frame)')
% xlabel('Time(s)');
% ylabel('v(m/s)','rotation',0);
% 
% subplot(2, 3, 3)
% plot(yawVelocity_smooth)
% title('yaw rate (body-fixed frame)')
% xlabel('Time(s)');
% ylabel('r(rad/s)','rotation',0);
% ylim([-1,1])
% 
% subplot(2, 3, 4)
% plot(f_u)
% title('input force in surge')
% xlabel('Time(s)')
% ylabel('F_u','rotation',0);
% 
% subplot(2, 3, 5)
% plot(f_v)
% title('input force in sway')
% xlabel('Time(s)')
% ylabel('F_v','rotation',0);
% 
% 
% subplot(2, 3, 6)
% plot(f_r)
% title('input force in yaw')
% xlabel('Time(s)')
% ylabel('F_r','rotation',0);
%%
function fx = f(t, x, f_u, f_v, f_r, ts, p)

fx = zeros(3, 1);

f_u = interp1(ts, f_u, t);
f_v = interp1(ts, f_v, t);
f_r = interp1(ts, f_r, t);

fx(1) = x(2) * x(3) + (p.x_uuc * abs(x(1)) + p.x_uc) * x(1) + p.x_fc* f_u; 
fx(2) = -x(1) * x(3) + (p.y_vvc * abs(x(2)) + p.y_vc) * x(2) + p.y_fc * f_v ;
fx(3) = p.n_udc * fx(1) + (p.n_rrc * abs(x(3)) + p.n_rc) * x(3) + p.n_uuc * abs(x(1)) * x(1) + p.n_fc * f_r;

end