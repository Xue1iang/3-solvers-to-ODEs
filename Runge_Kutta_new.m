clear
close all

force = xlsread("data\XYPSI\XYPSI_1\joystick.csv");
pose_slam = xlsread("data\XYPSI\XYPSI_1\pose_slam.csv");
%% Setup

% L - number of the position measurements
Pos = pose_slam;
 
[L,W]=size(Pos);
 
X=Pos(:,5);
Y=Pos(:,6);
 
ts=(Pos(:,1) - Pos(1,1))/1000000000;
tsf = [0:1/15:ts(end)]';
tf=(force(:,1)-Pos(1,1))/1000000000;
dt=1/15;
 
Fx=interp1(tf,force(:,2),ts);
%Done to prevent NaN occuring due to mismatches
Fx(1) = Fx(3);
Fx(2) = Fx(3);
Fx(L) = Fx(L-1);

Fy=interp1(tf,force(:,3),ts);
Fy(1) = Fy(3);
Fy(2) = Fy(3);
Fy(L) = Fy(L-1);

Fz=interp1(tf,force(:,7),ts);
Fz(1) = Fz(3);
Fz(2) = Fz(3);
Fz(L) = Fz(L-1);
 
 
%% Set simulator coefficients
co.Yvvc = -0.9901;
co.Yvc = -0.2277;
co.Fy = 0.2254;
 
co.Xuuc = -1.4934;
co.Xuc =-0.2246;
co.Fx = 0.1436;
 
co.Nrrc = -0.6320;
co.Nrc = -0.4187;
co.Fz = -1.6672;

co.Nrxxc = 0.1596;
%% Transform quaternion to yaw angle
i=1;
while i < L+1
    quat=[pose_slam(i,8),pose_slam(i,9),pose_slam(i,10),pose_slam(i,11)];
    eul(i,1:3)=quat2eul(quat);
    i=i+1;
end
yaw=eul(:,3);


%% Find angular velocity 
% The +- 6.28 is done to remove the inaccurate spikes that occur, because
% of the quaternion to euler transformation
% Psi_dot - angular velocity
i=1;
while i< L
    if yaw(i+1,1)-yaw(i,1) >6
        PSI_dot(i,1)=(yaw(i+1,1)-yaw(i,1)-6.28)/dt;
    elseif yaw(i+1,1)-yaw(i,1) <-6
        PSI_dot(i,1)=(yaw(i+1,1)-yaw(i,1)+6.28)/dt;
    else
        PSI_dot(i,1)=(yaw(i+1,1)-yaw(i,1))/dt;
    end
    i=i+1;
end

%% Smooth angular velocity
% Angular velocity is smoothed using 4 points average to filter out some of
% the noise present
i = 1;
while i<L-3
    PSI_dot(i,1)=(PSI_dot(i,1)+PSI_dot(i+1,1)+PSI_dot(i+2,1)+PSI_dot(i+3,1))/4;
    i=i+1;
end

%% Find velocities in body frame from displacement
% X and Y velocities are smoothed using 4 points average to filter out some of
% the noise present
% X_dot_B - body frame X velocity , Y_dot_B - body frame Y velocity
i=2;
while i<L-2
    X_dot_B(i-1,1)=(1/4)*((X(i+3,1)-X(i-1,1))/dt);
    Y_dot_B(i-1,1)=(1/4)*((Y(i+3,1)-Y(i-1,1))/dt);
    x_save=X_dot_B(i-1,1);
    X_dot_B(i-1,1)=X_dot_B(i-1,1)*cos(yaw(i-1,1))+Y_dot_B(i-1)*sin(yaw(i-1,1));
    Y_dot_B(i-1,1)=Y_dot_B(i-1,1)*cos(yaw(i-1,1))-x_save*sin(yaw(i-1,1));
    i=i+1;
end

%% Parameters

p =struct('x_uc', -0.20, ...       % linear drag coefficient
          'x_uuc', -1.74, ...      % nonlinear drag coefficient
          'x_fc', 0.14, ...        % x_fc = x_f / m, x_f is linear force coefficient
          'y_vc', -0.23, ...       % linear drag coefficient
          'y_vvc', -0.80, ...      % nonlinear drag coefficient
          'y_fc', 0.20, ...        % y_fc = y_f / m, y_f is linear force coefficient
          'n_rc', -0.48, ...       % n_rc = n_r / I_z, n_r is linear drag coefficient, I_z is the rotational inertia about the Zb axis
          'n_rrc', -0.20, ...      % nonlinear drag coefficient
          'n_fc', -1.3,  ...        % n_fc = n_f / I_z, n_rr, n_f is the linear force coefficient
          'n_udc', 0.04, ...       % n_udc = m * y_g / I_z, y_g is the coordinate of the centre of gravity in the Yb axis
          'n_uuc',0.63);           % n_uuc = n_uu / I_z, n_uu is the nonlinear drag codfficient
%% Data input

% setup

f_u = force(:, 2);
f_v = force(:, 3);
f_r = force(:, 7);

n = length(pose_slam);

% Time set
ts = (pose_slam(:, 1) - pose_slam(1, 1))/1000000000;
ts = ts(1:end, :);
tf = (force(:, 1) - pose_slam(1, 1))/1000000000;

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


delta = 1/15;         % step
t = 0:delta:500;        % time span
n = length(t);
x(:, 1) = [0; 0; 0];  % initial value of u, v, r

t1 = linspace(0, ts(end), n);

f_u = interp1(ts, f_u, t1);
f_v = interp1(ts, f_v, t1);
f_r = interp1(ts, f_r, t1);

% 4th-order Runge-kutta solver
for k = 1 : n-1
    
    z1 = f(t(k), x(:, k), f_u(k), f_v(k), f_r(k), p);
    z2 = f(t(k) + delta/2, x(:, k) + z1 * delta/2, f_u(k), f_v(k), f_r(k), p);
    z3 = f(t(k) + delta/2, x(:, k) + z2 * delta/2, f_u(k), f_v(k), f_r(k), p);
    z4 = f(t(k) + delta, x(:, k) + z3 * delta, f_u(k), f_v(k), f_r(k), p);
    
    x(:, k+1) = x(:, k) + delta * (z1 + z2 + z3 + z4) / 6;
end

u = x(1, :);  % u-velocity
v = x(2, :);  % v-velocity
r = x(3, :);  % r-yaw rate

%% Calculate the yaw angle

% intergrate the yaw rate to get the yaw angle. In this case, cumtrapz()
% is used to intergrate.

n = length(r); 
yawAngle = zeros(n ,1);                          % initializaion of yaw

% integrate the yaw rate to get yaw angle
yawAngle = cumtrapz(t, r); 
%% Calculate the trajectory

% To get the position (x, y) of the robot, it needs to be transfered from
% body-fixed frame to global frame via multiplying with rotation matrix.
% And then, intergrating the velocity to get the displacement

x_ = cos(yawAngle) .* u - sin(yawAngle) .* v;    % x_velocity in earth-fixed frame
y_ = sin(yawAngle) .* u + cos(yawAngle) .* v;    % y_velocity in earth-fixed frame

% integrate the velocity to get the displacement
positionGlobalX = zeros(n, 1);

positionGlobalX = cumtrapz(t, x_);

positionGlobalY = zeros(n, 1);

positionGlobalY = cumtrapz(t, y_);

% Subsample 
sample_scale = linspace(0, n, n*0.8);
x_subsample = interp1(t, x_, sample_scale);
y_subsample = interp1(t, y_, sample_scale);
positionGlobalX_subsample = interp1(t, positionGlobalX, sample_scale);
positionGlobalY_subsample = interp1(t, positionGlobalY, sample_scale);


X_dot_B = interp1(ts(5 : end), X_dot_B, t1);
Y_dot_B = interp1(ts(5 : end), Y_dot_B, t1);
PSI_dot = interp1(ts(2 : end), PSI_dot, t1);

% change the unit to second
t1 = linspace(0, tsf(end), length(u));
tuv = linspace(0, tsf(end), length(X_dot_B));
t_PSI = linspace(0, tsf(end), length(PSI_dot));
t_force = linspace(0, tsf(end), length(f_u));
%% Plot

time_axis_lims = [0 250];
lin_vel_lims = [-0.5 0.5];

figure(1)

subplot(2,3,1)
plot(tuv, X_dot_B)
hold on
plot(t1, u)
xlim(time_axis_lims)
ylim(lin_vel_lims)
title('u-velocity')
xlabel('time(s)')
ylabel('u(m/s)','rotation',0)
legend('measured velocity', 'Runge-kutta solver')

subplot(2,3,2)
plot(tuv, Y_dot_B)
hold on
plot(t1, v)
xlim(time_axis_lims)
ylim(lin_vel_lims)
title('v-velocity')
xlabel('time(s)')
ylabel('v(m/s)','rotation',0)
legend('measured velocity', 'Runge-kutta solver')

subplot(2,3,3)
plot(t_PSI, PSI_dot)
hold on
plot(t1, r)
xlim(time_axis_lims)
ylim(lin_vel_lims)
title('r-yaw rate')
xlabel('time(s)')
ylabel('r(rad/s)','rotation',0)
legend('measured velocity', 'Runge-kutta solver')

subplot(2,3,4)
plot(t_force, f_u)
xlim(time_axis_lims)
ylim(lin_vel_lims)
title('f_u-force')
xlabel('time(s)')
ylabel('f_u(N)','rotation',0)


subplot(2,3,5)
plot(t_force, f_v)
xlim(time_axis_lims)
ylim(lin_vel_lims)
title('f_v-force')
xlabel('time(s)')
ylabel('f_v(N)','rotation',0)

subplot(2,3,6)
plot(t_force, f_r)
xlim(time_axis_lims)
ylim(lin_vel_lims)
title('f_r-force')
xlabel('time(s)')
ylabel('f_r(N)','rotation',0)

%% displacement plot
figure(2)
quiver(positionGlobalX_subsample, positionGlobalY_subsample, x_subsample, y_subsample)
xlabel('width(m)')
ylabel('length(m)','rotation',0)
hold on
plot(positionGlobalX, positionGlobalY)
title('Displacement(quiver plot)')
%%
function fx = f(t, x, f_u, f_v, f_r, p)

    fx= zeros(3, 1);
    
    fx(1) = x(2) * x(3) + (p.x_uuc * abs(x(1)) + p.x_uc) * x(1) + p.x_fc * f_u; 
    fx(2) = -x(1) * x(3) + (p.y_vvc * abs(x(2)) + p.y_vc) * x(2) + p.y_fc * f_v ;
    fx(3) = p.n_udc * fx(1) + (p.n_rrc * abs(x(3)) + p.n_rc) * x(3) + p.n_uuc * abs(x(1)) * x(1) + p.n_fc * f_r;

end