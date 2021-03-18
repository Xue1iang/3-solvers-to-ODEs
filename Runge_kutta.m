clear
close all

data_joystick = xlsread("C:\Users\lenovo\Desktop\joystick.csv");
data_pos = xlsread("C:\Users\lenovo\Desktop\pose_slam.xlsx");
%% Parameters

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

f_u = data_joystick(:, 2);
f_v = data_joystick(:, 3);
f_r = data_joystick(:, 7);
L = length(data_pos);
%  Time set
ts = (data_pos(:, 1) - data_pos(1, 1))/1000000000;
tf = (data_joystick(:, 1) - data_pos(1, 1))/1000000000;

f_u = interp1(tf, f_u, ts);
f_u(1) = f_u(3);
f_u(2) = f_u(3);
f_u(L) = f_u(L-1);


f_v = interp1(tf, f_v, ts);
f_v(1) = f_v(3);
f_v(2) = f_v(3);
f_v(L) = f_v(L-1);

f_r = interp1(tf, f_r, ts);
f_r(1) = f_r(3);
f_r(2) = f_r(3);
f_r(L) = f_r(L-1);


delta = 0.001;        % step
t = 1200:delta:1800;  % time span
n = length(t);
x(:, 1) = [0; 0; 0];  % initial value of u, v, r

t1 = linspace(0, ts(end), n);

f_u = interp1(ts, f_u, t1);
f_v = interp1(ts, f_v, t1);
f_r = interp1(ts, f_r, t1);

for k = 1 : n-1
    
    z1 = f(t(k), x(:, k), f_u(k), f_v(k), f_r(k), p);
    z2 = f(t(k) + delta/2, x(:, k) + z1 * delta/2, f_u(k), f_v(k), f_r(k), p);
    z3 = f(t(k) + delta/2, x(:, k) + z2 * delta/2, f_u(k), f_v(k), f_r(k), p);
    z4 = f(t(k) + delta, x(:, k) + z3 * delta, f_u(k), f_v(k), f_r(k), p);
    
    x(:, k+1) = x(:, k) + delta * (z1 + z2 + z3 + z4) / 6;
end


u = x(1, :);
v = x(2, :);
r = x(3, :);

plot(r)     % plot the yaw rate
%% 
%% 
% 

function fx = f(t, x, f_u, f_v, f_r, p)

%     f_u = interp1(ts, f_u, t);
%     f_v = interp1(ts, f_v, t);
%     f_r = interp1(ts, f_r, t);
    fx= zeros(3, 1);
    
    fx(1) = x(2) * x(3) + (p.x_uuc * abs(x(1)) + p.x_uc) * x(1) + p.x_fc * f_u; 
    fx(2) = -x(1) * x(3) + (p.y_vvc * abs(x(2)) + p.y_vc) * x(2) + p.y_fc * f_v ;
    fx(3) = p.n_udc * fx(1) + (p.n_rrc * abs(x(3)) + p.n_rc) * x(3) + p.n_uuc * abs(x(1)) * x(1) + p.n_fc * f_r;

end