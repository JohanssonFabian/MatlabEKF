%% Section 1: Data Collection
% Read data from ESP32 via Bluetooth and save for next sections
% Test runs included on GitHub

% Initiate serial connection
clear; clc;
bt = serialport("COM11", 115200);
flush(bt);
rawData = [];

% Read data form BT
while true
    line = readline(bt);
    vals = str2double(split(line, ','));
    if numel(vals) == 6
        ax = vals(1) * 9.82;   % acceleration x (g -> m/s^2)
        ay = vals(2) * 9.82;   % acceleration y (g -> m/s^2)
        gz = vals(3);          % yaw rate (deg/s)
        xm = vals(4) / 1000;   % measured x position (mm -> m)
        ym = vals(5) / 1000;   % measured y position (mm -> m)
        dt = vals(6);          % time step dt (s)
        rawData(end+1, :) = [ax, ay, gz, xm, ym, dt];
    end
end

save('Data_run_x', 'rawData');

%% Section 2: Extended Kalman Filter (EKF)

% Extract data
clear;clc;
load('SquareRunTurns.mat') % Load .mat files with data from previous section
ax_all = rawData(:,1);
ay_all = rawData(:,2);
gz_all = rawData(:,3) * pi / 180; % Oops gz was in deg/s from Esp23, change to rad/s
xm_all = rawData(:,4);
ym_all = rawData(:,5);
dt_all = rawData(:,6);
N = size(rawData,1);

% Preallocate result vectors
x_est   = zeros(N,1);
y_est   = zeros(N,1);
vx_est  = zeros(N,1);
vy_est  = zeros(N,1);
yaw_est = zeros(N,1);

% Kalman parameters
H = [1 0 0 0 0;
     0 1 0 0 0];
R = R_meas * eye(2);  % measurement noise covariance

Q_base = 0.1;          % Base process noise (0.1)
Q_yaw  = 0.01;         % Yaw process noise (0.01)
R_meas = 0.001;        % Measurement noise (0.001)

% Initial state [x; y; vx; vy; yaw]
x = zeros(5,1);
P = eye(5);

for k = 1:N
    % Fetch inputs and previous yaw
    ax = ax_all(k);
    ay = ay_all(k);
    gz = gz_all(k);
    dt = dt_all(k);
    theta = x(5);

    % --- Prediction Step ---
    % Rotation matrix from body to world coordinates
    Rbw = [cos(theta), -sin(theta);
           sin(theta),  cos(theta)];
    acc_world = Rbw * [ax; ay];

    % Nonlinear state prediction
    x_pred = x;
    x_pred(1) = x(1) + x(3)*dt + 0.5*acc_world(1)*dt^2;
    x_pred(2) = x(2) + x(4)*dt + 0.5*acc_world(2)*dt^2;
    x_pred(3) = x(3) + acc_world(1)*dt;
    x_pred(4) = x(4) + acc_world(2)*dt;
    x_pred(5) = x(5) + gz * dt;

    % Jacobian F = df/dx
    F = [1, 0, dt, 0, 0;
         0, 1, 0, dt, 0;
         0, 0, 1,  0, (-sin(theta)*ax - cos(theta)*ay)*dt;
         0, 0, 0,  1, ( cos(theta)*ax - sin(theta)*ay)*dt;
         0, 0, 0, 0, 1];

    % Define Q 
    Q = Q_base * [0.25*dt^4,    0, 0.5*dt^3,    0, 0;
                  0,    0.25*dt^4,    0, 0.5*dt^3, 0;
                  0.5*dt^3,    0, dt^2,    0, 0;
                  0, 0.5*dt^3,    0, dt^2, 0;
                  0, 0, 0, 0, Q_yaw*dt^2];

    % Predict covariance
    P = F * P * F' + Q;
    x = x_pred;

    % --- Update Step ---
    z = [xm_all(k); ym_all(k)];
    y_tilde = z - H * x;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y_tilde;
    P = (eye(5) - K * H) * P;

    % Store estimates
    x_est(k)   = x(1);
    y_est(k)   = x(2);
    vx_est(k)  = x(3);
    vy_est(k)  = x(4);
    yaw_est(k) = x(5);
end

%% Section 3: Plotting
figure;
scatter(xm_all-1, ym_all-1, 15, "filled",  'Color', [0.2 0.4 0.8]); hold on;
plot(x_est-1, y_est-1, 'Color', [0.9 0.4 0.1], 'LineWidth', 1.5); hold on;
axis equal
xlabel('x (m)')
ylabel('y (m)')
legend('Measured Position', 'Kalman Filter')
grid on;

% Yaw Plot
figure;
plot((1:N)*mean(dt_all), rad2deg(yaw_est), 'Color', [0.2 0.6 0.3], 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Yaw (deg)');
grid on;