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


%% Section 2: Kalman Filter

% Extract data
clear;clc;
load('SquareRunTurns.mat'); % Load .mat files with data from previous section
ax_all = rawData(:,1);
ay_all = rawData(:,2);
xm_all = rawData(:,4);
ym_all = rawData(:,5);
dt_all = rawData(:,6);
N = size(rawData,1);

% Result vectors
x_est = zeros(N,1);
y_est = zeros(N,1);
vx_est = zeros(N,1);
vy_est = zeros(N,1);

% Kalman parameters
F = @(dt)[1 0 dt 0;
          0 1 0 dt;
          0 0 1  0;
          0 0 0  1];

B = @(dt)[0.5*dt^2 0;
          0 0.5*dt^2;
          dt 0;
          0 dt];

H = [1 0 0 0;
     0 1 0 0];

Q_base = 0.1;
R = 0.001 * eye(2);

% Initial state [x; y; vx; vy]
x = zeros(4,1);
P = eye(4);

% For-loop för Kalman
for k = 1:N

    % Update models with dt
    dt = dt_all(k);
    Fk = F(dt);
    Bk = B(dt);

    % Define Q
    Q = Q_base * [0.25*dt^4,    0, 0.5*dt^3,    0;
                  0,    0.25*dt^4,    0, 0.5*dt^3;
                  0.5*dt^3,    0, dt^2,    0;
                  0, 0.5*dt^3,    0, dt^2];

    % --- Prediction Step ---
    u = [ax_all(k); ay_all(k)];
    x = Fk * x + Bk * u;
    P = Fk * P * Fk' + Q;

    % --- Update Step ---
    z = [xm_all(k); ym_all(k)];
    y_tilde = z - H * x;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y_tilde;
    P = (eye(4) - K * H) * P;

    % Store estimates
    x_est(k) = x(1);
    y_est(k) = x(2);
    vx_est(k) = x(3);
    vy_est(k) = x(4);
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