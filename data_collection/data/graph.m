% Triboresistive Project (2024)

%% Load File
filename = 'Log_Sensor_T_.txt'; % Specify the file name
sensorT = readmatrix(filename);   % Read the file into a matrix

filename = 'Log_Robot_Pos_.txt'; % Specify the file name
RobotPos = readmatrix(filename);   % Read the file into a matrix

%% Plot Sensor
time = sensorT(:,1);
sensor1 = sensorT(:,2);
sensor2 = sensorT(:,3);
sensor3 = sensorT(:,4);
sensor4 = sensorT(:,5);

tiledlayout(4,1)
nexttile
plot(time, sensor1);
nexttile
plot(time, sensor2);
nexttile
plot(time, sensor3);
nexttile
plot(time, sensor4);

%% UR robot
pos_x = RobotPos(:,2);
pos_y = RobotPos(:,3);

plot(pos_x, pos_y)
xlim([-0.095-0.01 -0.095+0.04])
ylim([0.575-0.01 0.575+0.04])