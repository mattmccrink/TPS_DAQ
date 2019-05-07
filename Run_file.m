%% DAQ Read example
evalin('base','clc,clear all, close all')
       
file1 = 'H:\R_003.txt'; %Set absolute path i.e. drive:\folder\R_***.txt, or you can use local paths if you know what you're doing!

X = SD_READER(file1,32);
sensors = DAQ_DATA_PARSER(X);

sensors.event_index

%% Plotting examples

%Plot raw gyroscope values
figure();
plot(sensors.time_s,sensors.gyro_rps)
xlabel('Time (s)')
ylabel('Gyro (rad/s)')
legend('X','Y','Z')

%Plot GPS data two ways:
%First way: Extract valid GPS data points corresponding to 4 Hz sample
%rate
figure();
plot(sensors.time_s(sensors.GPS_valid),sensors.GPS_lat_deg(sensors.GPS_valid),'ro');
xlabel('Time (s)')
ylabel('Latitude (deg)')
hold on

%Second way: Cheat! Plot interpolated GPS data at same rate as sensor data
%(128 Hz). Zoom in and observe how data is interpolated. The red dots are
%actual measurements, the blue asterisks are only "true" when they overlap
%the red dots. 
plot(sensors.time_s,sensors.GPS_lat_deg_interp,'b*')


%Plot using Matlab datetime field
figure()
plot(sensors.GPS_time_UTC,sensors.p_pa)
xlabel('Time (UTC)')
ylabel('Baro pressure (Pa)')


%Plot displacement course over ground (East - X axis, North - y axis)
figure()
plot(sensors.GPS_east_m_interp,sensors.GPS_north_m_interp);
xlabel('East Displacement (m)')
ylabel('North Displacement (m)')
axis equal


%Plot 3D displacement course over ground (East - X axis, North - y axis,
%Down - z axis), flip z-sign to align with the way most people think of
%altitude...
figure()
stem3(sensors.GPS_east_m_interp,sensors.GPS_north_m_interp,-sensors.GPS_h_msl_m_interp);
xlabel('East Displacement (m)')
ylabel('North Displacement (m)')
zlabel('Height MSL (m)')
axis equal

%The last one was probably too dense, try again by plotting a smaller
%sample of your data (chosen arbitrarily as 300)
offset = floor(length(sensors.GPS_east_m_interp)/300);
figure()
stem3(sensors.GPS_east_m_interp(1:offset:end),...
    sensors.GPS_north_m_interp(1:offset:end),...
    -sensors.GPS_h_msl_m_interp(1:offset:end));
xlabel('East Displacement (m)')
ylabel('North Displacement (m)')
zlabel('Height MSL (m)')
axis equal

%Plot data for events happening between mark button presses (event 1 and
%2). This only works if marker_index is not empty!
figure();
Event_start = 1;
Event_end = 2;
index = sensors.event_index(Event_start):sensors.event_index(Event_end);
plot(sensors.time_s(index),sensors.mag_uT(index,:))
xlabel('Time (s)')
ylabel('Magnetic field (uT)')
legend('X','Y','Z')
