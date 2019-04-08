function sensors = DAQ_DATA_PARSER(X)
% This function parses the raw binary data from the OSU Data Acquistion
% devices. While the files have a .txt extension, they are NOT human
% readable, and require this parsing routine to extract the binary data.
% This routine expects a single array produced by SD_READER. The output is
% a structure with numerous data fields, described below. It should be
% noted that while the length of the vectors in the field are consistent,
% they are sampled at the sensor rates. I.E. the raw GPS fields are
% populated with data corresponding to the 4 Hz sampling rate, and the
% remaining filds are NAN. For convenience, an interpolated version of the
% GPS data is provided, but should be noted that the interpolated points
% are just that, interpolated, not measured.

% The basic performance specs for the device are as follows:
% INS and Sensor data rate - 128 Hz
% GPS data rate - 4 Hz
% Maximum angular rate -> +/- 1000 deg/s
% Maximum acceleration -> +/- 8 G
% Maximum magnetic field -> +/- 200 uT
% Maximum barometric pressure -> 110,000 Pa
% Minimum barometric pressure -> 30,000 Pa

% Sensors reported in body frame, X-nose, Y-right wing, Z-down
% GPS and INS reported in NED frame (North, East, Down)

% Data description: 
% time_s            -> Master time vector in seconds, spacing corresponds to 128Hz sample time
% GPS_valid         -> Index of valid GPS signals at the true 4 Hz spacing
% GPS_lat_deg       -> GPS latitude in degrees sampled at 4 Hz. To plot, use plot(sensors.GPS_lat_deg(sensors.GPS_valid)) or plot(sensors.GPS_lat_deg,'ro')
% GPS_lon_deg       -> GPS longitude, same caveats as GPS_lat_deg
% GPS_h_msl_m       -> GPS height in meters MSL, same caveats as GPS_lat_deg
% GPS_v_mps         -> GPS ground speed in m/s, same caveats as GPS_lat_deg
% GPS_head_deg      -> GPS heading REFERENCED FROM 0-360 deg (360=North), be careful! Same caveats as GPS_lat_deg
% GPS_satellites    -> GPS satellites visible, normally ~10-15, less in enclosed spaces. Minimum 4 for 3-D fix.
% GPS_fix_type      -> 1-time only, 2-2D, 3-3D, 4-3D+SBAS/WAAS
% GPS_north_m       -> Calculated true North displacement from turn-on position in m, same caveats as GPS_lat_deg
% GPS_east_m        -> Calculated true East displacement from turn-on position in m, same caveats as GPS_lat_deg
% GPS_v_ned_mps     -> [North East Down] components of velocity in the NED frame, meters/second, interpolated
% GPS_xy_acc_m      -> X-Y (i.e. North, East) position accuracy in m
% GPS_z_acc_m       -> Z (i.e. Down) position accuracy in m
% GPS_v_acc_mps     -> 3D velocity accuracy in m/s
% GPS_head_acc_deg  -> GPS derived heading accuracy in deg, HEADING NOT ACCURATE WHEN NOT MOVING!
% GPS_iTow          -> GPS Time-of-Week in ms
% GPS_year - GPS_ms -> GPS UTC time components (year,month,day,hour,minute,second,millisecond)
% GPS_time_UTC      -> GPS UTC time in Matlab friendly format, datetime
% GPS_**_interp     -> INTERPOLATED GPS data, upsampled to 128 Hz. Useful for plotting. Be careful, there be dragons in assuming the interpolated data has similar accuracy to the true 4 Hz data

%Sensor data
% gyro_rps          -> [x y z] Gyroscope data in the body frame, no bias removed, rad/s
% gyro_bias         -> [x y z] Gyroscope bias computed by onboard Kalman filter. May be corrupt if extreme maneuvers were performed (saturation limits)
% accel_mps2        -> [x y z] Acceleration data in the body frame, m/s^2
% mag_uT            -> [x y z] Magnetic data in the body frame, normal to see large biases, uT
% mag_bias          -> [x y z] Magnetic bias, perform 360 degree rotation to lock in
% p_pa              -> Temperature compensated barometric pressure in Pa
% p_ref_pa          -> Reference pressure (in Pa) to align GPS and barometric altitudes. AKA altimeter setting assuming standard atmosphere. 
% temp_C            -> Temperature in C, NOT GOOD IF ABOVE 60 C, battery damage can occur
% p_alt_m           -> Computed pressure altitude in m using reference pressure listed above, no good if reference pressure is off (bad GPS readings)
% quaternion        -> Attitude quaternion [R V[3]] 
% euler             -> [roll(x) pitch(y) yaw(z)] in rad. Right-handed coordinate system
% vel_INS_mps       -> Inertial velocity in NED frame, [North East Down] m/s. Bad INS alignment can quickly distort these values, needs robust GPS lock
% pos_INS_mps       -> INS position in NED frame, expressed as displacement in m [North East Down], again, bad alignment, bad data.

% event_index       -> Index of marker button presses, useful for finding specific maneuvers


%Constants
RE = 6371009; %Earth radius in m
d2m = 111131.745; %Meters per degree latitude
dt_data = 1/128; %Main sensor data reports at 128 Hz
dt_GPS = 1/4; % GPS reports at 4 Hz
clkfreq = 96000000; %Main system clock runs on 96 Mhz clock, more accurate time keeping


% SD reader and data parsing. Inserts into standard cells
I = X(X(:,1)==10,:);
G = X(X(:,1)==11,:);

%Calculate time step and phase align data to inertial estimates
T_I = TIME_STEP(I,clkfreq);
T_G = TIME_STEP(G,clkfreq)+ (G(1,2)-I(1,2))/clkfreq;

 % added by Ryan Powell 3/27
[~, indices] = unique(T_G); % get unique indices from T_G
indices_check = ones(1,length(indices)-1); % initialize check array
for i=1:length(indices)-1 % for size 1 less than number of unique indices
    if(indices(i)+1 ~= indices(i+1)) % if an index value is skipped
        indices_check(i) = 0; % then non unique element was recognized
    end
end
bad_indices = find(indices_check == 0) + 1; % get indices in T_G
T_G(bad_indices) = []; % remove duplicate indices in T_G
G(bad_indices,:) = []; % remove corresponding indices in G

%First data point can be erroneous
I = I(2:end,:);
G = G(2:end,:);

time = [0:dt_data:T_I(end)];

%% Scaling

I = interp1(T_I,I,time,'linear','extrap');
G1 = interp1(T_G,G(:,1:10),time,'linear','extrap');
G2 = interp1(T_G,G(:,11:13),time,'nearest','extrap');
G3 = interp1(T_G,G(:,14:18),time,'linear','extrap');
G4 = interp1(T_G,G(:,19:end),time,'nearest','extrap');
G = [G1 G2 G3 G4];

sensors = [];

% Time vector
sensors.time_s = time';             %Master time vector, in seconds
sensors.diff_TI = 1./diff(T_I);
sensors.diff_TG = 1./diff(T_G);

% GPS data 
kGPS = interp1(time,1:length(time),time(1):dt_GPS:time(end),'nearest');
sensors.GPS_valid = false(size(sensors.time_s));
sensors.GPS_valid(kGPS) = true;     %Boolean vector of valid GPS signals (set by dt_GPS)

% Raw GPS reported values
sensors.GPS_lat_deg     = G(:,8)/1e7;
sensors.GPS_lon_deg     = G(:,9)/1e7;
sensors.GPS_h_msl_m     = -G(:,10)/1000;  
sensors.GPS_v_mps       = (sum((G(:,15:17)/1000).^2,2)).^0.5;
sensors.GPS_head_deg    = abs(bitshift(int32(G(:,12)),-16))/100;
sensors.GPS_head_deg    = mod(sensors.GPS_head_deg+180,360)-180;
sensors.GPS_head_deg(sensors.GPS_head_deg<-180) = sensors.GPS_head_deg(sensors.GPS_head_deg<-180)+360;
sensors.GPS_satellites  = bitshift(int32(G(:,13)),-8);
sensors.GPS_fix_type    = bitand(int32(G(:,13)),7);

%Scaled GPS displacement values
sensors.GPS_north_m     = (sensors.GPS_lat_deg-sensors.GPS_lat_deg(10))*d2m;
sensors.GPS_east_m      = (sensors.GPS_lon_deg-sensors.GPS_lon_deg(10))*d2m.*cos(sensors.GPS_lat_deg*pi/180); %Scale to m based on initial latitude, will be increasingly incorrect for large(i.e. many km) displacements
sensors.GPS_v_ned_mps   = G(:,15:17)/1000;

%Reported GPS accuracy statistics
sensors.GPS_xy_acc_m    = double(bitand(int32(G(:,11)),65535))/1000;
sensors.GPS_z_acc_m     = double(bitshift(int32(G(:,11)),-16))/1000;
sensors.GPS_v_acc_mps   = G(:,18)/1000;
sensors.GPS_head_acc_deg= G(:,19)/1e5;

sensors.GPS_iTow        = G(:,14);
sensors.GPS_year        = bitand((G(:,20)),65535);
sensors.GPS_month       = bitand(bitshift((G(:,20)),-16),255);
sensors.GPS_day         = bitand(bitshift((G(:,20)),-24),255);

sensors.GPS_hour        = bitand(bitshift((G(:,21)),-0),255);
sensors.GPS_minute      = bitand(bitshift((G(:,21)),-8),255);
sensors.GPS_second      = bitand(bitshift((G(:,21)),-16),255);
sensors.GPS_ms          = mod(G(:,3),1000);
sensors.GPS_time_UTC    = datetime(sensors.GPS_year,sensors.GPS_month,sensors.GPS_day,sensors.GPS_hour,sensors.GPS_minute,sensors.GPS_second,sensors.GPS_ms);
    
% Make values NAN if valid not true
sensors.GPS_lat_deg_interp      = sensors.GPS_lat_deg;
sensors.GPS_lon_deg_interp      = sensors.GPS_lon_deg;
sensors.GPS_h_msl_m_interp      = sensors.GPS_h_msl_m;
sensors.GPS_north_m_interp      = sensors.GPS_north_m;
sensors.GPS_east_m_interp       = sensors.GPS_east_m;
sensors.GPS_lat_deg(~sensors.GPS_valid)     = NaN; %    |
sensors.GPS_lon_deg(~sensors.GPS_valid)     = NaN; %    | GPS values are set to NaN
sensors.GPS_h_msl_m(~sensors.GPS_valid)     = NaN; %    | if not valid
sensors.GPS_north_m(~sensors.GPS_valid)     = NaN; %    | 
sensors.GPS_east_m(~sensors.GPS_valid)      = NaN; %    |

% Gyros
sensors.gyro_rps        = I(:,11:13)*1000/32768*pi/180/9; %9 is for oversampling
sensors.gyro_bias_rps   = G(:,22:24)/1024/27*1000/32768*pi/180/9;
% Accelerometer
sensors.accel_mps2      = I(:,7:9)*9.807/4096/9;
% Magnetometer
sensors.mag_uT          = I(:,14:16);%*4800/16384;
sensors.mag_bias        = G(:,25:27);
% Barometer
sensors.p_pa            = I(:,18);
sensors.p_ref_pa        = mean(I(:,19));
sensors.temp_C          = I(:,17)/100;
sensors.p_alt_m         = -44330*(1-(sensors.p_pa./sensors.p_ref_pa).^(1/5.255)); %Standard atmosphere

%Fused inertial data
sensors.quaternion      = I(:,3:6)./2^30;
[sensors.euler_rad(:,1), sensors.euler_rad(:,2), sensors.euler_rad(:,3)]   = q2a(sensors.quaternion);
sensors.vel_INS_mps     = I(:,21:23)/1e6;
sensors.pos_INS_m       = I(:,24:26)/1e3;

[index, ~] = find(diff(I(:,27))>0.5);
sensors.event_index    = index+1;

