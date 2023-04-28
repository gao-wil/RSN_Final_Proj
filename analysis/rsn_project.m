clear all; close all;
bag = rosbag("lvl_2_hayden_map_12.bag");
bagd= select(bag, 'Topic', '/fusion');
msg = readMessages(bagd, 'DataFormat', 'struct');
msg_ptr = 1;
time_offset = double(msg{1}.Header.Stamp.Sec) + double(msg{1}.Header.Stamp.Nsec)*(10^-9);
while msg_ptr < length(msg)+1
    time(msg_ptr) = double(msg{msg_ptr}.Header.Stamp.Sec) + double(msg{msg_ptr}.Header.Stamp.Nsec)*(10^-9);
    time_norm(msg_ptr)=time(msg_ptr)-time_offset;
    yaw(msg_ptr) = msg{msg_ptr}.Yaw;
    pitch(msg_ptr)=msg{msg_ptr}.Pitch;
    roll(msg_ptr) =msg{msg_ptr}.Roll;
    accel_x(msg_ptr)  = msg{msg_ptr}.Imu.LinearAcceleration.X;
    accel_y(msg_ptr)  = msg{msg_ptr}.Imu.LinearAcceleration.Y;
    accel_z(msg_ptr)  = msg{msg_ptr}.Imu.LinearAcceleration.Z;
    sensor_x(msg_ptr) = msg{msg_ptr}.SensorY;
    sensor_y(msg_ptr) = msg{msg_ptr}.SensorX;
    range(msg_ptr) = double(msg{msg_ptr}.Range);
    msg_ptr=msg_ptr+1;
end
replace = find(range ==5000)
for n=1:length(replace)
    range(replace(n))=range(replace(n)-1);
end
replace = find(range ==300)
for n=1:length(replace)
    range(replace(n))=range(replace(n)-1);
end
diff_yaw = diff(unwrap(yaw));
yaw_replace = find(abs(diff_yaw)>5e-3)
for n=1:length(yaw_replace)
    accel_x(yaw_replace(n))=0;
    accel_y(yaw_replace(n))=0;
end
% replace = find(abs(accel_x) < 0.1);
% accel_x(replace) = 0;
% replace = find(abs(accel_y) < 0.1);
% accel_y(replace) = 0;
% accel_x = accel_x - mean(accel_x(1:300));
% accel_y = accel_y - mean(accel_y(1:300));
% accel_x=highpass(accel_x,0.005);
% accel_y=highpass(accel_y,0.005);
velocity_x=cumtrapz(time_norm,accel_x);
velocity_x=velocity_x +0.15;
velocity_y=cumtrapz(time_norm,accel_y);
velocity_z=cumtrapz(time_norm,accel_z);
x=cumtrapz(time_norm,velocity_x);
y=cumtrapz(time_norm,velocity_y);
z=cumtrapz(time_norm,velocity_z);
for n=1:length(x)
obstacle_x(n) = x(n) + range(n)/1000*cos(yaw(n));
obstacle_y(n) = y(n) + range(n)/1000*sin(yaw(n));
end
figure(1);
hold all;
pbaspect([1 1 1]);
scatter(x,-y);
scatter(obstacle_x, -obstacle_y);
axis([-16 16 -16 16]);
% velocity = cumtrapz(time_norm, accel_x);
% velocity_y=cumtrapz(time_norm, accel_y);
% for n=1: length(velocity)
%     velo_y(n) = velocity(n)*cos(wrapToPi(yaw(n)));
%     velo_x(n) = velocity(n)*sin(wrapToPi(yaw(n)));
%     y_velo_y(n)=velocity_y(n)*sin(wrapToPi(yaw(n)));
%     y_velo_x(n)=velocity_y(n)*cos(wrapToPi(yaw(n)));
% end
% X_y = cumtrapz(time_norm, velo_y);
% X_x = cumtrapz(time_norm, velo_x);
% Y_y = cumtrapz(time_norm, y_velo_y);
% Y_x = cumtrapz(time_norm, y_velo_x);
% figure(1)
% scatter(X_x + Y_x, X_y + Y_y);
% hold all;
% scatter(sensor_x, sensor_y);
% scatter(X_x, X_y);
%scatter(Y_x, Y_y);