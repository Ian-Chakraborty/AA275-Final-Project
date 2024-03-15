close all
clear
clc
%%
[time_c, goal_xyz_NED, psi_rel_deg, vel_input, psidot_des] = load_con("./late13/con_11_21-57-51.txt");
[time_v, vel_true] = load_vel("./late13/vel_11_21-57-51.txt");
[time_t, tgt_D] = load_tgt("./late13/tgt_11_21-57-51.txt");
[time_a, att_true] = load_att("./late13/att_11_21-57-51.txt");
[time_p, pos_true] = load_loc("./late13/loc_11_21-57-51.txt");
time_t = time_t + 17.155404;
time_c = time_c + 5;


figure()
plot(time_c,vel_input(:,1),'linewidth',1.5)
xlabel('Time (s)')
ylabel('X Velocity (m/s)')
hold on
plot(time_v, vel_true(:,1),'linewidth',1.5)
legend('Input', 'True')

figure()
plot(time_c,vel_input(:,2),'linewidth',1.5)
xlabel('Time (s)')
ylabel('Y Velocity (m/s)')
hold on
plot(time_v, vel_true(:,2),'linewidth',1.5)
legend('Input', 'True')

figure()
plot(time_c,vel_input(:,3),'linewidth',1.5)
xlabel('Time (s)')
ylabel('Z Velocity  (m/s)')
hold on
plot(time_v, vel_true(:,3),'linewidth',1.5)
legend('Input', 'True')

figure()
plot(time_c,2*psi_rel_deg,'LineWidth',1.5)
hold on
plot(time_a, att_true(:,2)*180/pi,'linewidth',1.5)
legend('|Relative Angle Input|','True Attitude')
title('Yaw Input')


figure()
plot(time_a,att_true,'linewidth',1.5)
xlabel('Time (s)')
ylabel('Att (rad)')
legend('Pitch','Yaw','Roll')


figure()
subplot(4,1,1)
plot(time_c,vel_input(:,1),'linewidth',1.5)
xlabel('Time (s)')
ylabel('X Velocity (m/s)')
hold on
plot(time_v, vel_true(:,1),'linewidth',1.5)
legend('Input', 'True')

subplot(4,1,2)
plot(time_c,vel_input(:,2),'linewidth',1.5)
xlabel('Time (s)')
ylabel('Y Velocity (m/s)')
hold on
plot(time_v, vel_true(:,2),'linewidth',1.5)
legend('Input', 'True')

subplot(4,1,3)
plot(time_c,vel_input(:,3),'linewidth',1.5)
xlabel('Time (s)')
ylabel('Z Velocity (m/s)')
hold on
plot(time_v, vel_true(:,3),'linewidth',1.5)
legend('Input', 'True')


subplot(4,1,4)
plot(time_c,2*psi_rel_deg,'linewidth',1.5)
xlabel('Time (s)')
ylabel('Angle (deg)')
hold on
plot(time_a, att_true(:,2)*180/pi + 53,'linewidth',1.5)
legend('|Relative Angle Input|','True Attitude')
sgtitle('Inputs vs True Values')



vcomp = vel_true(49:98,:);
vdiff = vcomp - vel_input;

sq_err = vdiff.^2;
msqerr = mean(sq_err,1)
rmse = sqrt(msqerr)



%%
function [time, tgt_xyz_D, psi_rel_deg, vel_input, psidot_des] = load_con(file)
    data_struct_import = importdata(file);
    
    N = length(data_struct_import.textdata);
    M = 9;
    data = nan*ones(N,M);
    infmt = "hh:mm:ss.SSSS";
    for i=1:N
        line = split(data_struct_import.textdata{i},',')';
        time = line(1);
        x = data_struct_import.textdata{i,3};
        y = data_struct_import.textdata{i,4};
        z = data_struct_import.textdata{i,5};
        psi = data_struct_import.textdata{i,6};
        xdot = data_struct_import.textdata{i,7};
        ydot = data_struct_import.textdata{i,8};
        zdot = data_struct_import.textdata{i,9};
        psi_dot = 0;
        x = str2double(strip(strip(strip(x),'['),']'));
        y = str2double(strip(strip(strip(y),'['),']'));
        z = str2double(strip(strip(strip(z),'['),']'));
        xdot = str2double(strip(strip(strip(xdot),'['),']'));
        ydot = str2double(strip(strip(strip(ydot),'['),']'));
        zdot = str2double(strip(strip(strip(zdot),'['),']'));
        psi = str2double(strip(strip(strip(psi),'['),']'));
        % psi_dot = str2double(strip(strip(strip(psi_dot),'['),']'));    
        time = milliseconds(duration(strip(time),"InputFormat",infmt));
        data(i,:) = [time, x, y, z, psi, xdot, ydot, zdot, psi_dot ];
    end
    % data(struct) = [data_struct{:}];

    tgt_xyz_D = data(:,2:4);
    psi_rel_deg = data(:,5);
    vel_input = data(:,6:8);
    psidot_des = data(:,9);
    time = data(:,1);
    time = 1/1000 * (time - time(1));
end

function [time, vel_true] = load_vel(file)
    data_struct_import = importdata(file);
    
    N = length(data_struct_import.textdata);
    M = 4;
    infmt = "hh:mm:ss.SSSS";
    for i=1:N
        line = split(data_struct_import.textdata{i},',')';
        time(i,1) = milliseconds(duration(strip(line{:}),"InputFormat",infmt));

    end
    % data(struct) = [data_struct{:}];
    time = 1/1000 * (time - time(1));
    vel_true = data_struct_import.data(:,:);
end

function [time_p, pos_true] = load_loc(file)
    data_struct_import = importdata(file);
    
    N = length(data_struct_import);
    infmt = "hh:mm:ss.SSSS";
    for i=1:N
        line = split(data_struct_import{i},',')';
        time(i,1) = milliseconds(duration(strip(line{1}),"InputFormat",infmt));
        xi = split(line{3},'=');
        xi = str2double(xi{2});
        x(i,1) = xi;
    
        yi = split(line{4},'=');
        yi = str2double(yi{2});
        y(i,1) = yi;
    
        zi = split(line{5},'=');
        zi = str2double(zi{2});
        z(i,1) = zi;
    
    end
    time = 1/1000 * (time - time(1));
    time_p = time;
    
    pos_true = [x,y,z];
end

function [time_a, att_true] = load_att(file)
    data_struct_import = importdata(file);

    N = length(data_struct_import);

    infmt = "hh:mm:ss.SSSS";
    for i=1:N
        line = split(data_struct_import{i},',')';
        time_a(i,1) = milliseconds(duration(strip(line{1}),"InputFormat",infmt));
        xi = split(line{3},'=');
        xi = str2double(xi{2});
        x(i,1) = xi;
    
        yi = split(line{4},'=');
        yi = str2double(yi{2});
        y(i,1) = yi;
    
        zi = split(line{5},'=');
        zi = str2double(zi{2});
        z(i,1) = zi;
    
    end
    time_a = 1/1000 * (time_a - time_a(1));
    att_true = [x,y,z];


end


function R = R_body_to_NED(pyr)
p = pyr(:,1);
y = pyr(:,2);
r = pyr(:,3);
theta = p;
psi = y;
phi = r;
cth = cos(theta);
sth = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);

cphi = cos(phi);
sphi = sin(phi);

R = [cth*cpsi, -cphi*spsi + sphi*sth*cpsi, sphi*spsi + cphi*sth*cpsi; ...
    cth*spsi, cphi*cpsi + sphi*sth*spsi, -sphi*cpsi + cphi*sth*spsi; ...
    -sth, sphi*cth, cphi*cth];



end

function [time, tgt_D] = load_tgt(file)
    data_struct_import = importdata(file);
    
    N = length(data_struct_import);
    infmt = "hh:mm:ss.SSSS";
    for i=1:N
        line = split(data_struct_import{i},',')';
        timei = line(1);
        time(i,1) = milliseconds(duration(strip(timei{:}),"InputFormat",infmt));
        xi = line(3);
        yi = line(4);
        zi = line(5);
        x(i,1) = str2double(strip(strip(strip(xi{:}),'['),']'));
        y(i,1) = str2double(strip(strip(strip(yi{:}),'['),']'));
        z(i,1) = str2double(strip(strip(strip(zi{:}),'['),']'));
    end
    % data(struct) = [data_struct{:}];
    time = 1/1000 * (time - time(1));
    tgt_D = [x,y,z];
end