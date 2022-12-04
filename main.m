close all; clear; clc
addpath('api')
addpath('functions');
% simRemoteApi.start(19999): simple test
% simRemoteApi.start(19999,1300,false,true) : synchronous test
%%

on = 1;

MAX_X = 100;
MAX_Y = 100;

tf=20;
dt=0.05;
t=0.05:0.05:tf;

kp=0.4;
%kw=5.0;
kw = 3;

nearClippingPlane=1.00e-2;
farClippingPlane=6.0;

%{
xs = 36;
ys = 47;
start = [xs, ys];
%}

MAP=2*(ones(MAX_X,MAX_Y));
obs_MAP = zeros(MAX_X, MAX_Y);

    MAP(38:40,1:20) = -1;
    MAP(38:40,30:70) = -1;
    MAP(38:40,80:100) = -1;
    MAP(60:62,1:20) = -1;
    MAP(60:62,30:70)=-1;
    MAP(60:62,80:100)=-1;
    MAP(1:37,49:51)=-1;
    MAP(63:100,49:51)=-1;

obs_MAP(38:40,1:20) = -1;
obs_MAP(38:40,30:70) = -1;
obs_MAP(38:40,80:100) = -1;
obs_MAP(60:62,1:20) = -1;
obs_MAP(60:62,30:70)=-1;
obs_MAP(60:62,80:100)=-1;
obs_MAP(1:37,49:51)=-1;
obs_MAP(63:100,49:51)=-1;

rob_siz = 3;
xd_2 = [];
yd_2 = [];


%%
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    
    disp('Connected to remote API server');

    vrep.simxSynchronous(clientID,true);

    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    %Handle
    %pioneer
    [~,pioneer]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [~,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [~,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    
    % kinect 
    [~,kinect_depth] = vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_oneshot_wait);
    [~, kinect_rgb] = vrep.simxGetObjectHandle(clientID, 'kinect_rgb',vrep.simx_opmode_oneshot_wait);

    % The angles for kinect
    delta_z=57*pi/180;
    np_z=640;
    delta_x=43*pi/180;
    np_x=480;
    d_delta_z=-delta_z/2:delta_z/(np_z-1):delta_z/2;
    d_delta_x=-delta_x/2:delta_x/(np_x-1):delta_x/2;
%     Ry=[cos(pi),0,sin(pi);0,1,0;sin(-pi),0,cos(pi)];
    Ry=eye(3);
    
    %Command
    [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.0,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.0,vrep.simx_opmode_blocking);
    
    Puncte=zeros(3,1);
    Traj=[0,0,0];
    Puncte_pre=Puncte;
    
    obs_points=[];
    
    %Puncte=Ry*Puncte;
    
    
    goal = [0.0, 0.0];  xg = goal(1); yg = goal(2);
    
    while on == 1
        xd =[];
        yd = [];
        data = [];
        dat = 1;
        disp(' ')
        disp('[one grid cell size in floor is 1x1 m]')
        goal(1) = input('x coordinate(0~10m): ');
        goal(2) = input('y coordinate(0~10m): ');
        xg = goal(1); yg = goal(2);

        [~,posc]=(vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
        posc=double(posc);
        [~,angc]=(vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
        angc=double(angc);

        xs = floor((posc(1))*10);
        ys = floor((posc(2))*10);

        s = [xs, ys]
        goal = [floor(goal(1)*10), floor(goal(2)*10)]

        traj_2 = [];
        traj_2 = A_Star(s, goal, MAP, obs_MAP, rob_siz);
        path_len = size(traj_2,1);
        if path_len==0
            no_path=1;
            d_goal = 0;
            disp('no possible path')
        end
        no_path = 0;
        traj_len = size(traj_2,1);
        traj = [];

        for i=1:traj_len
            traj(i,:) = traj_2(traj_len+1-i,:);
        end

        xd_2 = [];
        yd_2 = [];
        for i=1:traj_len
            xd_2(i) = traj(i,1)/10;
            yd_2(i) = traj(i,2)/10;
        end

        %{
        for i=1:traj_len
            xd_2(i) = traj(i,1)/10;
            yd_2(i) = traj(i,2)/10;
        end
        %}
        opt_traj= [];
        opt_traj = optimization(traj, obs_MAP, rob_siz);

        opt_len = size(opt_traj,1);
        xd = [];
        yd = [];
        for i=1:opt_len
            xd(i) = opt_traj(i,1)/10;
            yd(i) = opt_traj(i,2)/10;
        end

        d=0.06; %wheels separation
        r_w=0.0275; % wheel radius

        i=1;
        data(:,i)=[posc(1); posc(2); 0;0; angc(3)];
        dist_g = distance(posc(1), posc(2), xg, yg);
        pos_x = posc(1);
        pos_y = posc(2);

        map_plot(xd, yd, xd_2, yd_2, data)

        theta=atan2(yd(i)-posc(2),xd(i)-posc(1));
        omegad=omega(theta,angc(3));
        i = i+1;
        d_goal = distance(posc(1), posc(2), xg, yg);
        
        %start moving
        while d_goal>0.2 && no_path == 0
            xn = xd(i);
            yn = yd(i);
            d_next = distance(xn, yn, posc(1), posc(2));
            disp('next target')
            disp([xn yn]);
            while d_next > 0.20
                [~,posc]=(vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_blocking  ));
                posc=double(posc);
                [~,angc]=(vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_blocking  ));
                angc=double(angc);
                theta = atan2(yn-posc(2), xn-posc(1));
                omegad = omega(theta, angc(3));
                if abs(omegad) > 30*pi/180
                    disp('turning');
                    while abs(omegad) > 3*pi/180
                        data(:,dat)=[posc(1); posc(2); 0;0; angc(3)];
                        dat = dat+1;
                        vd = 1;
                        [~,posc]=vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_streaming  );
                        [~,angle]=vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_streaming  );
                        posc=double(posc);
                        angc=double(angle);
                        theta=atan2(yn-posc(2),xn-posc(1));
                        omegad=omega(theta,angc(3));
                        v_r = vd*kw*omegad*d/dt;
                        v_l = vd*kw*(-omegad*d)/dt;
                        [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,v_l,vrep.simx_opmode_blocking);
                        [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,v_r,vrep.simx_opmode_blocking);
                        %vrep.simxSynchronousTrigger(clientID);
                        [~, res, depth] = vrep.simxGetVisionSensorDepthBuffer2(clientID,kinect_depth, vrep.simx_opmode_oneshot_wait); 

                        [~, res, img1] = vrep.simxGetVisionSensorImage2(0,kinect_rgb, 0, vrep.simx_opmode_oneshot_wait);
                        %ximg1 = double(img1)/256;
                        %figure(3)
                        %imshow(depth);
                        obs_count = 0;
                        avoid = 0;
                        Puncte=zeros(3,1);
                        for j=1:5:np_x%
                            for k=1:5:np_z%
                                y=nearClippingPlane + depth(j,k) * (farClippingPlane);
                                if y>=farClippingPlane
                                %if y>=1
                                else
                                    x=-y*tan(d_delta_x(1,j));
                                    %if x>-0.4 
                                    if x>-0.25
                                        z=-y*tan(d_delta_z(1,k));
                                        %{
                                        if z > -0.3 && z<0.3 && y>0.25 && y<1
                                            obs_count = obs_count+1;
                                            obs_points(obs_count,:) = [y;z];
                                            if y>0.25 && y<0.5
                                                avoid = 1;
                                            end
                                        end
                                        %}
                                        %punct=[y;z;x];
                                        punct=[y;z;1];
                                        Puncte=[Puncte,punct];
                                    end
                                end
                            end
                        end

                        Puncte=Ry*Puncte;
                        PP=pointCloud(Puncte');
                        %if i>3
                        %tform = pcregistericp(Puncte_pre,PP,'Extrapolate',true);
                        %Traj=[Traj;Traj(end,:)+tform.T(4,1:3)];
                        %end
                        
                        figure(1)
                        %{
                        plot3(Puncte(1,1:1:end),Puncte(2,1:1:end),Puncte(3,1:1:end),'.','Color','b')
                        xlabel('x'); ylabel('y'); zlabel('z');
                        axis([0 farClippingPlane -farClippingPlane/2 farClippingPlane/2 -0.5 1])
                        view(0,90);
                        %}
                        
                        
                        plot(Puncte(1,:), Puncte(2,:), '.', 'Color', 'b')
                        xlabel('x'); ylabel('y');
                        
                        grid on;
                        %daspect([1 1 1]);
                        Puncte_pre=PP;
                        Puncte=[];
                        axis([0 5 -3 3]);

                        vrep.simxSynchronousTrigger(clientID);
                    end
                end
                %disp('moving');
                [~, res, depth] = vrep.simxGetVisionSensorDepthBuffer2(clientID,kinect_depth, vrep.simx_opmode_oneshot_wait);

                [~, res, img1] = vrep.simxGetVisionSensorImage2(0,kinect_rgb, 0, vrep.simx_opmode_oneshot_wait);
                ximg1 = double(img1)/256;
                %figure(3)
                %imshow(depth);
                obs_count = 0;
                avoid = 0;
                Puncte=zeros(3,1);
                for j=1:5:np_x%
                    for k=1:5:np_z%
                        y=nearClippingPlane + depth(j,k) * (farClippingPlane);
                        if y>=farClippingPlane
                        %if y>=1
                        else
                            x=-y*tan(d_delta_x(1,j));
                            %if x>-0.4
                            if x>-0.25
                                z=-y*tan(d_delta_z(1,k));
                                if z > -0.25 && z<0.28 && y>0.28 && y<1
                                    obs_count = obs_count+1;
                                    obs_points(obs_count,:) = [y;z];
                                    if y>0.25 && y<0.5
                                        avoid = 1;
                                    end
                                end
                                %punct=[y;z;x];
                                punct=[y;z;1];
                                Puncte=[Puncte,punct];
                            end
                        end
                    end
                end
                
                figure(1)
                
                %{
                plot3(Puncte(1,1:1:end),Puncte(2,1:1:end),Puncte(3,1:1:end),'.','Color','b')
                xlabel('x'); ylabel('y'); zlabel('z');
                axis([0 farClippingPlane -farClippingPlane/2 farClippingPlane/2 -0.5 1])
                view(0,90);
                %}
                
                plot(Puncte(1,:), Puncte(2,:), '.', 'Color', 'b')
                xlabel('x'); ylabel('y');
                grid on;
                axis([0 5 -3 3]);
                
                %daspect([1 1 1]);
                Puncte=[];
                
                if avoid ==1
                    disp('obs detect');
                    obss = [];
                    tmp_map = MAP;
                    tmp_obs_map = obs_MAP;
                    [~,posc]=(vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
                    posc=double(posc);
                    Px = posc(1);
                    Py = posc(2);
                    [~,angc]=(vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
                    angc=double(angc);
                    theta = angc(3);
                    ROT(1,1) = cos(theta);
                    ROT(1,2) = -1*sin(theta);
                    ROT(2,1) = sin(theta);
                    ROT(2,2) = cos(theta);
                    rotd = ROT*obs_points';
                    iner = rotd + [Px;Py];
                    for obs_i = 1:obs_count
                        tmp_map(floor(iner(1,obs_i)*10),floor(iner(2,obs_i)*10)) = -1;
                        tmp_obs_map(floor(iner(1,obs_i)*10),floor(iner(2,obs_i)*10)) = -1;
                        obs = [floor(iner(1,obs_i)*10), floor(iner(2,obs_i)*10)];
                        obss = [obss;obs];
                    end
                    
                    figure(4)
                    for map_x = 1:100
                        for map_y = 1:100
                            if tmp_map(map_x, map_y) == -1
                                plot(map_x, map_y,'s');
                                hold on;
                            end
                        end
                    end
                    axis([0 100 0 100]);
                    pbaspect([1 1 1]);
                    hold on;
                    grid on;
                    
                    
                    loc_traj_2 = [];
                    loc_s = [floor(Px*10), floor(Py*10)];
                    loc_goal = [floor(xn*10), floor(yn*10)];
                    loc_traj_2 = A_Star(loc_s, loc_goal, tmp_map, tmp_obs_map, rob_siz);
                    path_len = size(loc_traj_2,1);
                    
                    no_path = 0;
                    if path_len==0
                        no_path=1;
                        d_goal = 0;
                        disp('no possible path')
                        break;
                    end
                    loc_traj_len = size(loc_traj_2,1);
                    loc_traj = [];
                    for loc_i=1:loc_traj_len
                        loc_traj(loc_i,:) = loc_traj_2(loc_traj_len+1-loc_i,:);
                    end
                    %{
                    for loc_i=1:loc_traj_len
                        loc_xd_2(loc_i) = loc_traj(loc_i,1)/10;
                        loc_yd_2(loc_i) = loc_traj(loc_i,2)/10;
                    end
                    %}
                    loc_opt_traj= [];
                    loc_opt_traj = optimization(loc_traj, tmp_obs_map, rob_siz);
                    %loc_opt_traj = loc_traj;

                    loc_opt_len = size(loc_opt_traj,1);
                    loc_xd = [];
                    loc_yd = [];
                    for loc_i=1:loc_opt_len
                        loc_xd(loc_i) = loc_opt_traj(loc_i,1)/10;
                        loc_yd(loc_i) = loc_opt_traj(loc_i,2)/10;
                    end
                    figure(4)
                    hold on;
                    plot(loc_opt_traj(:,1), loc_opt_traj(:,2),'-r');
                    hold off;
                    tmp_traj_x = xd(i+1:end);
                    tmp_traj_y = yd(i+1:end);
                    for update = 2:loc_opt_len
                        xd(i+update-2) = loc_xd(update);
                        yd(i+update-2) = loc_yd(update);
                    end
                    tmp_traj_len = size(tmp_traj_x,2);

                    for update = 1:tmp_traj_len
                        xd(i+update-2+loc_opt_len) = tmp_traj_x(update);
                        yd(i+update-2+loc_opt_len) = tmp_traj_y(update);
                    end
                    xn = xd(i);
                    yn = yd(i);
                    opt_len = size(xd,2);

                    figure(1)
                    %{
                    plot3(Puncte(1,1:1:end),Puncte(2,1:1:end),Puncte(3,1:1:end),'.','Color','b')
                    xlabel('x'); ylabel('y'); zlabel('z');
                    axis([0 farClippingPlane -farClippingPlane/2 farClippingPlane/2 -0.5 1])
                    view(0,90);
                    %}
                    %{
                    plot(Puncte(1,:), Puncte(2,:), '.', 'Color', 'b')
                    xlabel('x'); ylabel('y');
                    grid on;
                    axis([0 5 -3 3]);
                    %daspect([1 1 1]);
                    %}
                    Puncte=[];

                    obs_points=[];

                    map_plot(xd, yd, xd_2, yd_2, data);
                end
                data(:,dat)=[posc(1); posc(2); 0;0; angc(3)];
                dat = dat+1;
                vd = 0.15;
                [~,posc]=(vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
                posc=double(posc);
                [~,angc]=(vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_blocking  )); % -1 for absolute position
                angc=double(angc);
                theta=atan2(yn-posc(2),xn-posc(1));
                omegad = omega(theta, angc(3));
                v_r=(vd+kw*d*omegad)/r_w;
                v_l=(vd-kw*d*omegad)/r_w;
                [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,v_l,vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,v_r,vrep.simx_opmode_blocking);

                
                
                %if i>3
                %tform = pcregistericp(Puncte_pre,PP,'Extrapolate',true);
                %Traj=[Traj;Traj(end,:)+tform.T(4,1:3)];
                %end


                vrep.simxSynchronousTrigger(clientID);
                d_next = distance(xn, yn, posc(1), posc(2));
            end
            if i < opt_len
                i = i+1;
            end
            d_goal = distance(posc(1), posc(2), xg, yg);
        end
        if no_path == 0
            map_plot(xd, yd, xd_2, yd_2, data)
        end
        on = input('keep going? ( y: 1 ) \n');
        %close all;
    end
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
end 
vrep.delete();
close all;

%%
if no_path == 0
    map_plot(xd, yd, xd_2, yd_2, data)
end
%map_plot(loc_traj(1,:),loc_traj(2,:), xd_2, yd_2, data)
%%
function map_plot(xd, yd, xd_2, yd_2, data_2)
len = size(data_2',1);
if len>1
    for i=1:len-1
        if data_2(1,i)<0.01
           data(1,i) = data_2(1,i+1);
        else
           data(1,i) = data_2(1,i);
        end
        if data_2(2,i)<0.01
           data(2,i) = data_2(2,i+1);
        else
           data(2,i) = data_2(2,i);
        end
    end
end
figure(2);
p1 = plot(xd*10,yd*10,'-b','DisplayName','opt path');
hold on;
if size(xd_2,1)>0
p2 = plot(xd_2*10,yd_2*10,'-m','DisplayName','A* path');
end
if len>1
    p3 = plot(data(1,:)*10,data(2,:)*10,'--b','Linewidth',1.5,'DisplayName','rob traj');
end
%plot(xd_2*10, yd_2*10, '--b','Linewidth',1.5)
grid on;
hold on;
p = plot([0 38], [49 49], '-r');
p = plot([0 38], [51 51],'-r');
p = plot([62 100], [49 49], '-r');
p = plot([62 100], [51 51],'-r');
p = plot([38 38], [49 51],'-r');
p = plot([62 62], [49 51],'-r');

p = plot([38 38], [0 20], '-r');
p = plot([38 38], [30 70], '-r');
p = plot([38 38], [80 100], '-r');
p = plot([40 40], [0 20], '-r');
p = plot([40 40], [30 70], '-r');
p = plot([40 40], [80 100], '-r');

p = plot([38 40], [20 20], '-r');
p = plot([38 40], [30 30], '-r');
p = plot([38 40], [70 70], '-r');
p = plot([38 40], [80 80], '-r');

p = plot([60 60], [0 20], '-r');
p = plot([60 60], [30 70], '-r');
p = plot([60 60], [80 100], '-r');
p = plot([62 62], [0 20], '-r');
p = plot([62 62], [30 70], '-r');
p = plot([62 62], [80 100], '-r');

p = plot([60 62], [20 20], '-r');
p = plot([60 62], [30 30], '-r');
p = plot([60 62], [70 70], '-r');
p = plot([60 62], [80 80], '-r');
ylabel('y (m)','Interpreter','latex')
xlabel('x (m)','Interpreter','latex')
axis([0 100 0 100])
pbaspect([ 1 1 1 ]);

if len>1
    legend([p1 p2 p3]);
else
    legend([p1 p2]);
end

hold off;

end
%%
%{
%제자리 회전 커맨드
if abs(omegad)>(60*pi/180) %0.5236 rad = 30 deg
    while abs(omegad)>(10*pi/180) %0.1745 = 10 deg
        vd = 0.6;
        [~,posc]=vrep.simxGetObjectPosition(clientID,pioneer,-1,vrep.simx_opmode_streaming  );
        [~,angle]=vrep.simxGetObjectOrientation(clientID,pioneer,-1,vrep.simx_opmode_streaming  ); % -1 for absolute position
        posc=double(posc);
        angc=double(angle);
                
        theta=atan2(yd(i)-posc(2),xd(i)-posc(1));
        omegad=kw*omega(theta,angc(3));
        v_r = vd*omegad*d/dt;
        v_l = vd*(-omegad*d)/dt;
        [~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,v_l,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,v_r,vrep.simx_opmode_blocking);
        vrep.simxSynchronousTrigger(clientID);
    end
end

%전진 커맨드
vd = 0.16;
theta=atan2(yd(i)-posc(2),xd(i)-posc(1));
omegad = kw*omega(theta, angc(3));
v_r=(vd+d*omegad)/r_w;
v_l=(vd-d*omegad)/r_w;
[~]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,v_l,vrep.simx_opmode_blocking);
[~]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,v_r,vrep.simx_opmode_blocking);
vrep.simxSynchronousTrigger(clientID);


%}