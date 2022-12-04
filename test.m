close all; clear; clc
%%

%s = [66, 75]
s = [81,79]
goal = [2, 2.5];
%goal = [5.6, 5.6];
MAX_X = 100;
MAX_Y = 100;

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


    goal = [floor(goal(1)*10), floor(goal(2)*10)]

    traj_2 = A_Star(s, goal, MAP, obs_MAP, rob_siz);
    
    traj_len = size(traj_2,1);
    
    for i=1:traj_len
        traj(i,:) = traj_2(traj_len+1-i,:);
    end

traj(1,:) = [55 65];
traj(2,:) = [55 60];
traj(3,:) = [55 55];
traj(4,:) = [55 50];
traj(5,:) = [55 45];
traj(6,:) = [55 40];
traj(7,:) = [55 35];
traj(8,:) = [50 30];
traj(9,:) = [45 25];
traj(10,:) = [40 25];
traj(11,:) = [35 25];
traj(12,:) = [30 25];

traj_len = size(traj,1);
    for i=1:traj_len
        xd_1(i) = traj(i,1)/10;
        yd_1(i) = traj(i,2)/10;
    end
    
    opt_traj = optimization(traj, obs_MAP, rob_siz);
    
    opt_len = size(opt_traj,1);
    for i=1:opt_len
        xd(i) = opt_traj(i,1)/10;
        yd(i) = opt_traj(i,2)/10;
    end

    

close all;

figure(1);
plot(xd_1*10,yd_1*10,'-b','Linewidth',1.5)
                grid on;
                hold on;

plot(xd*10,yd*10,'--r')
grid on;
hold on;
plot([0 38], [49 49], '-r');
plot([0 38], [51 51],'-r');
plot([62 100], [49 49], '-r');
plot([62 100], [51 51],'-r');
plot([38 38], [49 51],'-r');
plot([62 62], [49 51],'-r');

plot([38 38], [0 20], '-r');
plot([38 38], [30 70], '-r');
plot([38 38], [80 100], '-r');
plot([40 40], [0 20], '-r');
plot([40 40], [30 70], '-r');
plot([40 40], [80 100], '-r');

plot([38 40], [20 20], '-r');
plot([38 40], [30 30], '-r');
plot([38 40], [70 70], '-r');
plot([38 40], [80 80], '-r');

plot([60 60], [0 20], '-r');
plot([60 60], [30 70], '-r');
plot([60 60], [80 100], '-r');
plot([62 62], [0 20], '-r');
plot([62 62], [30 70], '-r');
plot([62 62], [80 100], '-r');

plot([60 62], [20 20], '-r');
plot([60 62], [30 30], '-r');
plot([60 62], [70 70], '-r');
plot([60 62], [80 80], '-r');

%{
    for i=1:100
        for j=1:100
            if obs_MAP(i,j) == -1
                plot(i, j, 's');
                grid on;
                hold on;
            end
        end
    end
%}
ylabel('y (m)','Interpreter','latex')
xlabel('x (m)','Interpreter','latex')
axis([0 100 0 100])
pbaspect([ 1 1 1 ]);