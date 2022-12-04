function loc_opt_traj = local_path_update(Px, Py, xn, yn, theta, MAP, obs_MAP, rob_siz, obs_points, obs_count)
    obss = [];
    tmp_map = MAP;
    tmp_obs_map = obs_MAP;
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
        loc_opt_traj= [];
        return;
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
   