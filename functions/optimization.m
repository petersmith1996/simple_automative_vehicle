function opt_traj = optimization(traj, map, rob_siz)
    traj_len = size(traj, 1);
    opt_path(1,:) = traj(1,:);
    opt_node = 2;
    pivot = 1;
    %pivot_pre=0;
    while pivot ~= traj_len
    %while pivot ~= pivot_pre
        pivot_set=0;
        for i=traj_len:-1:pivot
            x1 = traj(pivot,1);
            y1 = traj(pivot,2);
            x2 = traj(i,1);
            y2 = traj(i,2);
            %{
            if i==traj_len && pivot == 1
                disp('n');
                disp(n);
            end
            %}
            if abs(x2-x1) >= abs(y2-y1)
                n = abs(x1-x2)+1;
                xd = linspace(x1, x2, n);
                m = (y2-y1)/(x2-x1);
                if x2 ~= x1
                    yd = floor(m*(xd-x1) + y1);
                else
                    yd = traj(pivot,2);
                    yd_2 = yd;
                end
            else
                n = abs(y1-y2)+1;
                yd = linspace(y1, y2, n);
                m = (x2-x1)/(y2-y1);
                if y2 ~= y1
                    xd = floor(m*(yd-y1) + x1);
                else
                    xd = traj(pivot,1);
                    xd_2 = xd;
                end
            end
            %{
            if i==traj_len && pivot == 1
                disp('x');
                disp(xd);
                disp('y');
                disp(yd);
            end
            %}
            flag = 0;
            for j=n:-1:1
                if n==1
                    if abs(x2-x1) >= abs(y2-y1)
                        yd(j) = yd_2;
                    else
                        xd(j) = xd_2;
                    end
                end
                if collcheck(map, xd(j), yd(j), rob_siz) < 0
                    %xd(i)
                    %yd(i)
                    %opt_path(opt_node,:) = traj(i,:)
                    %opt_node = opt_node+1
                    %disp('collision')
                    flag = flag + 1;
                    %{
                    if i==traj_len && pivot == 1
                        disp("collision")
                        disp('j');
                        disp(j);
                        disp('xd')
                        disp(xd(j));
                        disp('yd');
                        disp(yd(j));
                    end
                    %}
                end
            end
            
            if flag == 0
                %disp('flag = 0')
                %disp('i');
                %disp(i);
                %p1 = [x1 y1];
                %p2 = [x2 y2];
                %disp('[x1, y1]');
                %disp(p1);
                %disp('[x2. y1]');
                %disp(p2);
                opt_path(opt_node,:) = traj(i,:);
                opt_node = opt_node+1;
                pivot_set = i;
                break;
            end
        end
        if pivot_set==0
            pivot = pivot+1;
        else
            pivot = pivot_set;
        end
    end
    %{
    len = 1;
    opt_node = opt_node-1;
    for i=1:opt_node-1
        x1 = opt_path(i,1);
        x2 = opt_path(i+1,1);
        dx = x2-x1;
        y1 = opt_path(i,2);
        y2 = opt_path(i+1,2);
        dy = y2-y1;
        if abs(dx)>=abs(dy)
            n = abs(x1-x2)+1;
            xd = linspace(x1, x2, n);
            m = (y2-y1)/(x2-x1);
            if x2 ~= x1
                yd = floor(m*(xd-x1) + y1);
            else
            %    yd = traj(pivot,2);
            %    yd_2 = yd;
            end
        else
            n = abs(y1-y2)+1;
            yd = linspace(y1, y2, n);
            m = (x2-x1)/(y2-y1);
            if y2 ~= y1
                xd = floor(m*(yd-y1) + x1);
            else
            %    xd = traj(pivot,1);
            %    xd_2 = xd;
            end
        end
        opt_traj(len:len+n-1,1) = xd;
        opt_traj(len:len+n-1,2) = yd;
        len = len+n;
    end
    %}
    opt_traj = opt_path;
end