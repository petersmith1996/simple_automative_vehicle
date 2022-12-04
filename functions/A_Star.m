%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM Demo
% Interactive A* search demo
% 04-26-2005
%   Copyright 2009-2010 The MathWorks, Inc.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function traj = A_Star(pos, goal , MAP, obs_MAP, rob_siz)
    %fp = fopen('log.txt', 'w');
    %DEFINE THE 2-D MAP ARRAY

    MAX_X=100;
    MAX_Y=100;
    MAX_VAL=100;

    j=0;
    x_val = 1;
    y_val = 1;
    n=0;%Number of Obstacles

    xval = goal(1);
    yval = goal(2);


    xTarget=xval;%X Coordinate of the Target
    yTarget=yval;%Y Coordinate of the Target
    MAP(xTarget,yTarget)=0;%Initialize MAP with location of the target
    %plot(xval+.5,yval,'gd');
    %text(xval+1,yval,'Target')
    %grid on;
    %hold on;

    xval = pos(1);
    yval = pos(2);

    xStart=xval;%Starting Position
    yStart=yval;%Starting Position
    MAP(xval,yval)=1;
    %plot(xval+.5,yval+.5,'bo');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i; 
                CLOSED(k,2)=j; 
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    path_cost=0;

    %goal_distance=distance(xNode,yNode,xTarget,yTarget);
    goal_distance=manhatten(xNode,yNode,xTarget,yTarget);

    coll = collcheck(obs_MAP, xNode, yNode, rob_siz);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %coll = 1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %fprintf(fp, 'start - x: %2.1f, y: %2.1f, coll: %d\n', xNode, yNode, coll); 
    if coll > -1
        %disp('collision detect');
        OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
        OPEN(OPEN_COUNT,1)=0;
    end
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % START ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
        exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
        exp_count=size(exp_array,1);
        %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
        %OPEN LIST FORMAT
        %--------------------------------------------------------------------------
        %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
        %--------------------------------------------------------------------------
        %EXPANDED ARRAY FORMAT
        %--------------------------------
        %|X val |Y val ||h(n) |g(n)|f(n)|
        %--------------------------------
        for i=1:exp_count
            %fprintf(fp, 'i: %d\n', i);
            flag=0;
            for j=1:OPEN_COUNT
            %fprintf(fp, 'j: %d\n', j);
                if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
                    OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
                    if OPEN(j,8)== exp_array(i,5)
                        %UPDATE PARENTS,gn,hn
                        OPEN(j,4)=xNode;
                        OPEN(j,5)=yNode;
                        OPEN(j,6)=exp_array(i,3);
                        OPEN(j,7)=exp_array(i,4);
                    end;%End of minimum fn check
                    flag=1;
                end;%End of node check
       %         if flag == 1
       %             break;
            end;%End of j for
            if flag == 0
                if exp_array(i,1)>2 && exp_array(i,1)<99 && exp_array(i,2)>2 && exp_array(i,2)<99
                    I = find(obs_MAP(exp_array(i,1)-2:exp_array(i,1)+2,exp_array(i,2)-2:exp_array(i,2)+2)==-1);
                    if size(I,1)>0
                    else
                        coll = collcheck(obs_MAP, exp_array(i,1), exp_array(i,2), rob_siz);
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %coll = 1;
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %fprintf(fp, 'x: %2.1f, y: %2.1f, coll: %d\n', xNode, yNode, coll); 
                        if coll > -1
                            %disp('collision detect');
                            OPEN_COUNT = OPEN_COUNT+1;
                            OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
                        end
                    end
                end
            end;%End of insert new element into the OPEN list
        end;%End of i for
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %END OF WHILE LOOP
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Find out the node with the smallest fn 
        index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
        if (index_min_node ~= -1)
            %Set xNode and yNode to the node with minimum fn
            xNode=OPEN(index_min_node,2);
            yNode=OPEN(index_min_node,3);
            path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
            %Move the Node to list CLOSED
            CLOSED_COUNT=CLOSED_COUNT+1;
            CLOSED(CLOSED_COUNT,1)=xNode;
            CLOSED(CLOSED_COUNT,2)=yNode;
            OPEN(index_min_node,1)=0;
        else
            %No path exists to the Target!!
            NoPath=0;%Exits the loop!
        end;%End of index_min_node check
    end;%End of While Loop
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path

    i=size(CLOSED,1);
    Optimal_path=[];
    xval=CLOSED(i,1);
    yval=CLOSED(i,2);
    i=1;
    Optimal_path(i,1)=xval;
    Optimal_path(i,2)=yval;
    i=i+1;

    if ( (xval == xTarget) && (yval == yTarget))
        inode=0;
        %Traverse OPEN and determine the parent nodes
        parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
        parent_y=OPEN(node_index(OPEN,xval,yval),5);

        while( parent_x ~= xStart || parent_y ~= yStart)
            Optimal_path(i,1) = parent_x;
            Optimal_path(i,2) = parent_y;
            %Get the grandparents:-)
            inode=node_index(OPEN,parent_x,parent_y);
            parent_x=OPEN(inode,4);%node_index returns the index of the node
            parent_y=OPEN(inode,5);
            i=i+1;
        end;
        
        j=size(Optimal_path,1);
        %Plot the Optimal Path!
        %{
        p=plot(Optimal_path(j,1),Optimal_path(j,2),'bo');
        hold on;
        disp('plotting path');
        plot(Optimal_path(:,1),Optimal_path(:,2));
        pbaspect([1 1 1]);
        hold on;
        disp('plotting obs');
        for i=1:100
            for j=1:100
                if MAP(i,j) == -1
                    plot(i, j, 's');
                    grid on;
                    hold on;
                end
            end
        end
        %}
        %{
        disp('plotting CLOSED');
        for i=1:size(CLOSED,1)
            plot(CLOSED(i,1), CLOSED(i,2),'x');
            hold on;
        end
        %}

        traj = Optimal_path;
    
    else
        %{
        pause(1);
        h=msgbox('Sorry, No path exists to the Target!','warn');
        uiwait(h,5);
        %}
        traj = [];
    end

    %{
    traj_len = size(traj,1)

    for i=1:traj_len
        tmp = traj(i,:);
        traj(i,:) = traj(traj_len+1-i,:);
        traj(traj_len+1-i,:) = tmp;
    end
    %}
    %fclose(fp);
end
    





