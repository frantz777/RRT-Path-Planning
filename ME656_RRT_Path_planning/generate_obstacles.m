
clear all; close all; clc;
for s = 1:3
figure(s); hold on;
axis([0 100 0 100]);
box on;

previous_greatest_uncert = 0;
previous_least_uncert = 500;
last_shortest = 200;

% Define and plot the start state for the planning problem
%               x  y
start_state = [ 5 50];
plot(start_state(1),start_state(2),'.r','MarkerSize',20);

% Define and plot the goal region for the planning problem
%              x1 y1  x2  y2   x3  y3  x4  y4
goal_region = [90  0 100   0  100 100  90 100];
goal_x = [goal_region(1) goal_region(3) goal_region(5) goal_region(7)];
goal_y = [goal_region(2) goal_region(4) goal_region(6) goal_region(8)];
patch(goal_x,goal_y,'green');

% Define and plot the locations of the obstacles
%             x1 y2 x2 y2 x3 y3 x4 y4
obstacles = [  5 10 15 10 15 20  5 20; % obstacle 1
              10 40 20 40 20 50 10 50; % obstacle 2
              20 70 30 70 30 80 20 80; % ...etc...
              30 20 40 20 40 30 30 30; 
              40 50 50 50 50 60 40 60;
              50  5 60  5 60 15 50 15;
              55 80 65 80 65 90 55 90;
              60 40 70 40 70 50 60 50;
              70 20 80 20 80 30 70 30;
              75 65 85 65 85 75 75 75 ];

num_obstacles = size(obstacles,1);

for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end
s = s+1;
end


for trials = 1:100 
disp(trials)

Path = Build_RRT_Map(obstacles);
Path(1,1)= 1;
Index = getGlobalnode_num ;
sum = 0;
sums =0;

Path_Taken = []; %zeros(200,2);
step = 1;



while not(Index == 1)
    
    child_x = Path(Index,2); %x cordinate for the ith node
    child_y = Path(Index,3); %y cordinate for the ith node
    
    Path_Taken(step,1) = child_x;
    Path_Taken(step,2) = child_y;
    
    step = step + 1; 
    Index = Path(Index,1);
    
    parent_x = Path(Index,2);
    parent_y = Path(Index,3);
    
    Path_Taken(step,1) = parent_x;
    Path_Taken(step,2) = parent_y;
    step = step + 1 ;
    
    Delta_x = parent_x - child_x;
    Delta_y = parent_y - child_y;

        
    Dist = (abs(Delta_x^2 + Delta_y^2))^(1/2); 
    sum = sum + Dist; %actual length of path
end
Length = sum;


recent_path_w_uncert = propagate_KF_path(Path_Taken,obstacles);
length_of_uncert_mat = size(recent_path_w_uncert,1);
%recent_path_w_uncert =[];
steps_for_pathD = length_of_uncert_mat;


recent_path_length = 0;%

while step <= steps_for_pathD - 1  %determine path lengh of this path
    currentx = recent_path_w_uncert(step,1);
    currenty = recent_path_w_uncert(step,2);
    
    nextx = recent_path_w_uncert(step + 1,1);
    nexty = recent_path_w_uncert(step + 1,2);
    
    
    
    Delta_x = nextx - currentx;
    Delta_y = nexty - currenty;
    %Index = Path(Index,1);
        
    Dist = (abs(Delta_x^2 + Delta_y^2))^(1/2);
    
    recent_path_length = recent_path_length + Dist; %actual length of path
end
    if recent_path_length < last_shortest
        shortest_path = recent_path_w_uncert;
        last_shortest = recent_path_length;
    end

    if (recent_path_w_uncert(length_of_uncert_mat,3)^.5 + recent_path_w_uncert(length_of_uncert_mat,4)^.5) > previous_greatest_uncert 
        greatest_uncert = recent_path_w_uncert;
        previous_greatest_uncert = (recent_path_w_uncert(length_of_uncert_mat,3)^.5 + recent_path_w_uncert(length_of_uncert_mat,4)^.5);
    end
    if (recent_path_w_uncert(length_of_uncert_mat,3)^.5 + recent_path_w_uncert(length_of_uncert_mat,4)^.5) < previous_least_uncert 
        least_uncert = recent_path_w_uncert;
        previous_least_uncert = (recent_path_w_uncert(length_of_uncert_mat,3)^.5 + recent_path_w_uncert(length_of_uncert_mat,4)^.5);
    end
    
trials = trials + 1 ;
end

%plot shortest path
figure(1);
title('Shortest Path')
for i = 1:size(shortest_path,1)
    
    if i <= (size(shortest_path,1) - 1)
        x = shortest_path(i,1);
        y = shortest_path(i,2);
        plot(x,y,'.r','MarkerSize',5);
        x_next = shortest_path(i+1,1);
        y_next = shortest_path(i+1,2);
        line([x,x_next],[y,y_next]);
        i = i+1;
        ellipse(shortest_path(i,3)^.5,shortest_path(i,4)^.5,0,shortest_path(i,1),shortest_path(i,2),'r',300); % PLOTS uncert
    else
        plot(x,y,'.r','MarkerSize',5);
    end
end
disp('Shortest Path');
disp(size(shortest_path,1));

%Plot greatest Uncert
figure(2);
title('Greatest Uncertainty')
for i = 1:size(greatest_uncert,1)
    
    if i <= (size(greatest_uncert,1) - 1)
        x = greatest_uncert(i,1);
        y = greatest_uncert(i,2);
        plot(x,y,'.r','MarkerSize',5);
        x_next = greatest_uncert(i+1,1);
        y_next = greatest_uncert(i+1,2);
        line([x,x_next],[y,y_next]);
        i = i+1;
        ellipse(greatest_uncert(i,3)^.5,greatest_uncert(i,4)^.5,0,greatest_uncert(i,1),greatest_uncert(i,2),'r',300); % PLOTS uncert
    else
        plot(x,y,'.r','MarkerSize',5);
    end
end
disp('Greatest Uncertainty');
disp(greatest_uncert(size(greatest_uncert,1),1)^.5 + greatest_uncert(size(greatest_uncert,1),2)^.5);

%plot Least Uncert
figure(3);
title('Least Uncertainty')
for i = 1:size(least_uncert,1)
    if i <= (size(least_uncert,1) - 1)
        x = least_uncert(i,1);
        y = least_uncert(i,2);
        plot(x,y,'.r','MarkerSize',5);
        x_next = least_uncert(i+1,1);
        y_next = least_uncert(i+1,2);
        line([x,x_next],[y,y_next]);
        i = i+1;
        ellipse(least_uncert(i,3)^.5,least_uncert(i,4)^.5,0,least_uncert(i,1),least_uncert(i,2),'r',300); % PLOTS uncert
    else
        plot(x,y,'.r','MarkerSize',5);
    end
end
disp('Least Uncertainty');
disp(least_uncert(size(least_uncert,1),1)^.5 + least_uncert(size(least_uncert,1),2)^.5);


