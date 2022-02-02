
function Build_RRT = Build_RRT_Map(obstacles)
    setGlobalnode_num(1);
    total_nodes = 1500;
    start_state = [5 50];
    Path = zeros(100,3);  %hold order of branches
    Path(1,1) = 0;
    Path(1,2)= 5;
    Path(1,3)= 50;

    while not(Path(getGlobalnode_num,2) > 90)                            %for k = 1:total_nodes % change to while loop once we get things working
        Random_XY_Generator = randi([0 100],1,2);
        X_random =  Random_XY_Generator(1,1);
        Y_random =  Random_XY_Generator(1,2);
       
        if collision_check_point(X_random, Y_random, obstacles) == 0 
            
            Path = Extend(Path, X_random, Y_random, obstacles);
            
        end
    end
    Build_RRT = Path;
end