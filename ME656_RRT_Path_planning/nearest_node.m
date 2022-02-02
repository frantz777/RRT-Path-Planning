function near_node = nearest_node(Path, x, y, obstacles)
    test_x = x;
    test_y = y;
    
    last_closest = 200;
    nearest_node_number = 1 ; % the ID of the node had has test to be closest, just initiate value with zero
    nodes = getGlobalnode_num;
    for i = 1:nodes
        
        node_x = Path(i,2); %x cordinate for the ith node
        node_y = Path(i,3); %y cordinate for the ith node
        
        Distance = abs(((test_x - node_x)^2 + (test_y - node_y)^2))^(1/2);
        
        if Distance < last_closest && collision_check_segment(node_x,node_y,test_x,test_y,obstacles) == 0
            
            last_closest = Distance;
            nearest_node_number = i;
            
        end
    end
    if nearest_node_number < 1
        nearest_node_number = 1;
    end
    %disp(i)
    near_node = nearest_node_number;% not sure Im returning the correct thing here
    
end