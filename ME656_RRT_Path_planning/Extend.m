function Ex = Extend(Path, Xtest, Ytest, obstacles)
        
        Index_of_near  = nearest_node(Path, Xtest, Ytest, obstacles); %index of nearest node
        %disp(Index_of_near)
        
        node_x = Path(Index_of_near,2); %x cordinate for the ith node
        node_y = Path(Index_of_near,3); %y cordinate for the ith node
        
        Delta_x = Xtest - node_x;
        Delta_y = Ytest - node_y;
        
        Dist = (abs(Delta_x^2 + Delta_y^2))^(1/2);
        
        new_point_x = 2*(Delta_x/Dist) + node_x;
        new_point_y = 2*(Delta_y/Dist) + node_y;
        
        setGlobalnode_num( getGlobalnode_num + 1);
        
        Path(getGlobalnode_num,1) = Index_of_near;
        Path(getGlobalnode_num,2) = new_point_x;
        Path(getGlobalnode_num,3) = new_point_y;
        
        %disp('Index_of_near')
        %disp(Index_of_near)
        %disp(Path)
        
        
        
        Ex = Path;
end