function propagate_path = propagate_KF_path(Path_Taken,obstacles)
    
    total_steps = size(Path_Taken,1);
    stepsDupe = total_steps;
   % disp(total_steps)
   % Length1 = total_steps ;
     Pk = eye(4);
    
     A = [1 1 0 0;
          0 1 0 0;
          0 0 1 1;
          0 0 0 1];
     
    Q = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
     
     R = [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];
      
    Uncert_Mat = [];   
    step = 1;
      
    while not(total_steps == 0)   
      
    if collision_check_segment(Path_Taken(total_steps,1), Path_Taken(total_steps,2), (Path_Taken(total_steps,1) + 5), Path_Taken(total_steps,2),obstacles) == 0 && collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),Path_Taken(total_steps,1),Path_Taken(total_steps,2) + 5 ,obstacles) == 0 && collision_check_segment(Path_Taken(total_steps,1), Path_Taken(total_steps,2), (Path_Taken(total_steps,1) - 5), Path_Taken(total_steps,2),obstacles) == 0 && collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),Path_Taken(total_steps,1),Path_Taken(total_steps,2) - 5 ,obstacles) == 0 % not in X or Y range
        %disp('No Sensor')
     
        H = [0 0 0 0;
             0 1 0 0;
             0 0 0 0;
             0 0 0 1]; 
         
        Pk = propagate_KF(A,H,Q,R,Pk);
        
        Uncert_Mat(step,1) = Path_Taken(total_steps,1); %X
        Uncert_Mat(step,2) = Path_Taken(total_steps,2); %Y
        Uncert_Mat(step,3) = Pk(1,1); % uncertainty in X
        Uncert_Mat(step,4) = Pk(3,3); % uncertainty in y
     
    elseif collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),(Path_Taken(total_steps,1) + 5),Path_Taken(total_steps,2),obstacles) == 1 | collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),(Path_Taken(total_steps,1) - 5),Path_Taken(total_steps,2),obstacles) == 1                              %&& collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),Path_Taken(total_steps,1),Path_Taken(total_steps,2) + 5 ,obstacles) == 0 % in X but not Y
        %disp('X Sensor')
        
        H = [1 0 0 0;
             0 1 0 0;
             0 0 0 0;
             0 0 0 1]; 
        Pk = propagate_KF(A,H,Q,R,Pk);
        
        Uncert_Mat(step,1) = Path_Taken(total_steps,1); %X
        Uncert_Mat(step,2) = Path_Taken(total_steps,2); %Y
        Uncert_Mat(step,3) = Pk(1,1); % uncertainty in X
        Uncert_Mat(step,4) = Pk(3,3); % uncertainty in y
        
    elseif collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),(Path_Taken(total_steps,1) - 5),Path_Taken(total_steps,2),obstacles) == 0 | collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),(Path_Taken(total_steps,1) - 5),Path_Taken(total_steps,2),obstacles) == 0           %&& collision_check_segment(Path_Taken(total_steps,1),Path_Taken(total_steps,2),Path_Taken(total_steps,1),Path_Taken(total_steps,2) + 5,obstacles) == 1
        %disp('No Sensor')
        
        H = [0 0 0 0;
             0 1 0 0;
             0 0 1 0;
             0 0 0 1]; 
        Pk = propagate_KF(A,H,Q,R,Pk);
        
        Uncert_Mat(step,1) = Path_Taken(total_steps,1); %X
        Uncert_Mat(step,2) = Path_Taken(total_steps,2); %Y
        Uncert_Mat(step,3) = Pk(1,1); % uncertainty in X
        Uncert_Mat(step,4) = Pk(3,3); % uncertainty in y
    else 
        disp('Not caught')
        
    end
    total_steps = total_steps - 1;
    step = step + 1; 
    %disp(Pk)
    end
    propagate_path = Uncert_Mat;
end

