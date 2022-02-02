function propagate = propagate_KF(A,H,Q,R,Pk)

   Pk = ((A*Pk*(A.') + Q)^-1 + (H.')*(R^-1)*H)^-1;
   
   propagate = Pk;  
   
end