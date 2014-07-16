function predict2 (v,g,Q,WB,dt)
%function predict (v,g,Q,WB,dt)
%
% Inputs:
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   XX, PX - predicted state and covariance (global variables)
%
% Tim Bailey 2004.

global XX2 PX2

s= sin(g+XX2(3)); c= cos(g+XX2(3));
vts= v*dt*s; vtc= v*dt*c;

% jacobians   
Gv= [1 0 -vts;
     0 1  vtc;
     0 0 1];
Gu= [dt*c -vts;
     dt*s  vtc;
     dt*sin(g)/WB v*dt*cos(g)/WB];
  
% predict covariance
PX2(1:3,1:3)= Gv*PX2(1:3,1:3)*Gv' + Gu*Q*Gu';
if size(PX2,1)>3
    PX2(1:3,4:end)= Gv*PX2(1:3,4:end);
    PX2(4:end,1:3)= PX2(1:3,4:end)';
end    

% predict state
XX2(1:3)= [XX2(1) + vtc; 
          XX2(2) + vts;
         pi_to_pi(XX2(3)+ v*dt*sin(g)/WB)];
     