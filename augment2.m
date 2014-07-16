function augment2(z,R)
%function augment(z,R)
%
% Inputs:
%   z, R - range-bearing measurements and covariances, each of a new feature
%
% Outputs:
%   x, P - augmented SLAM state and covariance (global variables)
%
% Notes: 
%   - We assume the number of vehicle pose states is three.
%   - Only one value for R is used, as all measurements are assumed to 
%   have same noise properties.
%
% Tim Bailey 2004.

% add new features to state
for i=1:size(z,2)
    add_one_z(z(:,i),R);
end

%
%

function add_one_z(z,R)
global XX2 PX2

len= length(XX2);
r= z(1); b= z(2);
s= sin(XX2(3)+b); 
c= cos(XX2(3)+b);

% augment x
XX2= [XX2;
     XX2(1) + r*c;
     XX2(2) + r*s];

% jacobians
Gv= [1 0 -r*s;
     0 1  r*c];
Gz= [c -r*s;
     s  r*c];
     
% augment P
rng= len+1:len+2;
PX2(rng,rng)= Gv*PX2(1:3,1:3)*Gv' + Gz*R*Gz'; % feature cov
PX2(rng,1:3)= Gv*PX2(1:3,1:3); % vehicle to feature xcorr
PX2(1:3,rng)= PX2(rng,1:3)';
if len>3
    rnm= 4:len;
    PX2(rng,rnm)= Gv*PX2(1:3,rnm); % map to feature xcorr
    PX2(rnm,rng)= PX2(rng,rnm)';
end
