function observe_heading2(phi, useheading)
%function observe_heading(phi, useheading)
%
% Perform state update for a given heading measurement, phi,
% with fixed measurement noise: sigmaPhi

global XX2 PX2
if useheading==0, return, end
sigmaPhi= 0.01*pi/180; % radians, heading uncertainty

H= zeros(1,length(XX2));
H(3)= 1;
v= pi_to_pi(phi - XX2(3));

KF_cholesky_update(v, sigmaPhi^2, H);
