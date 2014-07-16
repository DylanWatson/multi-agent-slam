function KF_cholesky_update2(v,R,H)
%function KF_cholesky_update(v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P]
% the innovation [v,R] and the (linearised) observation model H.
% The result is calculated using Cholesky factorisation, which
% is more numerically stable than a naive implementation.
%
% Tim Bailey 2003
% Adapted from code by Jose Guivant 
global XX2 PX2

%PHt= PX*H';
PHt= (H*PX2)'; % Matlab is column-major, so (H*PX)' is more efficient than PX*H' [Tim 2004]
S= H*PHt + R;

S= (S+S')*0.5; % ensure is symmetric
SChol= chol(S);

SCholInv= inv(SChol); % triangular matrix
W1= PHt * SCholInv;
W= W1 * SCholInv';

XX2= XX2 + W*v; % update 
PX2= PX2 - W1*W1';
