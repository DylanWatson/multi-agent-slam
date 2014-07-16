function data= ekfslam_sim(lm, wp, wp2)
%function data= ekfslam_sim(lm, wp)
%
% INPUTS: 
%   lm - set of landmarks
%   wp - set of waypoints
%
% OUTPUTS:
%   data - a data structure containing:
%       data.true: the vehicle 'true'-path (ie, where the vehicle *actually* went)
%       data.path: the vehicle path estimate (ie, where SLAM estimates the vehicle went)
%       data.state(k).x: the SLAM state vector at time k
%       data.state(k).P: the diagonals of the SLAM covariance matrix at time k
%
% NOTES:
%   This is a Distributed SLAM simulation
%   To run the simulation, run the 'slamSim' to start
%   Currently, the frontend is not supported for multi-robots
%   If you wish you change the landmarks and waypoints, modify the
%   variables in the 'slamSim' file
%
%
% VARIABLE INFO:
% idf - index tags for each landmark
% lm - landmarks for the map
% wp - waypoints for the first robot
% wp2 - waypoints for the second robot
% veh - creates the arrow used for the car for robot 1
% veh2 - creates the arrow used for the car for robot 2
% coordinates_for_distance - stores coordinates of small boxes to calculate
% the error between the true robot position and the prediction
%
%
% Tim Bailey and Juan Nieto 2004.
% Version 2.0


format compact
configfile; % ** USE THIS FILE TO CONFIGURE THE EKF-SLAM **

% Setup plots
fig=figure;
plot(lm(1,:),lm(2,:),'b*') %plots all of the set landmarks as blue in the simulation
hold on, axis equal %Keeps the current graph and adds another graph to it
plot(wp(1,:),wp(2,:), 'g', wp2(1,:),wp2(2,:),'r') %plots waypoints green and red for different vehicles
xlabel('meters'), ylabel('meters') %Label for the graph
set(fig, 'name', 'EKF-SLAM Simulator') %Sets label for the figure
h= setup_animations; %function that puts down all of the animations
h2 = setup_animations;
%Controls size of the vehicle in various ways
veh = [0 -WHEELBASE -WHEELBASE; 0 -2 2]; % vehicle animation
veh2 = [0 -WHEELBASE -WHEELBASE; 0 -2 2]; % vehicle animation
%veh2 = [0 -WHEELBASE -WHEELBASE; 0 -2 2]; 
plines=[]; % for laser line animation
pcount=0;
coordinates_for_distance = [];
count = 0;
area = [];
pause

% Initialise states and other global variables
global XX PX DATA XX2 PX2 DATA2
xtrue= zeros(3,1);
xtrue2 = zeros(3,1);
XX= zeros(3,1);
XX2 = zeros(3,1);
PX= zeros(3);
PX2 = zeros(3);
DATA= initialise_store(XX,PX,XX); % stored data for off-line
DATA2 = initialise_store(XX2, PX2, XX2);

% Initialise other variables and constants
dt= DT_CONTROLS;        % change in time between predicts
dtsum= 0;               % change in time since last observation
%x:y returns a matrix from x to y of x+1, x+2 until it gets to y 
ftag= 1:size(lm,2);     % identifier for each landmark
% zeros(x,y) creates a matrix of x*y that is full of zeros
da_table= zeros(1,size(lm,2)); % data association table  
da_table2 = zeros(1,size(lm,2)); % data association table 
iwp= 1;                 % index to first waypoint 
iwp2 = 1;
G= 0;                   % initial steer angle
G2 = 0; %Second vehicle steer angle
%QE and RE is some control noise and observation parameters
%QE is control noises
%RE is observation noises
QE= Q; RE= R; if SWITCH_INFLATE_NOISE, QE= 2*Q; RE= 2*R; end % inflate estimated noises (ie, add stabilising noise)
if SWITCH_SEED_RANDOM, rand('state',SWITCH_SEED_RANDOM), randn('state',SWITCH_SEED_RANDOM), end

if SWITCH_PROFILE, profile on -detail builtin, end

% Main loop 
while iwp ~= 0 || iwp2 ~= 0
    
    % Compute true data
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    [G2,iwp2]= compute_steering(xtrue2, wp2, iwp2, AT_WAYPOINT, G2, RATEG, MAXG, dt);
    if iwp==0 && NUMBER_LOOPS > 1, pack; iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % perform loops: if final waypoint reached, go back to first
    if iwp2==0 && NUMBER_LOOPS > 1, pack; iwp2=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % perform loops: if final waypoint reached, go back to first
    xtrue= vehicle_model(xtrue, V,G, WHEELBASE,dt);
    xtrue2= vehicle_model(xtrue2, V,G2, WHEELBASE,dt);
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    [Vn2,Gn2]= add_control_noise(V,G2,Q, SWITCH_CONTROL_NOISE);
    
    % EKF predict step
    predict (Vn,Gn,QE, WHEELBASE,dt);
    predict2 (Vn2,Gn2,QE, WHEELBASE,dt);
    %Y = ['Predicted: (', num2str(XX(1)), ',', num2str(XX(2)), ')'];
    %disp(Y)
    %print xtrue
    % If heading known, observe heading
    
   %Calculate the distance between xtrue and the predicated value
   %Takes the previous value and adds a new value to it
   coordinates_for_distance = [coordinates_for_distance, xtrue(1), xtrue(2), XX(1), XX(2)];
   
   %Don't do this if you only have one measurement
   if count > 0
       %Calculates the distance between xtrue and the predicted value from
       %the SLAM algorithm
       length_coordinates = [coordinates_for_distance(1), coordinates_for_distance(2);coordinates_for_distance(3),coordinates_for_distance(4)];
       length = pdist(length_coordinates, 'euclidean');
       %Calculates the distance between the first xtrue point and the
       %second xtrue point
       width_coordiantes = [coordinates_for_distance(1), coordinates_for_distance(2); coordinates_for_distance(5), coordinates_for_distance(6)];
       width = pdist(width_coordiantes, 'euclidean');
       %Calculates the area and then stores it for later summation
       area = [area,length*width];
       %Resets the matrix to only keep the two most recent values
       coordinates_for_distance = [coordinates_for_distance(5), coordinates_for_distance(6), coordinates_for_distance(7), coordinates_for_distance(8)];
       
   end
   count = count + 1;
    
    
    observe_heading(xtrue(3), SWITCH_HEADING_KNOWN);
    observe_heading2(xtrue2(3), SWITCH_HEADING_KNOWN);
    
    % Incorporate observation, (available every DT_OBSERVE seconds)
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        
        % Compute true data
        [z,ftag_visible]= get_observations(xtrue, lm, ftag, MAX_RANGE);
        [z2,ftag_visible2]= get_observations(xtrue2, lm, ftag, MAX_RANGE);
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        z2 = add_observation_noise(z2,R, SWITCH_SENSOR_NOISE);
        dim_of_z = size(z);
        dim_of_z2 = size(z2);
        number_of_z_landmarks = dim_of_z(1,2);
        number_of_z2_landmarks = dim_of_z2(1,2);
        for i = 1 : number_of_z_landmarks
            for j = 1 : number_of_z2_landmarks
                landmark1x = z(1,i);
                landmark1y = z(2,i);
                landmark2x = z2(1,j);
                landmark2y = z2(2,j);
                
                if abs(landmark1x - landmark2x) <= 1 && abs(landmark1y - landmark2y) <=1
                    avgx = (landmark1x+landmark2x)/2;
                    avgy = (landmark1y+landmark2y)/2;
                    robot1 = ['Robot 1: (', num2str(landmark1x, 5), ',', num2str(landmark1y, 5), ')'];
                    %disp(robot1)
                    robot2 = ['Robot 2: (', num2str(landmark2x, 5), ',', num2str(landmark2y, 5), ')'];
                    %disp(robot2)
                    avg_coordinate = ['Average: (', num2str(avgx, 5), ',', num2str(avgy, 5), ')'];
                   % disp(avg_coordinate)
                end
            end
        end
                
        
        
        
    
        % EKF update step
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(XX,z,ftag_visible, da_table);
            [zf2,idf2,zn2, da_table2]= data_associate_known(XX2,z2,ftag_visible2, da_table2);
        else
            [zf,idf, zn]= data_associate(XX,PX,z,RE, GATE_REJECT, GATE_AUGMENT); 
            [zf2,idf2, zn2]= data_associate(XX2,PX2,z2,RE, GATE_REJECT, GATE_AUGMENT); 
        end

        if SWITCH_USE_IEKF == 1
            update_iekf(zf,RE,idf, 5);
            update_iekf2(zf2,RE,idf2, 5);
        else
            update(zf,RE,idf, SWITCH_BATCH_UPDATE);
            update2(zf2,RE,idf2, SWITCH_BATCH_UPDATE);
        end
        augment(zn,RE);
        augment2(zn2,RE);
    end
    
    % Offline data store
    store_data(XX, PX, xtrue);
    store_data(XX2, PX2, xtrue2);
    
    % Plots
    xt= transformtoglobal(veh, xtrue);
    xt2= transformtoglobal(veh2, xtrue2);
    %Draws blue robot
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
    set(h2.xt, 'xdata', xt2(1,:), 'ydata', xt2(2,:))
    if SWITCH_GRAPHICS
        %transform to global takes the matricies of the position of the
        %rover and turns it into a graphical represtation that we see in
        %the animation
        xv= transformtoglobal(veh, XX(1:3));
        xv2 = transformtoglobal(veh2, XX2(1:3));
        pvcov= make_vehicle_covariance_ellipse(XX,PX);
        pvcov2= make_vehicle_covariance_ellipse(XX2,PX2);
        %Draws red robot
        %Green represents intersection between the red and the blue robot
        set(h.xv, 'xdata', xv(1,:), 'ydata', xv(2,:))
        set(h2.xv, 'xdata', xv2(1,:), 'ydata', xv2(2,:))
        %Draws ellipse for the uncertainity of the robot position
        set(h.vcov, 'xdata', pvcov(1,:), 'ydata', pvcov(2,:))  
        set(h2.vcov, 'xdata', pvcov2(1,:), 'ydata', pvcov2(2,:))
        pcount= pcount+1;
        if pcount == 120 % plot path infrequently
            pcount=0;
            %Draws the line of the path the robot actually takes
            set(h.pth, 'xdata', DATA.path(1,1:DATA.i), 'ydata', DATA.path(2,1:DATA.i))    
            set(h2.pth, 'xdata', DATA2.path(1,1:DATA2.i), 'ydata', DATA2.path(2,1:DATA2.i))
        end            
        
        if dtsum==0 & ~isempty(z) & ~isempty(z2) % plots related to observations
            set(h.xf, 'xdata', XX(4:2:end), 'ydata', XX(5:2:end))
            set(h2.xf, 'xdata', XX2(4:2:end), 'ydata', XX2(5:2:end))
            plines = make_laser_lines (z,XX(1:3));
            plines2 = make_laser_lines (z2,XX2(1:3));
            %Draws laser lines to landmarks
            set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:))
            set(h2.obs, 'xdata', plines2(1,:), 'ydata', plines2(2,:))
            pfcov= make_feature_covariance_ellipses(XX,PX);
            pfcov2 = make_feature_covariance_ellipses(XX2,PX2);
            %Draws ellipses for the uncertainty of a position for landmarks     
            set(h.fcov, 'xdata', pfcov(1,:), 'ydata', pfcov(2,:)) 
            set(h2.fcov, 'xdata', pfcov2(1,:), 'ydata', pfcov2(2,:)) 
        end
    end
    %Updates all figures with the recently made set commands
    drawnow
    
end % end of main loop
sum(area)
if SWITCH_PROFILE, profile report, end

data = finalise_data(DATA);
set(h.pth, 'xdata', data.path(1,:), 'ydata', data.path(2,:))    

clear global DATA 
clear global XX 
clear global PX

% 
%

function h= setup_animations()
h.xt= patch(0,0,'b','erasemode','xor'); % vehicle true
h.xv= patch(0,0,'r','erasemode','xor'); % vehicle estimate
h.pth= plot(0,0,'k.','markersize',2,'erasemode','background'); % vehicle path estimate
h.obs= plot(0,0,'y','erasemode','xor'); % observations
h.xf= plot(0,0,'r+','erasemode','xor'); % estimated features
h.vcov= plot(0,0,'r','erasemode','xor'); % vehicle covariance ellipses
h.fcov= plot(0,0,'r','erasemode','xor'); % feature covariance ellipses

%
%

function p= make_laser_lines (rb,xv)
% compute set of line segments for laser range-bearing measurements
if isempty(rb), p=[]; return, end
len= size(rb,2);

lnes(1,:)= zeros(1,len)+ xv(1);
lnes(2,:)= zeros(1,len)+ xv(2);
lnes(3:4,:)= transformtoglobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
p= line_plot_conversion (lnes);

%
%

function p= make_vehicle_covariance_ellipse(x,P)
% compute ellipses for plotting vehicle covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];

p= make_ellipse(x(1:2), P(1:2,1:2), circ);

function p= make_feature_covariance_ellipses(x,P)
% compute ellipses for plotting feature covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;
circ= 2*[cos(phi); sin(phi)];

lenx= length(x);
lenf= (lenx-3)/2;
p= zeros (2, lenf*(N+2));

ctr= 1;
for i=1:lenf
    ii= ctr:(ctr+N+1);
    jj= 2+2*i; jj= jj:jj+1;
    
    p(:,ii)= make_ellipse(x(jj), P(jj,jj), circ);
    ctr= ctr+N+2;
end

%
%

function p= make_ellipse(x,P,circ)
% make a single 2-D ellipse 
r= sqrtm_2by2(P);
a= r*circ;
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];

%
%

function data= initialise_store(x,P, xtrue)
% offline storage initialisation
data.i=1;
data.path= x;
data.true= xtrue;
data.state(1).x= x;
%data.state(1).P= P;
data.state(1).P= diag(P);

%
%

function store_data(x, P, xtrue)
% add current data to offline storage
global DATA
CHUNK= 5000;
len= size(DATA.path,2);
if DATA.i == len % grow array exponentially to amortise reallocation
    if len < CHUNK, len= CHUNK; end
    DATA.path= [DATA.path zeros(3,len)];
    DATA.true= [DATA.true zeros(3,len)];
    pack
end
i= DATA.i + 1;
DATA.i= i;
DATA.path(:,i)= x(1:3);
DATA.true(:,i)= xtrue;
DATA.state(i).x= x;
%DATA.state(i).P= P;
DATA.state(i).P= diag(P);

%
%

function data = finalise_data(data)
% offline storage finalisation
data.path= data.path(:,1:data.i);
data.true= data.true(:,1:data.i);
