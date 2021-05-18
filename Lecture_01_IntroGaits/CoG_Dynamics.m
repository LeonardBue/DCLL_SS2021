% *************************************************************************
%
%   Script to integrated GRFs to assess GoG Dynamics
%
% INPUT:    - None
% OUTPUT:   - None
%
%   C. David Remy remy@inm.uni-stuttgart.de
%   Matlab R2018
%   4/22/2020
%   v21

%% Initial Setup
% Make a clean sweep:
clear all
close all
clc

% Load GRF data from file:
data = load('GRF_Data.mat');

% Prepare output
figure('Name','Ground reaction force data','Units','Normalized','OuterPosition',[0,0,1,1])

% Extract parameters: 
% (Most of the following could be done much more efficiently, but it shows
% what data is stored in the file) 
g = data.g; % [m/s^2]
m = data.m; % [kg]
speedWalk = data.speedWalk; % [m/s]
speedRun  = data.speedRun;  % [m/s]
% Timing:
tWalk = data.tWalk;
tRun = data.tRun;
% GRFs (de-normalized from units of Body-weight to [N])
WalkHor_L = data.WalkHor*m*g;
WalkVer_L = data.WalkVer*m*g;
RunHor_L = data.RunHor*m*g;
RunVer_L = data.RunVer*m*g;
% Create GRFs for right legs by taking advantage of the fact that this is a
% symmetrical gait:  
WalkHor_R = [WalkHor_L(501:1000),WalkHor_L(1:500)];
WalkVer_R = [WalkVer_L(501:1000),WalkVer_L(1:500)];
RunHor_R = [RunHor_L(501:1000),RunHor_L(1:500)];
RunVer_R = [RunVer_L(501:1000),RunVer_L(1:500)];

% Show 
subplot(2,2,1)
hold on
plot(tWalk,WalkHor_L)
plot(tWalk,WalkHor_R)
plot(tWalk,WalkVer_L)
plot(tWalk,WalkVer_R)
title('Walking GRFs')
xlabel('Time [s]')
ylabel('Force [N]');
legend('L_{hor}','R_{hor}','L_{ver}','R_{ver}');
axis([0,tWalk(end),-500,2500]);
subplot(2,2,2)
hold on
plot(tRun,RunHor_L)
plot(tRun,RunHor_R)
plot(tRun,RunVer_L)
plot(tRun,RunVer_R)
title('Running GRFs')
xlabel('Time [s]')
ylabel('Force [N]');
axis([0,tRun(end),-500,2500]);

% To get accelerations, these forces are devided by m and the gravitational
% acceleration is subtracted from the vertical GRFS: 
a_WalkHor = (WalkHor_L + WalkHor_R)/m;
a_WalkVer = (WalkVer_L + WalkVer_R)/m - g;
a_RunHor = (RunHor_L + RunHor_R)/m;
a_RunVer = (RunVer_L + RunVer_R)/m - g;
% In theory these signals should be zero-mean, as the motion is not
% accelerated on averge.  However, this is data, so we have to correct for
% any potential drift by hand:  
a_WalkHor = a_WalkHor - mean(a_WalkHor);
a_WalkVer = a_WalkVer - mean(a_WalkVer);
a_RunHor = a_RunHor - mean(a_RunHor);
a_RunVer = a_RunVer - mean(a_RunVer);

% We can now simply integrate these accelerations to get the COM
% velocities.  Here we take advantage of the fact that we know the average
% values for all velocities to compute the integration constants:  
v_WalkHor = cumsum(a_WalkHor)*(tWalk(2)-tWalk(1));
v_WalkHor = v_WalkHor-mean(v_WalkHor) + speedWalk;
v_WalkVer = cumsum(a_WalkVer)*(tWalk(2)-tWalk(1));
v_WalkVer = v_WalkVer-mean(v_WalkVer) + 0;
v_RunHor = cumsum(a_RunHor)*(tRun(2)-tRun(1));
v_RunHor = v_RunHor-mean(v_RunHor) + speedRun;
v_RunVer = cumsum(a_RunVer)*(tRun(2)-tRun(1));
v_RunVer = v_RunVer-mean(v_RunVer) + 0;

% Finally, we can compute the fluctuations in hight.  There is no
% need for an integration constant, as the reference hight is arbitrary.
h_Walk = cumsum(v_WalkVer)*(tWalk(2)-tWalk(1));
h_Run = cumsum(v_RunVer)*(tRun(2)-tRun(1));

% Based on these results, we can compute the kinetic, potential, and total
% energy that is exchanged during a stride: 
e_kin_Walk = 0.5*m*(v_WalkHor.^2 + v_WalkVer.^2);
e_pot_Walk = m*g*h_Walk;
e_tot_Walk = e_kin_Walk + e_pot_Walk;
e_kin_Run = 0.5*m*(v_RunHor.^2 + v_RunVer.^2);
e_pot_Run = m*g*h_Run;
e_tot_Run = e_kin_Run + e_pot_Run;

subplot(2,2,3)
hold on
plot(tWalk,e_kin_Walk)
plot(tWalk,e_pot_Walk)
plot(tWalk,e_tot_Walk)
axis([0,tWalk(end),-50,650]);
title('Walking Energies')
xlabel('Time [s]')
ylabel('Energy [J]');
legend('E_{kin}','E_{pot}','E_{tot}');
subplot(2,2,4)
hold on
plot(tRun,e_kin_Run)
plot(tRun,e_pot_Run)
plot(tRun,e_tot_Run)
axis([0,tRun(end),-50,650]);
title('Running Energies')
xlabel('Time [s]')
ylabel('Energy [J]');


% Prepare output for direct comparison
figure('Name','Comparision of walking and running','Units','Normalized','OuterPosition',[0,0,1,1])
subplot(2,2,1)
hold on
plot(tWalk,e_kin_Walk-e_kin_Walk(1))
plot(tWalk,e_pot_Walk-e_pot_Walk(1))
plot(tWalk,e_tot_Walk-e_kin_Walk(1)-e_pot_Walk(1))
axis([0,tWalk(end),-200,100]);
title('Walking Energies (y-shifted)')
xlabel('Time [s]')
ylabel('Energy [J]');
legend('E_{kin}','E_{pot}','E_{tot}','Location','SouthEast');
subplot(2,2,2)
hold on
plot(tRun,e_kin_Run-e_kin_Run(1))
plot(tRun,e_pot_Run-e_pot_Run(1))
plot(tRun,e_tot_Run-e_kin_Run(1)-e_pot_Run(1))
axis([0,tRun(end),-200,100]);
title('Running Energies (y-shifted)')
xlabel('Time [s]')
ylabel('Energy [J]');
% To show CoG trajectories, we further integrated forward speed
x_Walk = cumsum(v_WalkHor)*(tWalk(2)-tWalk(1));
x_Run = cumsum(v_RunHor)*(tRun(2)-tRun(1));

subplot(2,2,3)
hold on
plot(x_Walk,h_Walk)
axis equal
title('Walking CoG Motion')
xlabel('Distance [m]');
ylabel('Height [m]')
subplot(2,2,4)
hold on
plot(x_Run,h_Run)
axis equal
title('Running CoG Motion')
xlabel('Distance [m]');
ylabel('Height [m]')
