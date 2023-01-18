%%
% OBJECTIVE: 
% Determine how sensitive the AP axis is to different digitizer points
% created in a model. 
% I determine a region of potential points that could be digitized on the
% posterior and anteior tibial plateau. Then randomly generate combinations
% to determine the AP axis and see how repeatable it is. 

% Plot the femoral frame. 

%%
clear all
close all
clc

%% Load stl file data
Leg = stlread('LEXTREM.stl'); 
LegPoints = Leg.Points(:,:);
x = Leg.Points(:,1);
y = Leg.Points(:,2);
z = Leg.Points(:,3);

%% Identify tibia
[P_range, A_range, L_tib, M_tib, O_distal_tib] = AP_rangeFilter(x,y,z);

%% Average Posterior point
P_tibAvg = mean(P_range);
plot3(P_tibAvg(1),P_tibAvg(2),P_tibAvg(3), 'g*','LineWidth',3)

%% Average Anterior point
A_tibAvg = mean(A_range);
A_tibAvg(3) = P_tibAvg(3); % Adjust the z to be in line with the posterior point
plot3(A_tibAvg(1),A_tibAvg(2),A_tibAvg(3), 'g*','LineWidth',3)

%% Tibia Origin
plat = [L_tib; M_tib; A_tibAvg; P_tibAvg];
O_tib = mean(plat,1);

%% TIBIA FRAME: DEFRATE METHOD
[uz_tib,ux_tib,uy_tib] = AP_ZXY(O_tib, A_tibAvg, P_tibAvg, O_distal_tib);
T_tibground = [ux_tib(1), uy_tib(1),uz_tib(1) O_tib(1);ux_tib(2), uy_tib(2),uz_tib(2) O_tib(2);ux_tib(3), uy_tib(3),uz_tib(3) O_tib(3);0 0 0 1];
    ux_tibground = T_tibground*[1000 0 0 1]';
    uy_tibground = T_tibground*[0 1000 0 1]';
    uz_tibground = T_tibground*[0 0 1000 1]';
    myplot(T_tibground(1:3,4),ux_tibground,uy_tibground,uz_tibground,'r','r','r','k',2)
    
%% FEMUR FRAME
[ux_fem, uy_fem, uz_fem, O_fem] = femFrame_YZX(x,y,z);
T_femground = [ux_fem(1), uy_fem(1),uz_fem(1) O_fem(1);ux_fem(2), uy_fem(2),uz_fem(2) O_fem(2);ux_fem(3), uy_fem(3),uz_fem(3) O_fem(3);0 0 0 1];
    ux_femground = T_femground*[1000 0 0 1]';
    uy_femground = T_femground*[0 1000 0 1]';
    uz_femground = T_femground*[0 0 1000 1]';
    myplot(T_femground(1:3,4),ux_femground,uy_femground,uz_femground,'r','r','r','k',2)
    
%% Femur wrt tibia
T_groundtib = inv(T_tibground);
T_FemTib = T_groundtib*T_femground;
T_femtibground = T_tibground*T_FemTib;
    ux_femtib = T_femtibground*[1000 0 0 1]';
    uy_femtib = T_femtibground*[0 1000 0 1]';
    uz_femtib = T_femtibground*[0 0 1000 1]';
    myplot(T_femtibground(1:3,4),ux_femtib,uy_femtib,uz_femtib,'k','k','k','k',2)
T_TibFem = inv(T_FemTib);

%% Applying the T_FemTib_Traj matrices to this model
scale = 1;
kneeID = 0; % right knee = 0 ; left knee = 1
% call the running pipeline that includes the GS rotation and translation
% adjustments
[T_FemTib_Traj,n] = modelRun(scale,kneeID, T_TibFem, T_FemTib)

T= cell(n,1);
for i = 1:n
   T{i} = T_tibground*T_FemTib_Traj{i}; % T = Fem wrt ground
end

%% plot
for i = 1:n
    ux_femtib_traj = T{i}*[1000 0 0 1]';
    uy_femtib_traj = T{i}*[0 1000 0 1]';
    uz_femtib_traj = T{i}*[0 0 1000 1]';
    myplot(T{i}(1:3,4),ux_femtib_traj,uy_femtib_traj,uz_femtib_traj,'k','k','k','k',2)
    pause
end
