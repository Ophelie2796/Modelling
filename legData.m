%%
% OBJECTIVE: 
% Identify possible ways to define the tibia frame using a model STL file
% leg.
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
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);

%% METHOD 1 (ZYX):
% Identify and plot tibia
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);
% Key Definitions:
    % Z-axis: Defined as the line that intersects the midpoint of the
    % line between the medial and lateral points on the distal tibia bone near the
    % tibia pot and is perpendicular to the ML tibial plateau line.
    % Y-axis: Line between ML points of tibia plateau
    % X-axis: cross-product
[uz, uy, ux] = ML_ZYX(O_tib, M_tib, L_tib, O_distal_tib);
title('ZYX: Z intersects midpt of distal tib bone and perp to ML tib plateau')

%% METHOD 1b (ZXY): DEFRATE PAPER
% Identify and plot tibia
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);
% Key Definitions
    % Tibia Z-axis: Defined as the line that intersects the midpoint of the
    % line between the AP points on the tibia bone near the
    % tibia pot and is perpendicular to the AP tibial plateau line 
    % (that has been adjusted for this model).
    % Tibia X-axis: Line between AP line where the anterior point is
    % adjusted to be in line with the posterior point
    % Tibia Y-axis: cross product

% ADJUSTMENT: align anterior point to be in line with posterior point
A_tib(3) = P_tib(3);

[uz,ux,uy] = AP_ZXY(O_tib, A_tib, P_tib, O_distal_tib);
title({'Defrate CS: ZXY: Z intersects midpt of distal tib bone and perp to AP tib plateau',
    'Aligned A pt with P pt'})

%% METHOD 2 (ZYX and same plane):
% Identify and plot tibia
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);

% Key Definitions
    % Tibia Z-axis: Defined as the line that intersects the midpoint of the
    % line between the medial and lateral points on the tibia bone near the
    % tibia pot and is perpendicular to the ML tibial plateau line that has 
    % been adjusted to be on single plane.
    % Y-axis: Line between ML points of tibia plateau
    % X-axis: cross-product

% ADJUSTMENT: adjust the tibial plateau points so that they are on a plane
[L_tib_plane, M_tib_plane, A_tib_plane, P_tib_plane] = makePlane(L_tib, M_tib, A_tib, P_tib);

[uz, uy, ux] = ML_ZYX(O_tib, M_tib_plane, L_tib_plane, O_distal_tib);
title({'ZYX: Z intersects midpt of distal tib bone and perp to ML tib plateau',
    'Tibial plateau points adjusted to be on one plane'})

%% METHOD 2b (ZXY and same plane): DEFRATE PAPER
% Identify and plot tibia
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);
% Key Definitions
    % Tibia Z-axis: Defined as the line that intersects the midpoint of the
    % line between the AP points on the tibia bone near the
    % tibia pot and is perpendicular to the AP tibial plateau line that has 
    % been adjusted to be on single plane.
    % Y-axis: Line between AP points of tibia plateau
    % X-axis: cross-product

% ADJUSTMENT: adjust the tibial plateau points so that they are on a plane
[L_tib_plane, M_tib_plane, A_tib_plane, P_tib_plane] = makePlane(L_tib, M_tib, A_tib, P_tib);
[uz,ux,uy] = AP_ZXY(O_tib, A_tib_plane, P_tib_plane, O_distal_tib);
title({'Defrate CS: ZXY: Z intersects midpt of distal tib bone and perp to AP tib plateau', ...
    'Tibial plateau points adjusted to be on one plane'})

%% METHOD 3: 
%% MATH NEEDS TO CHANGE TO USE AP LINE RATHER THAN ML LINE SINCE I FIND THE X AXIS FIRST%%
% Identify and plot tibia
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);
% ADJUSTMENT: adjust the tibial plateau points so that they are on a plane
[L_tib_plane, M_tib_plane, A_tib_plane, P_tib_plane] = makePlane(L_tib, M_tib, A_tib, P_tib);

% Key Definitions
    % TIBIA Z-axis: Defined as line that intersects the origin of 
    % tib plateau and the origin (midpoint) of the distal tibia bone
    % X-axis: perpendicular to z-axis and intersects the anterior point
    % Y-axis: cross-product

z_vec = O_distal_tib-O_tib;
z_mag = sqrt(z_vec(1)^2 + z_vec(2)^2 + z_vec(3)^2) ;
uz = [z_vec(1)/z_mag z_vec(2)/z_mag z_vec(3)/z_mag];


E = O_distal_tib; 
F = O_tib; 
G = A_tib_plane; 
EF = F-E;
EG = G-E;
EF_mag = sqrt(EF(1)^2+EF(2)^2+EF(3)^2) ;
EG_mag = sqrt(EG(1)^2+EG(2)^2+EG(3)^2);
cos_angle = dot(EF,EG) / (EF_mag*EG_mag);
EH_mag = EG_mag*cos_angle;
percent = EH_mag/EF_mag;
EH_vec = EF*percent;

D = [E(1)+EH_vec(1) E(2)+EH_vec(2) E(3)+EH_vec(3)];
y_vec = G-D; 
y_mag = sqrt(y_vec(1)^2+y_vec(2)^2+y_vec(3)^2);
uy = [y_vec(1)/y_mag y_vec(2)/y_mag y_vec(3)/y_mag] ;

ux = cross(uz,uy);

    % Plot Method 3
    plot3([O_tib(1) O_tib(1)+1000*(ux(1))],[O_tib(2) O_tib(2)+1000*(ux(2))],[O_tib(3) O_tib(3)+1000*(ux(3))], 'm','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+1000*(uz(1))],[O_tib(2) O_tib(2)+1000*(uz(2))],[O_tib(3) O_tib(3)+1000*(uz(3))], 'm','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+1000*(uy(1))],[O_tib(2) O_tib(2)+1000*(uy(2))],[O_tib(3) O_tib(3)+1000*(uy(3))], 'm','LineWidth',2)
    title({'ZYX: z defined as line from origin of tibial plane to origin of distal tibia','X axis is perp to z and intersects anterior point'})

%% METHOD 4: Create tibial plane and find perpendicular 
% Identify and plot tibia
[L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z);

% ADJUSTMENT: adjust the tibial plateau points so that they are on a plane
[L_tib_plane, M_tib_plane, A_tib_plane, P_tib_plane] = makePlane(L_tib, M_tib, A_tib, P_tib);

y = L_tib_plane-M_tib_plane;
y_mag = sqrt(y(1)^2 + y(2)^2 + y(3)^2);
uy = [y(1)/y_mag y(2)/y_mag y(3)/y_mag];
x = A_tib_plane-P_tib_plane;
x_mag = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
ux = [x(1)/x_mag x(2)/x_mag x(3)/x_mag];

z = cross(x,y);
z_mag = sqrt(z(1)^2 + z(2)^2 + z(3)^2);
uz = [z(1)/z_mag z(2)/z_mag z(3)/z_mag];

    % Plot Method 4
    plot3([O_tib(1) O_tib(1)+1000*(uy(1))],[O_tib(2) O_tib(2)+1000*(uy(2))],[O_tib(3) O_tib(3)+1000*(uy(3))], 'r','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+1000*(ux(1))],[O_tib(2) O_tib(2)+1000*(ux(2))],[O_tib(3) O_tib(3)+1000*(ux(3))], 'r','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+1000*(uz(1))],[O_tib(2) O_tib(2)+1000*(uz(2))],[O_tib(3) O_tib(3)+1000*(uz(3))], 'r','LineWidth',2)
    title('YXZ: using tibial plane points')