function [uz, ux, uy] = AP_ZXY(O_tib, A_tib, P_tib, O_distal_tib)

%% Objective
% Define a tibial frame.
% Z axis:Perpendicular to the AP line of the tib plateau and intersects the
% origin of the distal tibia bone. 
% X axis: AP line of tibial plateau.
% Y axis: cross product. 
% Origin: O_tib already defined in main code as average of MLAP tibial
% plateau points.

%% Reference for following math: https://www.quora.com/How-do-I-find-the-perpendicular-distance-between-a-point-and-a-line-in-3D

% Define AP line and distal to posterior point
AP = A_tib - P_tib;
P_distal = O_distal_tib - P_tib;

% Find magnitudes
AP_mag = sqrt(AP(1)^2 + AP(2)^2 + AP(3)^2);
Pdistal_mag = sqrt(P_distal(1)^2 + P_distal(2)^2 + P_distal(3)^2);

% Angle between the two lines
cos_angle = dot(AP, P_distal) / (AP_mag*Pdistal_mag);

perp_mag = Pdistal_mag*cos_angle;
percent = perp_mag/AP_mag;
perp_vec = AP*percent;
tib_pt = [P_tib(1)+perp_vec(1) P_tib(2)+perp_vec(2) P_tib(3)+perp_vec(3)];
z = O_distal_tib - tib_pt;
z_mag = sqrt(z(1)^2 + z(2)^2 + z(3)^2) ;
uz = [z(1)/z_mag z(2)/z_mag z(3)/z_mag];

% Tibia x-axis: AP points
x = A_tib - P_tib;
x_mag = sqrt(x(1)^2 + x(2)^2 + x(3)^2) ;
ux = [x(1)/x_mag x(2)/x_mag x(3)/x_mag] ;

% Tibia y-axis 
uy = cross(uz,ux);

    % Plot Frame
    plot3([O_tib(1) O_tib(1)+uz(1)*1000], [O_tib(2) O_tib(2)+uz(2)*1000], [O_tib(3) O_tib(3)+uz(3)*1000], 'k','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+uy(1)*1000], [O_tib(2) O_tib(2)+uy(2)*1000], [O_tib(3) O_tib(3)+uy(3)*1000], 'k','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+ux(1)*1000], [O_tib(2) O_tib(2)+ux(2)*1000], [O_tib(3) O_tib(3)+ux(3)*1000], 'k','LineWidth',2)
