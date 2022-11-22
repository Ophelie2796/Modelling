function [uz, uy, ux] = ML_ZYX(O_tib, M_tib, L_tib, O_distal_tib)

%% Objective
% Define a tibial frame where the Z axis is defined first as the line that
% is perpendicular to the ML line of the tib plateau and intersects the
% origin of the distal tibia bone. 
% Then the Y axis is defines as the ML line.
% The X axis is the cross product. 

%% Reference for following math: https://www.quora.com/How-do-I-find-the-perpendicular-distance-between-a-point-and-a-line-in-3D

% Define ML and line from distal to lateral point
ML = M_tib - L_tib ;
L_distal = O_distal_tib - L_tib;

% Find magnitudes
ML_mag = sqrt(ML(1)^2 + ML(2)^2 + ML(3)^2);
Ldistal_mag = sqrt(L_distal(1)^2 + L_distal(2)^2 + L_distal(3)^2);

% Angle between two lines
cos_angle = dot(ML, L_distal) / (ML_mag*Ldistal_mag);

perp_mag = Ldistal_mag*cos_angle;
percent = perp_mag/ML_mag;
perp_vec = ML*percent;
tib_pt = [ L_tib(1)+perp_vec(1) L_tib(2)+perp_vec(2) L_tib(3)+perp_vec(3)];
z = O_distal_tib - tib_pt ; 
z_mag = sqrt(z(1)^2 + z(2)^2 + z(3)^2) ;
uz = [z(1)/z_mag z(2)/z_mag z(3)/z_mag];

% Tibia y-axis: medial and lateral points
y = L_tib - M_tib;
y_mag = sqrt(y(1)^2 + y(2)^2 + y(3)^2) ;
uy = [y(1)/y_mag y(2)/y_mag y(3)/y_mag] ;

% Tibia x-axis 
ux = cross(uy,uz);

    % Plot Frame
    plot3([O_tib(1) O_tib(1)+uz(1)*1000], [O_tib(2) O_tib(2)+uz(2)*1000], [O_tib(3) O_tib(3)+uz(3)*1000], 'b','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+uy(1)*1000], [O_tib(2) O_tib(2)+uy(2)*1000], [O_tib(3) O_tib(3)+uy(3)*1000], 'b','LineWidth',2)
    plot3([O_tib(1) O_tib(1)+ux(1)*1000], [O_tib(2) O_tib(2)+ux(2)*1000], [O_tib(3) O_tib(3)+ux(3)*1000], 'b','LineWidth',2)

