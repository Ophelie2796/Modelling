function [ux_fem, uy_fem, uz_fem, O_fem] = femFrame_YZX(x,y,z)
%% OBJECTIVE:
% Define femoral frame.
% Y axis: ML condyle points adjusted to align with center of rotation
% Z axis: Intersects origin of proximal femoral bone and perpendicular to
% y-axis.
% X axis: cross product.
% Origin: midpoint of ML points 

%% Remove patella and fibula 
x_knee = x;
x_knee(6200:6750) = [];
y_knee = y;
y_knee(6200:6750) = [];
z_knee = z;
z_knee(6200:6750) = [];
Leg = [x_knee, y_knee, z_knee];

% Identify femur: Filter points 400<z<700
Femur = Leg(Leg(:,3)<700 & Leg(:,3)>400,:);
x_fem = Femur(:,1);
y_fem = Femur(:,2);
z_fem = Femur(:,3);
plot3(x_fem,y_fem,z_fem,'b*')

%% Identify MLAP proximal femur bone
proximal_fem = Leg(Leg(:,3)>900 & Leg(:,3)<1200,:);
O_proximal_fem = mean(proximal_fem,1);
plot3(O_proximal_fem(1),O_proximal_fem(2),O_proximal_fem(3),'r*')

%% Lateral point of femur
L_fem_x = min(x_fem);
L_fem_ID = find(Femur(:,1)==L_fem_x);
L_fem = Femur(L_fem_ID,:);

%% Medial point of femur
M_fem_x = max(x_fem);
M_fem_ID = find(Femur(:,1)==M_fem_x);
M_fem = Femur(M_fem_ID,:);

%% Find model of FEA
FEA = [L_fem;M_fem];

plot3(L_fem(1),L_fem(2),L_fem(3),'r*','LineWidth',3)
plot3(M_fem(1),M_fem(2),M_fem(3),'r*','LineWidth',3)

% Femur y-axis 
y_vec = L_fem - M_fem;
y_mag = sqrt(y_vec(1)^2+y_vec(2)^2+y_vec(3)^2);
uy_fem = [y_vec(1)/y_mag y_vec(2)/y_mag y_vec(3)/y_mag];

% Femur origin: Midpoint between lateral and medial femoral condyle points
O_fem = (L_fem+M_fem)/2;

%% Define Z axis as line that is perpendicular to FEA and intersects the origin of the proximal femoral bone
ML = M_fem - L_fem;
prox_L = O_proximal_fem - L_fem;

% Find magnitudes
ML_mag = sqrt(ML(1)^2 + ML(2)^2 +ML(3)^2);
prox_L_mag = sqrt(prox_L(1)^2 +prox_L(2)^2+prox_L(3)^2);

% Angle between 2 lines 
cos_angle = dot(ML,prox_L)/(ML_mag*prox_L_mag);
perp_mag = prox_L_mag*cos_angle;
fem_pt = [L_fem(1) L_fem(2)-perp_mag L_fem(3)];
% percent = perp_mag/ML_mag;
% perp_vec = ML*percent;
%fem_pt = [L_fem(1)+perp_vec(1) L_fem(2)+perp_vec(2) L_fem(3)+perp_vec(3)];
z = O_proximal_fem - fem_pt;
z_mag = sqrt(z(1)^2 +z(2)^2+z(3)^2);
uz_fem = [z(1)/z_mag z(2)/z_mag z(3)/z_mag];

%% X axis
ux_fem = cross(uy_fem,uz_fem);

    plot3([O_fem(1) O_fem(1)+uz_fem(1)*1000], [O_fem(2) O_fem(2)+uz_fem(2)*1000], [O_fem(3) O_fem(3)+uz_fem(3)*1000], 'k','LineWidth',2)
    plot3([O_fem(1) O_fem(1)+uy_fem(1)*1000], [O_fem(2) O_fem(2)+uy_fem(2)*1000], [O_fem(3) O_fem(3)+uy_fem(3)*1000], 'k','LineWidth',2)
    plot3([O_fem(1) O_fem(1)+ux_fem(1)*1000], [O_fem(2) O_fem(2)+ux_fem(2)*1000], [O_fem(3) O_fem(3)+ux_fem(3)*1000], 'k','LineWidth',2)







