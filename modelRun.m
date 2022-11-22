function [T_FemTib_Traj,n] = modelRun(scale,kneeID, T_TibFem, T_FemTib)
% Important notation

% Purpose:
% (A) ADJUSTMENT OF COORDINATE SYSTEMS
% This script finds the adjusted coordinate frames that match the coordinate
% systems published in In-Vivo Measurement of Dynamic Joint Motion
% Using High Speed Biplane Radiography and CT: Application to Canine ACL
% Deficiency by Tashman, Anderst 
    % TCO = tibial correct orientation that matches the published CS
    % FCO = femoral coorect orientation that matches the published CS

% (B) KINEMATIC ROTATIONS
% The rotations are calculated following the grood suntay convention. 
    % Flexion/extension = about the FCO y-axis 
    % Internal/external = about the TCO z-axis (long axis) 
    % Ad/Abduction = about floating axis (mutually perpendicular to the
    % first two axes of FCO and TCO)

% (C) KINEMATIC TRANSLATIONS    
% The translations are calculated using the vector from the femur origin,
% loacated at the midpoint of the femoral condyles, to the tibia origin,
% located at the midpoint of the most medial and lateral borders of the
% tibial plateau, and expressed in the TCO coordinate system. 

%% Creating tibia as a base frame
% coordinates defining the axis and these are what I want to rotate
x = [1 0 0 1]';
y = [0 1 0 1]';
z = [0 0 1 1]';

%% (A) ADJUSTMENT
% rotate tibia about x axis to correct coordinate system
ux_tibAxis = [1 0 0];
% degree of adjustment needed
tib_adjust_angle = 0;
T_TCOTib = rotx(tib_adjust_angle*pi/180);
T_TCOTib(:,4) = [0;0;0];
T_TCOTib(4,:) = [0 0 0 1];

% rotate femur about x axis 
ux_femAxis = [1 0 0];
% degree of adjustment needed
fem_adjust_angle = 0; % positive = femur rotates varus
T_FCOF = rotx(fem_adjust_angle*pi/180);
T_FCOF(:,4) = [0;0;0];
T_FCOF(4,:) = [0 0 0 1];


% Drop Jump data
load('..\frames\IntRotHopKinematics\IntRotHopData_men.csv')
load('..\frames\IntRotHopKinematics\IntRotHopData_women.csv') 
    
% Identify columns: positive motion of femur wrt tibia
setting =  IntRotHopData_women(:,1); 
flex    =  IntRotHopData_women(:,2);    %  % Flexion of fem wrt tib: positive
valgus_angle  =  IntRotHopData_women(:,3);         % Valgus rot of fem wrt tib: positive
int_angle     = IntRotHopData_women(:,4);        % Ext rot of fem wrt tib: positive (corresponds with interior rotation of tib wrt fem)
ant     =  scale*IntRotHopData_women(:,5);   % Anterior translation of tibia wrt femur: positive
prox    =  scale*IntRotHopData_women(:,6);        % proximal/distal distance remains zero
lat     =  scale*IntRotHopData_women(:,7); % Lateral translation of tib wrt femur: positive
%zeros(100,1);  %
n = length(IntRotHopData_women);
%% LEFT KNEE: ADJUSTMENTS
if kneeID == 1
    int_angle = -int_angle;
    flexion_angle = -flex;
    ant = -ant;
elseif kneeID == 0
    int_angle = int_angle;
    flexion_angle = flex;
    ant = ant;
end

%% DEFINE KINEMATIC TRAJECTORY 
T_FemTib_Traj = cell(n,1);
% create for loop to iterate through the entire trajectory
for i = 1:n
%    i=1;
%% (B) KINEMATIC ROTATIONS
% Internal rotation of the OG tibia about TCO z axis
% Axis of rotation
uz_TCO = T_TCOTib*[0;0;1;1];
% Internal rotation = negative 
curr_int_angle = -int_angle(i);
[ux_intTib] = rotAboutTibAxis([0 0 0], x(1:3)', uz_TCO(1:3)', curr_int_angle);
[uy_intTib] = rotAboutTibAxis([0 0 0]', y(1:3)', uz_TCO(1:3)', curr_int_angle);
[uz_intTib] = rotAboutTibAxis([0 0 0]', z(1:3)', uz_TCO(1:3)', curr_int_angle);
ux_intTib = ux_intTib/norm(ux_intTib);
uy_intTib = uy_intTib/norm(uy_intTib);
uz_intTib = uz_intTib/norm(uz_intTib);
T_TintT = [ux_intTib(1) uy_intTib(1) uz_intTib(1); ux_intTib(2) uy_intTib(2) uz_intTib(2); ux_intTib(3) uy_intTib(3) uz_intTib(3)];
% T_TintTCO = rotz(curr_int_angle*pi/180);
 T_TintT(:,4) = [0; 0; 0];
 T_TintT(4,:) = [0 0 0 1];

% Flexion of OG femur about FCO FEA 
% axis of rotation = FEA
%uy_flexAxis = [0 1 0];
uy_flexAxis = T_FCOF*[0;1;0;1];

% Desired flexion angle. Positive = flexion of femur wrt tibia
curr_flex_angle = flexion_angle(i);
[ux_fem_flex] = rotAboutTibAxis([0 0 0], x(1:3)', uy_flexAxis(1:3)', curr_flex_angle);
[uy_fem_flex] = rotAboutTibAxis([0 0 0], y(1:3)', uy_flexAxis(1:3)', curr_flex_angle);
[uz_fem_flex] = rotAboutTibAxis([0 0 0], z(1:3)', uy_flexAxis(1:3)', curr_flex_angle);

% Var/val rotation of OG femur about e2 created by the cross product of FCO
% y-axis and TCO z-axis
% Find TCO z axis wrt OG femur because that is the
% frame we are working in
T_TCOF = T_TibFem*T_TCOTib;
uz_TCOF = T_TCOF(1:3,3);
% Axis of rotation = e2
e2 = cross(T_FCOF(1:3,2),uz_TCOF);
ue2 = e2/norm(e2);

% desired valgus angle. positive = valgus of femur wrt tibia
curr_val_angle = valgus_angle(i);%-valgus_angle(1);
[ux_fem_vv] = rotAboutTibAxis([0 0 0], ux_fem_flex(1:3), ue2(1:3)', curr_val_angle);
[uy_fem_vv] = rotAboutTibAxis([0 0 0], uy_fem_flex(1:3), ue2(1:3)', curr_val_angle);
[uz_fem_vv] = rotAboutTibAxis([0 0 0], uz_fem_flex(1:3), ue2(1:3)', curr_val_angle);
% unit vectors
ux_fem_vv = ux_fem_vv/norm(ux_fem_vv);
uy_fem_vv = uy_fem_vv/norm(uy_fem_vv);
uz_fem_vv = uz_fem_vv/norm(uz_fem_vv);

T_FFVF = [ux_fem_vv(1) uy_fem_vv(1) uz_fem_vv(1); ux_fem_vv(2) uy_fem_vv(2) uz_fem_vv(2); ux_fem_vv(3) uy_fem_vv(3) uz_fem_vv(3)];
T_FFVF(:,4) = [0 0 0];
T_FFVF(4,:) = [0 0 0 1];
T_FFFV = inv(T_FFVF);

% Final rotation of femur wrt tibia
% T_TintFCO = T_TCOFCO*T_TintT;
% % T_FFVTint = T_FFFV*T_TintFCO;
% % Final rotation of tibia wrt femur
% T_TintFFV = inv(T_FFVTint);

T_FFVTint = inv(T_TintT)*T_FemTib*T_FCOF*T_FFVF;
T_TintFFV = inv(T_FFVTint);

%% (C) TRANSLATIONS
% test offset
ant_extrapolated = 5.2039;
prox_extrapolated = -24.0902;
offset_datazerodiginTIB = [-ant_extrapolated;0;prox_extrapolated]-T_FemTib(1:3,4);

translation_x = -(ant(i)+offset_datazerodiginTIB(1))*(T_TCOTib(1:3,1));
translation_y = -(lat(i)+offset_datazerodiginTIB(2))*(T_TCOTib(1:3,2));
translation_z = (prox(i)-offset_datazerodiginTIB(3))*(T_TCOTib(1:3,3));

translation = translation_x + translation_y + translation_z ;

R_FemTibFinal = T_FFVTint(1:3,1:3);
R_FemTibFinal(:,4) = translation;
R_FemTibFinal(4,:) = [0 0 0 1];
T_FemTibFinal = R_FemTibFinal;
T_TibFemFinal = inv(T_FemTibFinal);

T_FemTib_Traj{i} = T_FemTibFinal;

end