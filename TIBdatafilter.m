function [L_tib, M_tib, A_tib, P_tib, O_tib, O_distal_tib] = TIBdatafilter(x,y,z)
%% Remove patella and fibula 
x_knee = x;
x_knee(6200:6750) = [];
y_knee = y;
y_knee(6200:6750) = [];
z_knee = z;
z_knee(6200:6750) = [];
Leg = [x_knee, y_knee, z_knee];

%% Plot leg without patella
figure;
plot3(x_knee,y_knee,z_knee,'k*')
axis equal
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')

%% Identify tibia: Filter points 0<z<400 
Knee = Leg(Leg(:,3)<400,:);
Knee = Knee(Knee(:,3)>0,:);
hold on
plot3(Knee(:,1),Knee(:,2),Knee(:,3),'r*')

%% Identify MLAP distal tibia bone
distal_tib = Leg(Leg(:,3)<-150,:);
distal_tib = distal_tib(distal_tib(:,3)>-210,:);
O_distal_tib = mean(distal_tib,1);
plot3(O_distal_tib(1),O_distal_tib(2),O_distal_tib(3),'r*')

%% Lateral point of tibia
x_tib = Knee(:,1);
y_tib = Knee(:,2);
z_tib = Knee(:,3);
lat_tib_x = min(x_tib);
lat_tib_ID = find(Knee(:,1) == lat_tib_x);
L_tib = Knee(lat_tib_ID,:);

%% Medial point of tibia
med_tib_x = max(x_tib);
med_tib_ID = find(Knee(:,1) == med_tib_x);
M_tib = Knee(med_tib_ID,:);

%% Posterior point of tibia
post_tib_y = max(y_tib);
post_tib_ID = find(Knee(:,2) == post_tib_y);
P_tib = Knee(post_tib_ID,:);


%% Anterior point of tibia
ant_tib_y = min(y_tib);
ant_tib_ID = find(Knee(:,2) == ant_tib_y);
A_tib = Knee(ant_tib_ID,:);
  

%% Tibia Origin
plat = [L_tib; M_tib; A_tib; P_tib];
O_tib = mean(plat,1);

   %% Plot points
    plot3(L_tib(1),L_tib(2),L_tib(3),'b*','LineWidth',3)
    plot3(M_tib(1),M_tib(2),M_tib(3),'b*','LineWidth',3)
    plot3(A_tib(1),A_tib(2),A_tib(3),'b*','LineWidth',3)
    plot3(P_tib(1),P_tib(2),P_tib(3),'b*','LineWidth',3)
    plot3(O_tib(1), O_tib(2),O_tib(3),'b*','LineWidth',3)